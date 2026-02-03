"""
Alert Notification System for River Level Monitoring

Implements multi-channel alert system for threshold exceedance:
- SMS notifications via Twilio
- Email alerts via SMTP
- Webhook support for third-party integrations
- Configurable thresholds (warning, critical, emergency)
- Alert rate limiting to prevent spam
- Integration with the existing monitoring pipeline

References:
- Rahman & Ahmed (2020) - Dual-alerting (buzzer + SMS)
- Standard IoT alert system patterns

Author: Klaus Dieter Kupper
Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
"""

import os
import json
import time
import logging
import smtplib
import requests
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from typing import Dict, List, Optional, Callable, Any
from enum import Enum
from collections import defaultdict

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class AlertLevel(Enum):
    """Alert severity levels"""
    INFO = 1
    WARNING = 2
    CRITICAL = 3
    EMERGENCY = 4


class AlertType(Enum):
    """Types of alerts"""
    WATER_LEVEL_HIGH = "water_level_high"
    WATER_LEVEL_LOW = "water_level_low"
    RAPID_CHANGE = "rapid_change"
    SENSOR_ERROR = "sensor_error"
    BATTERY_LOW = "battery_low"
    CONNECTION_LOST = "connection_lost"
    DATA_GAP = "data_gap"
    SYSTEM_ERROR = "system_error"


@dataclass
class AlertThresholds:
    """Configurable alert thresholds"""
    # Water level thresholds (cm)
    warning_level_cm: float = 50.0
    critical_level_cm: float = 100.0
    emergency_level_cm: float = 150.0

    # Rate of change thresholds (cm/hour)
    warning_rate_cm_h: float = 5.0
    critical_rate_cm_h: float = 10.0
    emergency_rate_cm_h: float = 20.0

    # Battery thresholds (%)
    battery_warning: float = 30.0
    battery_critical: float = 15.0

    # Data gap threshold (minutes)
    data_gap_warning_min: float = 30.0
    data_gap_critical_min: float = 60.0


@dataclass
class Alert:
    """Alert data structure"""
    alert_id: str
    alert_type: AlertType
    level: AlertLevel
    message: str
    timestamp: datetime
    sensor_id: str
    value: float
    threshold: float
    metadata: Dict[str, Any] = field(default_factory=dict)
    acknowledged: bool = False
    sent_channels: List[str] = field(default_factory=list)


@dataclass
class RateLimitConfig:
    """Rate limiting configuration"""
    # Minimum interval between same alerts (seconds)
    min_interval_seconds: int = 300  # 5 minutes

    # Maximum alerts per hour per sensor
    max_alerts_per_hour: int = 12

    # Cooldown period after emergency (minutes)
    emergency_cooldown_min: int = 15

    # Allow escalation through rate limit
    allow_escalation: bool = True


class AlertChannel(ABC):
    """Abstract base class for alert channels"""

    @abstractmethod
    def send(self, alert: Alert) -> bool:
        """Send alert through this channel"""
        pass

    @abstractmethod
    def is_configured(self) -> bool:
        """Check if channel is properly configured"""
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """Channel name for logging"""
        pass


class SMSChannel(AlertChannel):
    """SMS notification channel using Twilio"""

    def __init__(self, account_sid: str = None, auth_token: str = None,
                 from_number: str = None, to_numbers: List[str] = None):
        self.account_sid = account_sid or os.getenv('TWILIO_ACCOUNT_SID')
        self.auth_token = auth_token or os.getenv('TWILIO_AUTH_TOKEN')
        self.from_number = from_number or os.getenv('TWILIO_FROM_NUMBER')
        self.to_numbers = to_numbers or []

        if os.getenv('ALERT_SMS_NUMBERS'):
            self.to_numbers = os.getenv('ALERT_SMS_NUMBERS').split(',')

    @property
    def name(self) -> str:
        return "SMS"

    def is_configured(self) -> bool:
        return all([
            self.account_sid,
            self.auth_token,
            self.from_number,
            len(self.to_numbers) > 0
        ])

    def send(self, alert: Alert) -> bool:
        if not self.is_configured():
            logger.warning("SMS channel not configured")
            return False

        try:
            from twilio.rest import Client
            client = Client(self.account_sid, self.auth_token)

            # Format SMS message (keep it short)
            sms_message = self._format_sms(alert)

            success = True
            for number in self.to_numbers:
                try:
                    message = client.messages.create(
                        body=sms_message,
                        from_=self.from_number,
                        to=number.strip()
                    )
                    logger.info(f"SMS sent to {number}: {message.sid}")
                except Exception as e:
                    logger.error(f"Failed to send SMS to {number}: {e}")
                    success = False

            return success

        except ImportError:
            logger.error("Twilio library not installed. Run: pip install twilio")
            return False
        except Exception as e:
            logger.error(f"SMS send error: {e}")
            return False

    def _format_sms(self, alert: Alert) -> str:
        """Format alert for SMS (max 160 chars preferred)"""
        level_emoji = {
            AlertLevel.INFO: "ℹ️",
            AlertLevel.WARNING: "⚠️",
            AlertLevel.CRITICAL: "🚨",
            AlertLevel.EMERGENCY: "🆘"
        }
        emoji = level_emoji.get(alert.level, "")

        msg = f"{emoji} {alert.level.name}: {alert.message}"
        if len(msg) > 155:
            msg = msg[:152] + "..."
        return msg


class EmailChannel(AlertChannel):
    """Email notification channel using SMTP"""

    def __init__(self, smtp_host: str = None, smtp_port: int = 587,
                 username: str = None, password: str = None,
                 from_email: str = None, to_emails: List[str] = None,
                 use_tls: bool = True):
        self.smtp_host = smtp_host or os.getenv('SMTP_HOST', 'smtp.gmail.com')
        self.smtp_port = smtp_port or int(os.getenv('SMTP_PORT', '587'))
        self.username = username or os.getenv('SMTP_USERNAME')
        self.password = password or os.getenv('SMTP_PASSWORD')
        self.from_email = from_email or os.getenv('ALERT_FROM_EMAIL')
        self.to_emails = to_emails or []
        self.use_tls = use_tls

        if os.getenv('ALERT_TO_EMAILS'):
            self.to_emails = os.getenv('ALERT_TO_EMAILS').split(',')

    @property
    def name(self) -> str:
        return "Email"

    def is_configured(self) -> bool:
        return all([
            self.smtp_host,
            self.username,
            self.password,
            self.from_email,
            len(self.to_emails) > 0
        ])

    def send(self, alert: Alert) -> bool:
        if not self.is_configured():
            logger.warning("Email channel not configured")
            return False

        try:
            msg = MIMEMultipart('alternative')
            msg['Subject'] = self._format_subject(alert)
            msg['From'] = self.from_email
            msg['To'] = ', '.join(self.to_emails)

            # Plain text version
            text_content = self._format_text(alert)
            msg.attach(MIMEText(text_content, 'plain'))

            # HTML version
            html_content = self._format_html(alert)
            msg.attach(MIMEText(html_content, 'html'))

            # Send email
            with smtplib.SMTP(self.smtp_host, self.smtp_port) as server:
                if self.use_tls:
                    server.starttls()
                server.login(self.username, self.password)
                server.sendmail(self.from_email, self.to_emails, msg.as_string())

            logger.info(f"Email sent to {self.to_emails}")
            return True

        except Exception as e:
            logger.error(f"Email send error: {e}")
            return False

    def _format_subject(self, alert: Alert) -> str:
        return f"[{alert.level.name}] River Monitor Alert: {alert.alert_type.value}"

    def _format_text(self, alert: Alert) -> str:
        return f"""
River Level Monitoring Alert
============================

Level: {alert.level.name}
Type: {alert.alert_type.value}
Sensor: {alert.sensor_id}
Time: {alert.timestamp.strftime('%Y-%m-%d %H:%M:%S')}

{alert.message}

Current Value: {alert.value:.2f}
Threshold: {alert.threshold:.2f}

---
River Level Monitoring System
        """

    def _format_html(self, alert: Alert) -> str:
        color_map = {
            AlertLevel.INFO: '#17a2b8',
            AlertLevel.WARNING: '#ffc107',
            AlertLevel.CRITICAL: '#dc3545',
            AlertLevel.EMERGENCY: '#721c24'
        }
        color = color_map.get(alert.level, '#6c757d')

        return f"""
        <html>
        <body style="font-family: Arial, sans-serif;">
            <div style="background-color: {color}; color: white; padding: 15px; border-radius: 5px;">
                <h2 style="margin: 0;">{alert.level.name}: {alert.alert_type.value}</h2>
            </div>
            <div style="padding: 20px;">
                <p><strong>Sensor:</strong> {alert.sensor_id}</p>
                <p><strong>Time:</strong> {alert.timestamp.strftime('%Y-%m-%d %H:%M:%S')}</p>
                <p><strong>Message:</strong> {alert.message}</p>
                <table style="border-collapse: collapse; margin-top: 15px;">
                    <tr>
                        <td style="padding: 8px; border: 1px solid #ddd;"><strong>Current Value</strong></td>
                        <td style="padding: 8px; border: 1px solid #ddd;">{alert.value:.2f}</td>
                    </tr>
                    <tr>
                        <td style="padding: 8px; border: 1px solid #ddd;"><strong>Threshold</strong></td>
                        <td style="padding: 8px; border: 1px solid #ddd;">{alert.threshold:.2f}</td>
                    </tr>
                </table>
            </div>
            <div style="color: #6c757d; font-size: 12px; padding: 10px;">
                River Level Monitoring System
            </div>
        </body>
        </html>
        """


class WebhookChannel(AlertChannel):
    """Webhook notification channel for third-party integrations"""

    def __init__(self, webhook_url: str = None, headers: Dict[str, str] = None,
                 method: str = "POST", timeout: int = 10):
        self.webhook_url = webhook_url or os.getenv('ALERT_WEBHOOK_URL')
        self.headers = headers or {'Content-Type': 'application/json'}
        self.method = method
        self.timeout = timeout

        # Support for custom headers from environment
        if os.getenv('ALERT_WEBHOOK_AUTH'):
            self.headers['Authorization'] = os.getenv('ALERT_WEBHOOK_AUTH')

    @property
    def name(self) -> str:
        return "Webhook"

    def is_configured(self) -> bool:
        return bool(self.webhook_url)

    def send(self, alert: Alert) -> bool:
        if not self.is_configured():
            logger.warning("Webhook channel not configured")
            return False

        try:
            payload = self._format_payload(alert)

            if self.method.upper() == "POST":
                response = requests.post(
                    self.webhook_url,
                    json=payload,
                    headers=self.headers,
                    timeout=self.timeout
                )
            else:
                response = requests.get(
                    self.webhook_url,
                    params=payload,
                    headers=self.headers,
                    timeout=self.timeout
                )

            if response.status_code in [200, 201, 202, 204]:
                logger.info(f"Webhook delivered: {response.status_code}")
                return True
            else:
                logger.error(f"Webhook failed: {response.status_code} - {response.text}")
                return False

        except Exception as e:
            logger.error(f"Webhook error: {e}")
            return False

    def _format_payload(self, alert: Alert) -> Dict:
        return {
            "alert_id": alert.alert_id,
            "type": alert.alert_type.value,
            "level": alert.level.name,
            "message": alert.message,
            "timestamp": alert.timestamp.isoformat(),
            "sensor_id": alert.sensor_id,
            "value": alert.value,
            "threshold": alert.threshold,
            "metadata": alert.metadata
        }


class AlertManager:
    """
    Main alert management system

    Handles:
    - Alert generation based on thresholds
    - Rate limiting
    - Multi-channel delivery
    - Alert history and acknowledgment
    """

    def __init__(self, thresholds: AlertThresholds = None,
                 rate_limit: RateLimitConfig = None):
        self.thresholds = thresholds or AlertThresholds()
        self.rate_limit = rate_limit or RateLimitConfig()

        self.channels: List[AlertChannel] = []
        self.alert_history: List[Alert] = []
        self.last_alert_time: Dict[str, datetime] = defaultdict(lambda: datetime.min)
        self.alerts_per_hour: Dict[str, List[datetime]] = defaultdict(list)

        self._alert_counter = 0

    def add_channel(self, channel: AlertChannel) -> None:
        """Add a notification channel"""
        if channel.is_configured():
            self.channels.append(channel)
            logger.info(f"Added alert channel: {channel.name}")
        else:
            logger.warning(f"Channel {channel.name} not configured, skipping")

    def process_reading(self, sensor_id: str, water_level_cm: float,
                        rate_of_change_cm_h: float = 0.0,
                        battery_percent: float = 100.0,
                        timestamp: datetime = None) -> List[Alert]:
        """
        Process a sensor reading and generate alerts if needed

        Returns list of alerts that were sent
        """
        timestamp = timestamp or datetime.now()
        alerts = []

        # Check water level thresholds
        level_alert = self._check_water_level(sensor_id, water_level_cm, timestamp)
        if level_alert:
            alerts.append(level_alert)

        # Check rate of change
        rate_alert = self._check_rate_of_change(sensor_id, rate_of_change_cm_h, timestamp)
        if rate_alert:
            alerts.append(rate_alert)

        # Check battery
        battery_alert = self._check_battery(sensor_id, battery_percent, timestamp)
        if battery_alert:
            alerts.append(battery_alert)

        # Send alerts through channels
        for alert in alerts:
            self._send_alert(alert)

        return alerts

    def check_data_gap(self, sensor_id: str, last_reading_time: datetime) -> Optional[Alert]:
        """Check for data gaps (missing readings)"""
        now = datetime.now()
        gap_minutes = (now - last_reading_time).total_seconds() / 60

        if gap_minutes >= self.thresholds.data_gap_critical_min:
            return self._create_and_send_alert(
                AlertType.DATA_GAP,
                AlertLevel.CRITICAL,
                sensor_id,
                gap_minutes,
                self.thresholds.data_gap_critical_min,
                f"No data from {sensor_id} for {gap_minutes:.0f} minutes"
            )
        elif gap_minutes >= self.thresholds.data_gap_warning_min:
            return self._create_and_send_alert(
                AlertType.DATA_GAP,
                AlertLevel.WARNING,
                sensor_id,
                gap_minutes,
                self.thresholds.data_gap_warning_min,
                f"Data gap detected: {gap_minutes:.0f} minutes since last reading"
            )

        return None

    def send_custom_alert(self, alert_type: AlertType, level: AlertLevel,
                          sensor_id: str, message: str,
                          value: float = 0.0, threshold: float = 0.0) -> Alert:
        """Send a custom alert"""
        return self._create_and_send_alert(
            alert_type, level, sensor_id, value, threshold, message
        )

    def acknowledge_alert(self, alert_id: str) -> bool:
        """Acknowledge an alert"""
        for alert in self.alert_history:
            if alert.alert_id == alert_id:
                alert.acknowledged = True
                logger.info(f"Alert {alert_id} acknowledged")
                return True
        return False

    def get_active_alerts(self, sensor_id: str = None) -> List[Alert]:
        """Get unacknowledged alerts"""
        alerts = [a for a in self.alert_history if not a.acknowledged]
        if sensor_id:
            alerts = [a for a in alerts if a.sensor_id == sensor_id]
        return alerts

    def get_alert_history(self, hours: int = 24, sensor_id: str = None) -> List[Alert]:
        """Get alert history for specified period"""
        cutoff = datetime.now() - timedelta(hours=hours)
        alerts = [a for a in self.alert_history if a.timestamp >= cutoff]
        if sensor_id:
            alerts = [a for a in alerts if a.sensor_id == sensor_id]
        return alerts

    def _check_water_level(self, sensor_id: str, level_cm: float,
                           timestamp: datetime) -> Optional[Alert]:
        """Check water level thresholds"""
        if level_cm >= self.thresholds.emergency_level_cm:
            return self._create_alert(
                AlertType.WATER_LEVEL_HIGH,
                AlertLevel.EMERGENCY,
                sensor_id, level_cm,
                self.thresholds.emergency_level_cm,
                f"EMERGENCY: Water level at {level_cm:.1f}cm - Immediate action required!",
                timestamp
            )
        elif level_cm >= self.thresholds.critical_level_cm:
            return self._create_alert(
                AlertType.WATER_LEVEL_HIGH,
                AlertLevel.CRITICAL,
                sensor_id, level_cm,
                self.thresholds.critical_level_cm,
                f"Critical water level: {level_cm:.1f}cm",
                timestamp
            )
        elif level_cm >= self.thresholds.warning_level_cm:
            return self._create_alert(
                AlertType.WATER_LEVEL_HIGH,
                AlertLevel.WARNING,
                sensor_id, level_cm,
                self.thresholds.warning_level_cm,
                f"Water level elevated: {level_cm:.1f}cm",
                timestamp
            )
        return None

    def _check_rate_of_change(self, sensor_id: str, rate_cm_h: float,
                              timestamp: datetime) -> Optional[Alert]:
        """Check rate of change thresholds"""
        abs_rate = abs(rate_cm_h)

        if abs_rate >= self.thresholds.emergency_rate_cm_h:
            return self._create_alert(
                AlertType.RAPID_CHANGE,
                AlertLevel.EMERGENCY,
                sensor_id, rate_cm_h,
                self.thresholds.emergency_rate_cm_h,
                f"EMERGENCY: Rapid level change {rate_cm_h:+.1f} cm/hour!",
                timestamp
            )
        elif abs_rate >= self.thresholds.critical_rate_cm_h:
            return self._create_alert(
                AlertType.RAPID_CHANGE,
                AlertLevel.CRITICAL,
                sensor_id, rate_cm_h,
                self.thresholds.critical_rate_cm_h,
                f"Rapid level change detected: {rate_cm_h:+.1f} cm/hour",
                timestamp
            )
        elif abs_rate >= self.thresholds.warning_rate_cm_h:
            return self._create_alert(
                AlertType.RAPID_CHANGE,
                AlertLevel.WARNING,
                sensor_id, rate_cm_h,
                self.thresholds.warning_rate_cm_h,
                f"Water level changing: {rate_cm_h:+.1f} cm/hour",
                timestamp
            )
        return None

    def _check_battery(self, sensor_id: str, battery_pct: float,
                       timestamp: datetime) -> Optional[Alert]:
        """Check battery thresholds"""
        if battery_pct <= self.thresholds.battery_critical:
            return self._create_alert(
                AlertType.BATTERY_LOW,
                AlertLevel.CRITICAL,
                sensor_id, battery_pct,
                self.thresholds.battery_critical,
                f"Critical battery level: {battery_pct:.0f}%",
                timestamp
            )
        elif battery_pct <= self.thresholds.battery_warning:
            return self._create_alert(
                AlertType.BATTERY_LOW,
                AlertLevel.WARNING,
                sensor_id, battery_pct,
                self.thresholds.battery_warning,
                f"Low battery: {battery_pct:.0f}%",
                timestamp
            )
        return None

    def _create_alert(self, alert_type: AlertType, level: AlertLevel,
                      sensor_id: str, value: float, threshold: float,
                      message: str, timestamp: datetime = None) -> Alert:
        """Create an alert object"""
        self._alert_counter += 1
        return Alert(
            alert_id=f"ALR-{self._alert_counter:06d}",
            alert_type=alert_type,
            level=level,
            message=message,
            timestamp=timestamp or datetime.now(),
            sensor_id=sensor_id,
            value=value,
            threshold=threshold
        )

    def _create_and_send_alert(self, alert_type: AlertType, level: AlertLevel,
                                sensor_id: str, value: float, threshold: float,
                                message: str) -> Alert:
        """Create and send an alert"""
        alert = self._create_alert(alert_type, level, sensor_id, value, threshold, message)
        self._send_alert(alert)
        return alert

    def _send_alert(self, alert: Alert) -> bool:
        """Send alert through all channels with rate limiting"""
        # Check rate limit
        if not self._should_send_alert(alert):
            logger.debug(f"Alert {alert.alert_id} rate-limited")
            return False

        # Send through all channels
        sent_any = False
        for channel in self.channels:
            try:
                if channel.send(alert):
                    alert.sent_channels.append(channel.name)
                    sent_any = True
            except Exception as e:
                logger.error(f"Failed to send via {channel.name}: {e}")

        if sent_any:
            self.alert_history.append(alert)
            self._update_rate_limit_tracking(alert)
            logger.info(f"Alert sent: {alert.alert_id} - {alert.message}")

        return sent_any

    def _should_send_alert(self, alert: Alert) -> bool:
        """Check if alert should be sent based on rate limiting"""
        key = f"{alert.sensor_id}:{alert.alert_type.value}"
        now = datetime.now()

        # Check minimum interval
        last_time = self.last_alert_time[key]
        seconds_since_last = (now - last_time).total_seconds()

        if seconds_since_last < self.rate_limit.min_interval_seconds:
            # Allow escalation (higher level alert)
            if self.rate_limit.allow_escalation:
                last_alerts = [a for a in self.alert_history
                              if a.sensor_id == alert.sensor_id
                              and a.alert_type == alert.alert_type
                              and (now - a.timestamp).total_seconds() < self.rate_limit.min_interval_seconds]
                if last_alerts:
                    last_level = max(a.level.value for a in last_alerts)
                    if alert.level.value > last_level:
                        return True  # Allow escalation
            return False

        # Check alerts per hour
        hour_ago = now - timedelta(hours=1)
        self.alerts_per_hour[key] = [t for t in self.alerts_per_hour[key] if t > hour_ago]

        if len(self.alerts_per_hour[key]) >= self.rate_limit.max_alerts_per_hour:
            return False

        return True

    def _update_rate_limit_tracking(self, alert: Alert) -> None:
        """Update rate limiting tracking"""
        key = f"{alert.sensor_id}:{alert.alert_type.value}"
        now = datetime.now()

        self.last_alert_time[key] = now
        self.alerts_per_hour[key].append(now)


def create_default_alert_manager() -> AlertManager:
    """Create an AlertManager with default channels from environment"""
    manager = AlertManager()

    # Add SMS channel if configured
    sms = SMSChannel()
    manager.add_channel(sms)

    # Add Email channel if configured
    email = EmailChannel()
    manager.add_channel(email)

    # Add Webhook channel if configured
    webhook = WebhookChannel()
    manager.add_channel(webhook)

    return manager


# Example usage
if __name__ == "__main__":
    # Create alert manager with custom thresholds
    thresholds = AlertThresholds(
        warning_level_cm=50.0,
        critical_level_cm=100.0,
        emergency_level_cm=150.0
    )

    manager = AlertManager(thresholds=thresholds)

    # Add webhook for testing (doesn't require external service)
    webhook = WebhookChannel(webhook_url="https://webhook.site/test")
    manager.add_channel(webhook)

    # Simulate sensor reading
    alerts = manager.process_reading(
        sensor_id="SENSOR-001",
        water_level_cm=120.0,  # Above critical threshold
        rate_of_change_cm_h=8.0,
        battery_percent=25.0
    )

    print(f"Generated {len(alerts)} alerts:")
    for alert in alerts:
        print(f"  - [{alert.level.name}] {alert.message}")

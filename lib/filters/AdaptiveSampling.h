/**
 * AdaptiveSampling.h - Adaptive Transmission Interval Calculator
 *
 * Implements intelligent adaptive sampling for river level monitoring.
 * Adjusts transmission intervals based on:
 * - Rate of water level change
 * - Current water level relative to critical thresholds
 * - Battery status for power conservation
 * - Time of day (optional diurnal patterns)
 *
 * Based on the thesis requirement:
 * "If the river rises >5cm/min, transmit every 1 minute. If stable, transmit every 30 minutes"
 *
 * References:
 * - Ragnoli et al. (2020): Adaptive duty cycling for smart sensor networks
 * - Casals et al. (2017): Energy optimization in LoRaWAN networks
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef ADAPTIVE_SAMPLING_H
#define ADAPTIVE_SAMPLING_H

#include <Arduino.h>

/**
 * Sampling mode enumeration for status reporting
 */
enum class SamplingMode : uint8_t {
    EMERGENCY = 0,    // Rapid rise detected - maximum frequency
    ALERT = 1,        // Critical level or significant change
    NORMAL = 2,       // Standard operation
    STABLE = 3,       // Minimal change - power saving mode
    BATTERY_SAVE = 4  // Low battery - extended intervals
};

/**
 * Configuration structure for adaptive sampling
 */
struct AdaptiveSamplingConfig {
    // Interval bounds (seconds)
    uint32_t minIntervalSec = 30;       // Minimum interval (emergency mode)
    uint32_t normalIntervalSec = 120;   // Normal interval (standard operation)
    uint32_t maxIntervalSec = 300;      // Maximum interval (stable/battery save)
    uint32_t emergencyIntervalSec = 60; // Emergency interval (rapid change)

    // Rate of change thresholds (cm per measurement interval)
    float rapidChangeCmPerInterval = 5.0f;    // >5cm change triggers emergency mode
    float moderateChangeCmPerInterval = 2.0f; // >2cm triggers alert mode
    float stableChangeCmPerInterval = 0.5f;   // <0.5cm considered stable

    // Critical level thresholds (cm)
    float criticalLevelCm = 50.0f;     // Critical river level
    float warningLevelCm = 30.0f;      // Warning river level

    // Battery thresholds (%)
    uint8_t batteryLowPercent = 20;    // Below this, use power saving mode
    uint8_t batteryCriticalPercent = 10; // Below this, maximum power saving

    // Consecutive readings configuration
    uint8_t consecutiveStableRequired = 3;  // Readings before switching to stable mode
    uint8_t consecutiveRapidRequired = 1;   // Readings for emergency mode (immediate)
};

/**
 * Result structure from interval calculation
 */
struct SamplingResult {
    uint32_t intervalSec;      // Calculated interval in seconds
    SamplingMode mode;         // Current sampling mode
    bool rapidChangeDetected;  // True if rapid change triggered emergency
    bool criticalLevel;        // True if at critical water level
    bool batteryLow;           // True if in battery saving mode
    float rateOfChangeCmPerMin; // Calculated rate of change (cm/min)
};

/**
 * Adaptive Sampling Interval Calculator
 *
 * Calculates optimal transmission intervals based on current conditions.
 * Tracks history to make intelligent decisions about sampling frequency.
 */
class AdaptiveSampler {
public:
    /**
     * Constructor with default configuration
     */
    AdaptiveSampler() : _config(), _lastDistanceCm(0.0f), _lastTimestampMs(0),
                         _consecutiveStable(0), _consecutiveRapid(0),
                         _initialized(false) {}

    /**
     * Constructor with custom configuration
     *
     * @param config Custom sampling configuration
     */
    explicit AdaptiveSampler(const AdaptiveSamplingConfig& config)
        : _config(config), _lastDistanceCm(0.0f), _lastTimestampMs(0),
          _consecutiveStable(0), _consecutiveRapid(0), _initialized(false) {}

    /**
     * Reset sampler state
     */
    void reset() {
        _lastDistanceCm = 0.0f;
        _lastTimestampMs = 0;
        _consecutiveStable = 0;
        _consecutiveRapid = 0;
        _initialized = false;
    }

    /**
     * Set custom configuration
     *
     * @param config New configuration to use
     */
    void setConfig(const AdaptiveSamplingConfig& config) {
        _config = config;
    }

    /**
     * Get current configuration
     *
     * @return Current configuration
     */
    const AdaptiveSamplingConfig& getConfig() const {
        return _config;
    }

    /**
     * Calculate the next transmission interval
     *
     * @param currentDistanceCm Current distance reading (cm)
     * @param riverLevelCm Current calculated river level (cm)
     * @param batteryPercent Current battery percentage (0-100)
     * @param timestampMs Current timestamp (millis()) - optional, uses internal timing if 0
     * @return SamplingResult with interval and status information
     */
    SamplingResult calculateInterval(float currentDistanceCm, float riverLevelCm,
                                      uint8_t batteryPercent, uint32_t timestampMs = 0) {
        SamplingResult result;
        result.rapidChangeDetected = false;
        result.criticalLevel = false;
        result.batteryLow = false;
        result.rateOfChangeCmPerMin = 0.0f;

        // Use millis() if no timestamp provided
        if (timestampMs == 0) {
            timestampMs = millis();
        }

        // Calculate rate of change if we have a previous reading
        float changeFromLast = 0.0f;
        float rateOfChange = 0.0f;

        if (_initialized && _lastTimestampMs > 0) {
            changeFromLast = abs(currentDistanceCm - _lastDistanceCm);
            uint32_t elapsedMs = timestampMs - _lastTimestampMs;

            if (elapsedMs > 0) {
                // Calculate rate in cm per minute
                rateOfChange = (changeFromLast / elapsedMs) * 60000.0f;
                result.rateOfChangeCmPerMin = rateOfChange;
            }
        }

        // Update tracking variables
        _lastDistanceCm = currentDistanceCm;
        _lastTimestampMs = timestampMs;
        _initialized = true;

        // Priority 1: Battery critical - maximum power saving
        if (batteryPercent <= _config.batteryCriticalPercent) {
            result.mode = SamplingMode::BATTERY_SAVE;
            result.intervalSec = _config.maxIntervalSec;
            result.batteryLow = true;
            _consecutiveStable = 0;
            _consecutiveRapid = 0;
            return result;
        }

        // Priority 2: Rapid change detection (potential flood)
        // Check against per-interval threshold, normalized by time
        if (changeFromLast >= _config.rapidChangeCmPerInterval) {
            _consecutiveRapid++;
            _consecutiveStable = 0;

            if (_consecutiveRapid >= _config.consecutiveRapidRequired) {
                result.mode = SamplingMode::EMERGENCY;
                result.intervalSec = _config.emergencyIntervalSec;
                result.rapidChangeDetected = true;

                // Check if also at critical level
                if (riverLevelCm >= _config.criticalLevelCm) {
                    result.criticalLevel = true;
                    // Even faster for emergency + critical
                    result.intervalSec = _config.minIntervalSec;
                }

                return result;
            }
        } else {
            _consecutiveRapid = 0;
        }

        // Priority 3: Critical level monitoring
        if (riverLevelCm >= _config.criticalLevelCm) {
            result.mode = SamplingMode::ALERT;
            result.intervalSec = _config.minIntervalSec;
            result.criticalLevel = true;
            _consecutiveStable = 0;
            return result;
        }

        // Priority 4: Warning level
        if (riverLevelCm >= _config.warningLevelCm) {
            result.mode = SamplingMode::ALERT;
            result.intervalSec = (_config.minIntervalSec + _config.normalIntervalSec) / 2;
            _consecutiveStable = 0;
            return result;
        }

        // Priority 5: Battery low (but not critical)
        if (batteryPercent <= _config.batteryLowPercent) {
            result.mode = SamplingMode::BATTERY_SAVE;
            result.intervalSec = _config.maxIntervalSec;
            result.batteryLow = true;
            return result;
        }

        // Priority 6: Moderate change
        if (changeFromLast >= _config.moderateChangeCmPerInterval) {
            result.mode = SamplingMode::NORMAL;
            // Scale interval between normal and min based on change magnitude
            float changeRatio = changeFromLast / _config.rapidChangeCmPerInterval;
            if (changeRatio > 1.0f) changeRatio = 1.0f;

            result.intervalSec = _config.normalIntervalSec -
                (uint32_t)((float)(_config.normalIntervalSec - _config.minIntervalSec) * changeRatio);
            _consecutiveStable = 0;
            return result;
        }

        // Priority 7: Stable conditions
        if (changeFromLast <= _config.stableChangeCmPerInterval) {
            _consecutiveStable++;

            if (_consecutiveStable >= _config.consecutiveStableRequired) {
                result.mode = SamplingMode::STABLE;
                result.intervalSec = _config.maxIntervalSec;
                return result;
            }
        } else {
            _consecutiveStable = 0;
        }

        // Default: Normal operation
        result.mode = SamplingMode::NORMAL;
        result.intervalSec = _config.normalIntervalSec;
        return result;
    }

    /**
     * Get a string description of the sampling mode
     *
     * @param mode Sampling mode
     * @return String description
     */
    static const char* getModeString(SamplingMode mode) {
        switch (mode) {
            case SamplingMode::EMERGENCY:    return "EMERGENCY";
            case SamplingMode::ALERT:        return "ALERT";
            case SamplingMode::NORMAL:       return "NORMAL";
            case SamplingMode::STABLE:       return "STABLE";
            case SamplingMode::BATTERY_SAVE: return "BATTERY_SAVE";
            default:                         return "UNKNOWN";
        }
    }

    /**
     * Check if currently in emergency or alert mode
     *
     * @return true if high-priority sampling is active
     */
    bool isHighPriority() const {
        return _consecutiveRapid > 0;
    }

    /**
     * Get the number of consecutive stable readings
     *
     * @return Count of consecutive stable readings
     */
    uint8_t getConsecutiveStable() const {
        return _consecutiveStable;
    }

    /**
     * Get the last recorded distance
     *
     * @return Last distance in cm
     */
    float getLastDistance() const {
        return _lastDistanceCm;
    }

private:
    AdaptiveSamplingConfig _config;
    float _lastDistanceCm;
    uint32_t _lastTimestampMs;
    uint8_t _consecutiveStable;
    uint8_t _consecutiveRapid;
    bool _initialized;
};

/**
 * Simple interval calculator function (for use without class instance)
 *
 * Implements the basic thesis requirement:
 * "If the river rises >5cm/min, transmit every 1 minute. If stable, transmit every 30 minutes"
 *
 * @param changeFromLastCm Change from last reading (cm)
 * @param riverLevelCm Current river level (cm)
 * @param batteryPercent Battery level (0-100)
 * @param minIntervalSec Minimum interval (default 60s = 1 min)
 * @param maxIntervalSec Maximum interval (default 1800s = 30 min)
 * @param rapidThresholdCm Rapid change threshold (default 5cm)
 * @param criticalLevelCm Critical level threshold (default 100cm)
 * @param batteryLowPercent Low battery threshold (default 20%)
 * @return Calculated interval in seconds
 */
inline uint32_t calculateSimpleInterval(
    float changeFromLastCm,
    float riverLevelCm,
    uint8_t batteryPercent,
    uint32_t minIntervalSec = 60,
    uint32_t maxIntervalSec = 1800,
    float rapidThresholdCm = 5.0f,
    float criticalLevelCm = 100.0f,
    uint8_t batteryLowPercent = 20
) {
    // Battery conservation takes priority
    if (batteryPercent < batteryLowPercent) {
        return maxIntervalSec;
    }

    // Rapid change = minimum interval
    if (changeFromLastCm >= rapidThresholdCm) {
        return minIntervalSec;
    }

    // Critical level = minimum interval
    if (riverLevelCm >= criticalLevelCm) {
        return minIntervalSec;
    }

    // Moderate change = proportional interval
    if (changeFromLastCm >= 1.0f) {
        float ratio = changeFromLastCm / rapidThresholdCm;
        if (ratio > 1.0f) ratio = 1.0f;
        return maxIntervalSec - (uint32_t)((float)(maxIntervalSec - minIntervalSec) * ratio);
    }

    // Stable = maximum interval
    return maxIntervalSec;
}

#endif // ADAPTIVE_SAMPLING_H

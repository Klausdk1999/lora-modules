/**
 * ThesisImprovements.h - Integration of Thesis Improvement Modules
 *
 * This header integrates all the thesis improvement modules:
 * - Kalman Filter for Single-Sensor and Multi-Sensor Fusion
 * - Anomaly Detection
 * - Temperature Compensation Validation
 * - Statistical Analysis
 *
 * Use this as a comprehensive include for the complete
 * thesis improvement functionality.
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef THESIS_IMPROVEMENTS_H
#define THESIS_IMPROVEMENTS_H

// Include all improvement modules
#include "../fusion/KalmanFilter.h"
#include "../analytics/AnomalyDetection.h"
#include "../analytics/StatisticalAnalysis.h"
#include "../analytics/TemperatureCompensation.h"
#include "../common/RiverMonitorConfig.h"

// ============================================================================
// Integrated Sensor Processing Pipeline
// ============================================================================

/**
 * ProcessedReading - Complete processed sensor reading with all improvements
 */
struct ProcessedReading {
    // Raw readings
    float lidarDistCm;              // Raw LiDAR distance
    float ultrasonicDistCm;         // Raw ultrasonic distance
    float temperature;              // Ambient temperature

    // Individually filtered readings (Kalman per sensor)
    float lidarFilteredCm;          // Kalman-filtered LiDAR
    float ultrasonicFilteredCm;     // Kalman-filtered ultrasonic

    // Compensated readings
    float lidarCompensatedCm;       // Temperature-compensated LiDAR
    float ultrasonicCompensatedCm;  // Temperature-compensated ultrasonic

    // Fused reading
    float fusedDistanceCm;          // Kalman-fused distance (both sensors)
    float fusedUncertainty;         // Fusion uncertainty (std dev)

    // River level
    float riverLevelCm;             // Calculated river level
    float rateOfChangeCmH;          // Rate of change (cm/hour)

    // Quality metrics
    bool isAnomaly;                 // Anomaly detected
    AnomalyType anomalyType;        // Type of anomaly
    float anomalyScore;             // Anomaly severity (0-1)

    // Weights
    float lidarWeight;              // Weight given to LiDAR in fusion (0-1)
    float ultrasonicWeight;         // Weight given to ultrasonic in fusion (0-1)

    // Flags
    uint8_t statusFlags;            // Status flags for transmission
    bool valid;                     // Whether reading is valid
};

// Status flag bits for ProcessedReading
#define STATUS_LIDAR_VALID          0x01
#define STATUS_ULTRASONIC_VALID     0x02
#define STATUS_TEMP_COMPENSATED     0x04
#define STATUS_FUSION_APPLIED       0x08
#define STATUS_ANOMALY_DETECTED     0x10
#define STATUS_RAPID_CHANGE         0x20
#define STATUS_CRITICAL_LEVEL       0x40
#define STATUS_BATTERY_LOW          0x80

// ============================================================================
// Integrated Sensor Processor Class
// ============================================================================

/**
 * IntegratedSensorProcessor - Complete sensor processing pipeline
 *
 * Combines all thesis improvements into a single processing class:
 * 1. Individual Kalman filtering per sensor
 * 2. Temperature compensation for both sensors
 * 3. Multi-sensor Kalman fusion
 * 4. Anomaly detection on fused result
 * 5. River level calculation
 * 6. Rate of change monitoring
 */
class IntegratedSensorProcessor {
public:
    /**
     * Constructor with calibration parameters
     */
    IntegratedSensorProcessor(const SensorCalibration& calibration) :
        _riverCalc(calibration),
        _ultrasonicComp(25.0f),
        _lidarComp(25.0f),
        _lidarFilter(KALMAN_LIDAR_NOISE),
        _ultrasonicFilter(KALMAN_ULTRASONIC_NOISE),
        _fusionFilter(),
        _anomalyDetector(),
        _lastReading(0.0f),
        _lastTimestamp(0) {

        // Configure anomaly detector for water level monitoring
        _anomalyDetector.setValidRange(
            calibration.minDistanceCm,
            calibration.maxDistanceCm
        );
        _anomalyDetector.setThresholds(
            3.0f,   // Z-score threshold
            1.5f,   // IQR multiplier
            10.0f,  // Rate threshold (cm/min)
            0.1f    // Stuck threshold
        );
    }

    /**
     * Process sensor readings through the complete pipeline
     */
    ProcessedReading process(float lidarDistCm, float ultrasonicDistCm,
                              float temperature, int16_t lidarSignal = -1,
                              uint32_t timestamp = 0) {
        ProcessedReading result = {};
        result.lidarDistCm = lidarDistCm;
        result.ultrasonicDistCm = ultrasonicDistCm;
        result.temperature = temperature;
        result.statusFlags = 0;

        if (timestamp == 0) timestamp = millis();

        // Check sensor validity
        bool hasLidar = (lidarDistCm > 0);
        bool hasUltrasonic = (ultrasonicDistCm > 0);

        if (hasLidar) result.statusFlags |= STATUS_LIDAR_VALID;
        if (hasUltrasonic) result.statusFlags |= STATUS_ULTRASONIC_VALID;

        if (!hasLidar && !hasUltrasonic) {
            result.valid = false;
            return result;
        }

        // Step 1: Individual Kalman filtering per sensor
        if (hasLidar) {
            result.lidarFilteredCm = _lidarFilter.filter(lidarDistCm, timestamp);
        }
        if (hasUltrasonic) {
            result.ultrasonicFilteredCm = _ultrasonicFilter.filter(ultrasonicDistCm, timestamp);
        }

        // Step 2: Temperature compensation
        if (hasUltrasonic) {
            CompensationResult usComp = _ultrasonicComp.compensate(
                result.ultrasonicFilteredCm, temperature);
            result.ultrasonicCompensatedCm = usComp.compensatedDistance_cm;
        }

        if (hasLidar) {
            CompensationResult lidarCompResult = _lidarComp.compensate(
                result.lidarFilteredCm, temperature);
            result.lidarCompensatedCm = lidarCompResult.compensatedDistance_cm;
        }

        result.statusFlags |= STATUS_TEMP_COMPENSATED;

        // Step 3: Multi-sensor Kalman fusion
        FusionResult fusion = _fusionFilter.fuse(
            hasLidar ? result.lidarCompensatedCm : -1.0f,
            hasUltrasonic ? result.ultrasonicCompensatedCm : -1.0f,
            lidarSignal,
            true  // Temperature compensated
        );

        if (fusion.valid) {
            result.fusedDistanceCm = fusion.fusedDistance;
            result.fusedUncertainty = fusion.uncertainty;
            result.lidarWeight = fusion.lidarWeight;
            result.ultrasonicWeight = fusion.ultrasonicWeight;
            result.statusFlags |= STATUS_FUSION_APPLIED;
        } else {
            // Fallback to best available
            result.fusedDistanceCm = hasLidar ?
                result.lidarCompensatedCm : result.ultrasonicCompensatedCm;
            result.fusedUncertainty = 5.0f;
            result.lidarWeight = hasLidar ? 1.0f : 0.0f;
            result.ultrasonicWeight = hasUltrasonic && !hasLidar ? 1.0f : 0.0f;
        }

        // Step 4: Calculate river level
        float correctedDist;
        result.riverLevelCm = _riverCalc.calculateRiverLevel(
            result.fusedDistanceCm, temperature, &correctedDist);

        // Step 5: Calculate rate of change
        if (_lastTimestamp > 0) {
            float dtHours = (timestamp - _lastTimestamp) / 3600000.0f;
            if (dtHours > 0.001f) {
                result.rateOfChangeCmH = (result.riverLevelCm - _lastReading) / dtHours;
            }
        }

        // Update history
        _lastReading = result.riverLevelCm;
        _lastTimestamp = timestamp;

        // Step 6: Anomaly detection
        AnomalyResult anomaly = _anomalyDetector.addReading(
            result.fusedDistanceCm, timestamp);

        result.isAnomaly = anomaly.isAnomaly;
        result.anomalyType = anomaly.type;
        result.anomalyScore = anomaly.anomalyScore;

        if (anomaly.isAnomaly) {
            result.statusFlags |= STATUS_ANOMALY_DETECTED;
        }

        // Step 7: Check for rapid change
        if (fabs(result.rateOfChangeCmH) > 10.0f) {  // 10 cm/hour
            result.statusFlags |= STATUS_RAPID_CHANGE;
            _fusionFilter.setRapidChangeMode(true);
            _lidarFilter.setRapidChangeMode(true);
            _ultrasonicFilter.setRapidChangeMode(true);
        } else {
            _fusionFilter.setRapidChangeMode(false);
            _lidarFilter.setRapidChangeMode(false);
            _ultrasonicFilter.setRapidChangeMode(false);
        }

        // Step 8: Check for critical level
        if (result.riverLevelCm > 100.0f) {  // 1 meter
            result.statusFlags |= STATUS_CRITICAL_LEVEL;
        }

        result.valid = true;
        return result;
    }

    /**
     * Get LiDAR filter for direct access
     */
    KalmanFilterSingle& getLidarFilter() { return _lidarFilter; }

    /**
     * Get Ultrasonic filter for direct access
     */
    KalmanFilterSingle& getUltrasonicFilter() { return _ultrasonicFilter; }

    /**
     * Get Fusion filter for direct access
     */
    KalmanFilterFusion& getFusionFilter() { return _fusionFilter; }

    /**
     * Get anomaly detector statistics
     */
    void getAnomalyStats(float& mean, float& stdDev, float& min, float& max) const {
        _anomalyDetector.getStatistics(mean, stdDev, min, max);
    }

    /**
     * Reset processor state
     */
    void reset() {
        _lidarFilter.reset();
        _ultrasonicFilter.reset();
        _fusionFilter.reset();
        _anomalyDetector.reset();
        _lastReading = 0.0f;
        _lastTimestamp = 0;
    }

    /**
     * Get river level calculator for direct access
     */
    RiverLevelCalculator& getRiverCalculator() {
        return _riverCalc;
    }

private:
    RiverLevelCalculator _riverCalc;
    UltrasonicCompensation _ultrasonicComp;
    LiDARCompensation _lidarComp;
    KalmanFilterSingle _lidarFilter;
    KalmanFilterSingle _ultrasonicFilter;
    KalmanFilterFusion _fusionFilter;
    AnomalyDetector _anomalyDetector;
    float _lastReading;
    uint32_t _lastTimestamp;
};

// ============================================================================
// Extended Payload with Improvements
// ============================================================================

/**
 * Extended payload structure including thesis improvement data
 * Total: 20 bytes (fits LoRaWAN SF7)
 */
struct __attribute__((packed)) ImprovedSensorPayload {
    uint8_t version;                // Payload version (0x02 for improved)
    uint8_t statusFlags;            // Status flags
    uint16_t fusedDistMm;           // Fused distance (mm)
    uint16_t riverLevelMm;          // River level (mm)
    int16_t rateOfChange;           // Rate of change (mm/hour, signed)
    uint8_t fusionUncertainty;      // Fusion uncertainty (cm, 0-255)
    uint8_t anomalyScore;           // Anomaly score (0-100)
    int8_t temperature;             // Temperature (°C)
    uint8_t batteryPercent;         // Battery (%)
    int16_t lidarDistMm;            // Raw LiDAR (mm)
    int16_t ultrasonicDistMm;       // Raw ultrasonic (mm)
    uint8_t lidarWeight;            // LiDAR fusion weight (0-100%)
    uint8_t reserved;               // Reserved for future use
};

#define PAYLOAD_VERSION_IMPROVED    0x02

/**
 * Encode ProcessedReading to payload
 */
inline void encodeImprovedPayload(const ProcessedReading& reading,
                                   uint8_t batteryPercent,
                                   ImprovedSensorPayload& payload) {
    payload.version = PAYLOAD_VERSION_IMPROVED;
    payload.statusFlags = reading.statusFlags;
    payload.fusedDistMm = (uint16_t)(reading.fusedDistanceCm * 10);
    payload.riverLevelMm = (uint16_t)(reading.riverLevelCm * 10);
    payload.rateOfChange = (int16_t)(reading.rateOfChangeCmH * 10);
    payload.fusionUncertainty = (uint8_t)min(255.0f, reading.fusedUncertainty);
    payload.anomalyScore = (uint8_t)(reading.anomalyScore * 100);
    payload.temperature = (int8_t)reading.temperature;
    payload.batteryPercent = batteryPercent;
    payload.lidarDistMm = (reading.statusFlags & STATUS_LIDAR_VALID) ?
        (int16_t)(reading.lidarDistCm * 10) : -1;
    payload.ultrasonicDistMm = (reading.statusFlags & STATUS_ULTRASONIC_VALID) ?
        (int16_t)(reading.ultrasonicDistCm * 10) : -1;
    payload.lidarWeight = (uint8_t)(reading.lidarWeight * 100);
    payload.reserved = 0;
}

// ============================================================================
// Statistical Validation Helper
// ============================================================================

/**
 * Validate sensor readings with statistical analysis
 */
class SensorValidation {
public:
    void addDataPoint(float measured, float reference, float temperature) {
        if (_count < 100) {
            _measured[_count] = measured;
            _reference[_count] = reference;
            _temperature[_count] = temperature;
            _count++;
        }
    }

    DescriptiveStats getErrorStatistics() {
        if (_count < 5) {
            DescriptiveStats invalid = {};
            return invalid;
        }

        float errors[100];
        for (uint16_t i = 0; i < _count; i++) {
            errors[i] = _measured[i] - _reference[i];
        }

        return StatisticalAnalysis::descriptive(errors, _count);
    }

    ConfidenceInterval getErrorCI() {
        DescriptiveStats stats = getErrorStatistics();
        if (!stats.valid) {
            ConfidenceInterval invalid = {};
            return invalid;
        }
        return StatisticalAnalysis::confidenceInterval(
            stats.mean, stats.stdError, stats.n, 0.95f);
    }

    CorrelationResult getTemperatureCorrelation() {
        if (_count < 5) {
            CorrelationResult invalid = {};
            return invalid;
        }

        float errors[100];
        for (uint16_t i = 0; i < _count; i++) {
            errors[i] = _measured[i] - _reference[i];
        }

        return StatisticalAnalysis::pearsonCorrelation(
            _temperature, errors, _count);
    }

    void clear() { _count = 0; }
    uint16_t getCount() const { return _count; }

private:
    float _measured[100];
    float _reference[100];
    float _temperature[100];
    uint16_t _count = 0;
};

// ============================================================================
// Usage Examples
// ============================================================================
/*

=== EXAMPLE 1: Single Sensor Kalman Filtering (LiDAR only) ===

#include "fusion/KalmanFilter.h"

// Create filter for LiDAR
KalmanFilterSingle lidarFilter = createLidarFilter();

// In sensor reading loop:
float rawLidar = tfNova.readDistanceCm();
float filteredLidar = lidarFilter.filter(rawLidar);

Serial.print("Raw: "); Serial.print(rawLidar);
Serial.print(" cm, Filtered: "); Serial.print(filteredLidar);
Serial.print(" cm (±"); Serial.print(lidarFilter.getUncertainty());
Serial.println(" cm)");


=== EXAMPLE 2: Single Sensor Kalman Filtering (Ultrasonic only) ===

#include "fusion/KalmanFilter.h"

// Create filter for ultrasonic (with temperature compensation)
KalmanFilterSingle usFilter = createUltrasonicFilter(true);  // temp compensated

// In sensor reading loop:
float rawUS = ultrasonic.readDistanceCm();
float filteredUS = usFilter.filter(rawUS);


=== EXAMPLE 3: Multi-Sensor Fusion (both sensors) ===

#include "fusion/KalmanFilter.h"

// Create fusion filter
KalmanFilterFusion fusion = createFusionFilter();

// In sensor reading loop:
float lidarDist = tfNova.readDistanceCm();
float usDist = ultrasonic.readDistanceCm();
int16_t lidarSignal = tfNova.getSignalStrength();

// Fuse readings (pass -1 for unavailable sensors)
FusionResult result = fusion.fuse(lidarDist, usDist, lidarSignal, true);

if (result.valid) {
    Serial.print("Fused: "); Serial.print(result.fusedDistance);
    Serial.print(" cm (LiDAR weight: "); Serial.print(result.lidarWeight * 100);
    Serial.println("%)");
}


=== EXAMPLE 4: Using Fusion with only one sensor available ===

// Works seamlessly when one sensor is unavailable
FusionResult result = fusion.fuse(lidarDist, -1);  // No ultrasonic
// or
FusionResult result = fusion.fuse(-1, usDist);     // No LiDAR


=== EXAMPLE 5: Complete Integrated Pipeline ===

#include "integration/ThesisImprovements.h"

// Setup calibration
SensorCalibration calibration;
calibration.baselineDistanceCm = 400.0f;
calibration.sensorHeightCm = 400.0f;
calibration.minDistanceCm = 10.0f;
calibration.maxDistanceCm = 2200.0f;

// Create integrated processor
IntegratedSensorProcessor processor(calibration);

// In sensor reading loop:
float lidarDist = tfNova.readDistanceCm();
float usDist = ultrasonic.readDistanceCm();
float temperature = dht.readTemperature();
int16_t lidarSignal = tfNova.getSignalStrength();

// Process through full pipeline
ProcessedReading result = processor.process(
    lidarDist, usDist, temperature, lidarSignal);

if (result.valid) {
    // Individual filtered values
    Serial.print("LiDAR filtered: "); Serial.println(result.lidarFilteredCm);
    Serial.print("Ultrasonic filtered: "); Serial.println(result.ultrasonicFilteredCm);

    // Fused result
    Serial.print("Fused: "); Serial.print(result.fusedDistanceCm);
    Serial.print(" cm (±"); Serial.print(result.fusedUncertainty);
    Serial.println(" cm)");

    // Weights
    Serial.print("LiDAR weight: "); Serial.print(result.lidarWeight * 100);
    Serial.println("%");

    // River level
    Serial.print("River level: "); Serial.println(result.riverLevelCm);

    // Anomaly check
    if (result.isAnomaly) {
        Serial.print("ANOMALY: ");
        Serial.println(anomalyTypeToString(result.anomalyType));
    }
}


=== EXAMPLE 6: Accessing Individual Filters from Processor ===

// Get individual filter statistics
Serial.print("LiDAR filter updates: ");
Serial.println(processor.getLidarFilter().getUpdateCount());

Serial.print("Ultrasonic uncertainty: ");
Serial.println(processor.getUltrasonicFilter().getUncertainty());

*/

#endif // THESIS_IMPROVEMENTS_H

/**
 * RiverMonitorConfig.h - Shared Configuration and Calibration for River Monitoring
 *
 * This header provides common configuration parameters and calibration constants
 * for river level monitoring using LiDAR and ultrasonic sensors.
 *
 * Features:
 * - Sensor installation angle correction (trigonometric)
 * - Temperature compensation for speed-of-sound (ultrasonic)
 * - Temperature compensation for LiDAR drift
 * - River level/depth calculation from distance measurements
 * - Configurable calibration parameters
 *
 * Mathematical Foundation:
 * - Trigonometric correction: D_vertical = D_measured * cos(theta)
 * - Speed of sound: v(T) = 331.3 + 0.606 * T (m/s)
 * - Temperature compensation: D_corrected = D_measured - k * (T - T_cal)
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef RIVER_MONITOR_CONFIG_H
#define RIVER_MONITOR_CONFIG_H

#include <Arduino.h>
#include <math.h>

// Define PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Sensor Configuration Structure
// ============================================================================
// This structure holds all calibration parameters for a sensor node
struct SensorCalibration {
    // Installation parameters
    float installationAngleDeg;     // Sensor tilt angle in degrees (0 = pointing straight down)
    float baselineDistanceCm;       // Reference distance (e.g., distance to riverbed when empty)
    float sensorHeightCm;           // Height of sensor above reference point (e.g., riverbed)

    // Temperature compensation parameters
    float calibrationTempC;         // Temperature at which sensor was calibrated (default 25C)
    float tempCompensationK;        // Temperature compensation factor k (cm per degree C)

    // Sensor limits
    float minDistanceCm;            // Minimum valid distance (sensor blind zone)
    float maxDistanceCm;            // Maximum valid distance (sensor range limit)

    // Default constructor with reasonable defaults
    SensorCalibration() :
        installationAngleDeg(0.0f),
        baselineDistanceCm(400.0f),
        sensorHeightCm(400.0f),
        calibrationTempC(25.0f),
        tempCompensationK(0.0f),
        minDistanceCm(10.0f),
        maxDistanceCm(2000.0f) {}
};

// ============================================================================
// Extended Sensor Payload Structure
// ============================================================================
// This structure includes both raw and calculated values for transmission
// Total size: 16 bytes (fits within LoRaWAN SF7 limits)
struct __attribute__((packed)) ExtendedSensorPayload {
    uint8_t sensorType;             // 1 = TF02-Pro, 2 = HC-SR04, 3 = JSN-SR04T, 0xFF = error
    uint16_t rawDistanceMm;         // Raw distance measurement in millimeters
    uint16_t correctedDistanceMm;   // Corrected distance (after temp/angle compensation)
    uint16_t riverLevelMm;          // Calculated river level/depth in millimeters
    int16_t signalStrength;         // Signal strength (LiDAR) or 0 (ultrasonic)
    int8_t temperature;             // Ambient/sensor temperature in Celsius
    uint8_t batteryPercent;         // Battery level (0-100)
    uint8_t readingCount;           // Number of valid readings used in average
    uint8_t flags;                  // Status flags (see below)
    uint8_t errorCode;              // Error code if sensorType == 0xFF
};

// Flag bit definitions
#define FLAG_RAPID_CHANGE       0x01  // Bit 0: Rapid water level change detected
#define FLAG_CRITICAL_LEVEL     0x02  // Bit 1: Critical water level exceeded
#define FLAG_BATTERY_LOW        0x04  // Bit 2: Battery low warning
#define FLAG_SENSOR_ERROR       0x08  // Bit 3: Sensor error occurred
#define FLAG_TEMP_COMPENSATED   0x10  // Bit 4: Temperature compensation applied
#define FLAG_ANGLE_CORRECTED    0x20  // Bit 5: Angle correction applied

// Error code definitions
#define ERR_CODE_NONE               0
#define ERR_CODE_NOT_INITIALIZED    1
#define ERR_CODE_NO_VALID_READINGS  2
#define ERR_CODE_ALL_READINGS_ZERO  3
#define ERR_CODE_CHECKSUM_FAILURE   4
#define ERR_CODE_TIMEOUT            5
#define ERR_CODE_INSUFFICIENT_VALID 6
#define ERR_CODE_TEMP_SENSOR_FAIL   7

// Sensor type definitions
#define SENSOR_TYPE_TF02_PRO    1
#define SENSOR_TYPE_HC_SR04     2
#define SENSOR_TYPE_JSN_SR04T   3
#define SENSOR_TYPE_TF_LUNA     4
#define SENSOR_TYPE_ERROR       0xFF

// ============================================================================
// River Level Calculation Class
// ============================================================================
class RiverLevelCalculator {
public:
    /**
     * Constructor with default calibration
     */
    RiverLevelCalculator() : _calibration() {}

    /**
     * Constructor with custom calibration
     */
    RiverLevelCalculator(const SensorCalibration& calibration) : _calibration(calibration) {}

    /**
     * Set calibration parameters
     */
    void setCalibration(const SensorCalibration& calibration) {
        _calibration = calibration;
    }

    /**
     * Get current calibration
     */
    const SensorCalibration& getCalibration() const {
        return _calibration;
    }

    /**
     * Set installation angle in degrees
     * @param angleDeg Angle from vertical (0 = straight down, 15 = tilted 15 degrees)
     */
    void setInstallationAngle(float angleDeg) {
        _calibration.installationAngleDeg = angleDeg;
    }

    /**
     * Set baseline distance (empty river reference)
     * @param distanceCm Distance to riverbed/bottom when river is empty
     */
    void setBaselineDistance(float distanceCm) {
        _calibration.baselineDistanceCm = distanceCm;
    }

    /**
     * Set sensor height above reference
     * @param heightCm Height of sensor above riverbed
     */
    void setSensorHeight(float heightCm) {
        _calibration.sensorHeightCm = heightCm;
    }

    /**
     * Set temperature compensation parameters
     * @param calibTempC Calibration temperature (default 25C)
     * @param k Compensation factor in cm per degree C (typical: 0.017 for ultrasonic)
     */
    void setTemperatureCompensation(float calibTempC, float k) {
        _calibration.calibrationTempC = calibTempC;
        _calibration.tempCompensationK = k;
    }

    /**
     * Apply trigonometric correction for sensor tilt angle
     *
     * When the sensor is tilted, the beam travels a longer path (hypotenuse)
     * than the vertical distance to the water. We correct using cosine.
     *
     * Formula: D_vertical = D_measured * cos(theta)
     *
     * @param measuredDistanceCm Raw distance from sensor
     * @return Vertical distance component
     */
    float applyAngleCorrection(float measuredDistanceCm) const {
        if (_calibration.installationAngleDeg == 0.0f) {
            return measuredDistanceCm;  // No correction needed
        }

        float angleRad = _calibration.installationAngleDeg * (M_PI / 180.0f);
        return measuredDistanceCm * cos(angleRad);
    }

    /**
     * Apply temperature compensation to distance measurement
     *
     * Formula: D_corrected = D_measured - k * (T - T_cal)
     *
     * For ultrasonic sensors, k is derived from the speed of sound variation.
     * For LiDAR sensors, k is typically smaller (optical path less affected).
     *
     * This can reduce measurement error by up to 70% according to literature.
     *
     * @param measuredDistanceCm Raw distance from sensor
     * @param currentTempC Current ambient temperature
     * @return Temperature-compensated distance
     */
    float applyTemperatureCompensation(float measuredDistanceCm, float currentTempC) const {
        if (_calibration.tempCompensationK == 0.0f) {
            return measuredDistanceCm;  // No compensation configured
        }

        float tempDelta = currentTempC - _calibration.calibrationTempC;
        float correction = _calibration.tempCompensationK * tempDelta;

        return measuredDistanceCm - correction;
    }

    /**
     * Apply all corrections and calculate river level
     *
     * The complete calculation pipeline:
     * 1. Apply temperature compensation
     * 2. Apply angle correction
     * 3. Calculate river level from baseline
     *
     * River Level = Baseline Distance - Corrected Current Distance
     * (Higher water = smaller distance = larger river level)
     *
     * @param rawDistanceCm Raw sensor reading
     * @param currentTempC Current temperature for compensation
     * @param correctedDistanceOut [out] Corrected distance (optional)
     * @return Calculated river level in cm
     */
    float calculateRiverLevel(float rawDistanceCm, float currentTempC,
                              float* correctedDistanceOut = nullptr) const {
        // Step 1: Apply temperature compensation
        float tempCompensated = applyTemperatureCompensation(rawDistanceCm, currentTempC);

        // Step 2: Apply angle correction
        float angleCorrected = applyAngleCorrection(tempCompensated);

        // Output corrected distance if requested
        if (correctedDistanceOut != nullptr) {
            *correctedDistanceOut = angleCorrected;
        }

        // Step 3: Calculate river level
        // Level = Distance to Bottom (Baseline) - Distance to Surface (Current)
        float verticalBaseline = applyAngleCorrection(_calibration.baselineDistanceCm);
        float level = verticalBaseline - angleCorrected;

        // Clamp to zero if negative (noise or measurement error)
        if (level < 0.0f) {
            level = 0.0f;
        }

        return level;
    }

    /**
     * Alternative calculation using sensor height
     *
     * River Level = Sensor Height - Distance to Water Surface
     * This assumes sensor height is the reference (e.g., height above riverbed)
     *
     * @param rawDistanceCm Raw sensor reading
     * @param currentTempC Current temperature for compensation
     * @return Calculated river level in cm
     */
    float calculateRiverLevelFromHeight(float rawDistanceCm, float currentTempC) const {
        // Apply corrections
        float tempCompensated = applyTemperatureCompensation(rawDistanceCm, currentTempC);
        float angleCorrected = applyAngleCorrection(tempCompensated);

        // Calculate level from sensor height
        float level = _calibration.sensorHeightCm - angleCorrected;

        // Clamp to zero
        if (level < 0.0f) {
            level = 0.0f;
        }

        return level;
    }

    /**
     * Calculate speed of sound in air at given temperature
     *
     * Based on Mohammed et al. (2019) and Tawalbeh et al. (2023):
     * v(T) = 331.3 + 0.606 * T (m/s)
     *
     * @param temperatureC Temperature in Celsius
     * @return Speed of sound in m/s
     */
    static float calculateSpeedOfSound(float temperatureC) {
        return 331.3f + (0.606f * temperatureC);
    }

    /**
     * Calculate temperature compensation factor k for ultrasonic sensors
     *
     * Based on the speed of sound relationship, we can derive k as the
     * expected distance error per degree Celsius deviation.
     *
     * For a measurement at distance D and reference temp T_ref:
     * k = D * 0.606 / v(T_ref)
     *
     * At 25C: v = 346.45 m/s, so k ≈ D * 0.00175 per degree C
     * For D = 100cm: k ≈ 0.175 cm/C
     *
     * @param referenceDistanceCm Reference distance for calculation
     * @param referenceTempC Reference temperature (default 25C)
     * @return Compensation factor k in cm per degree C
     */
    static float calculateUltrasonicK(float referenceDistanceCm, float referenceTempC = 25.0f) {
        float vRef = calculateSpeedOfSound(referenceTempC);
        // k = D * dv/dT / v = D * 0.606 / v
        return referenceDistanceCm * 0.606f / vRef;
    }

    /**
     * Validate distance reading is within sensor limits
     */
    bool isValidReading(float distanceCm) const {
        return (distanceCm >= _calibration.minDistanceCm &&
                distanceCm <= _calibration.maxDistanceCm);
    }

private:
    SensorCalibration _calibration;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Convert degrees to radians
 */
inline float degToRad(float degrees) {
    return degrees * (M_PI / 180.0f);
}

/**
 * Convert radians to degrees
 */
inline float radToDeg(float radians) {
    return radians * (180.0f / M_PI);
}

/**
 * Simple median calculation for small arrays
 * Uses bubble sort (efficient for small N)
 */
inline float calculateMedian(float readings[], uint8_t count) {
    if (count == 0) return 0.0f;
    if (count == 1) return readings[0];

    // Copy to temporary array
    float sorted[16];  // Max 16 readings supported
    uint8_t n = min((uint8_t)16, count);
    for (uint8_t i = 0; i < n; i++) {
        sorted[i] = readings[i];
    }

    // Bubble sort
    for (uint8_t i = 0; i < n - 1; i++) {
        for (uint8_t j = 0; j < n - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    // Return median
    if (n % 2 == 0) {
        return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0f;
    } else {
        return sorted[n / 2];
    }
}

/**
 * Filter outliers based on deviation from median
 * Returns the average of non-outlier values
 */
inline float filterOutliers(float readings[], uint8_t& validCount,
                            float median, float thresholdPct) {
    float sum = 0.0f;
    uint8_t filteredCount = 0;
    float threshold = median * (thresholdPct / 100.0f);

    for (uint8_t i = 0; i < validCount; i++) {
        float deviation = fabs(readings[i] - median);
        if (deviation <= threshold) {
            sum += readings[i];
            filteredCount++;
        }
    }

    validCount = filteredCount;

    if (filteredCount == 0) {
        return median;  // Fallback to median
    }

    return sum / filteredCount;
}

#endif // RIVER_MONITOR_CONFIG_H

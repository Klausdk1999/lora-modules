/**
 * TemperatureCompensation.h - Temperature Compensation Validation Module
 *
 * Implements and validates temperature compensation algorithms for both
 * ultrasonic and LiDAR sensors used in water level monitoring.
 *
 * Ultrasonic Compensation:
 * - Speed of sound varies with temperature: v(T) = 331.5 + 0.607 * T (m/s)
 * - Distance error: ~0.17% per °C deviation from reference
 * - Can reduce error by up to 70% (Mohammed et al. 2019)
 *
 * LiDAR Compensation:
 * - Internal temperature affects optical alignment and electronics
 * - Typical drift: ~0.1% per °C (Paul 2020)
 * - Less affected than ultrasonic but still relevant for precision
 *
 * References:
 * - Mohammed et al. (2019) - Ultrasonic temperature compensation
 * - Tawalbeh et al. (2023) - 20°C diurnal swings causing several cm error
 * - Panagopoulos et al. (2021) - Temperature compensation in field
 * - Paul (2020) - LiDAR temperature dependency
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef TEMPERATURE_COMPENSATION_H
#define TEMPERATURE_COMPENSATION_H

#include <Arduino.h>
#include <math.h>

// ============================================================================
// Physical Constants
// ============================================================================

// Speed of sound coefficients (v = a + b*T)
#define SPEED_OF_SOUND_A        331.5f      // Base speed at 0°C (m/s)
#define SPEED_OF_SOUND_B        0.607f      // Temperature coefficient (m/s/°C)

// Alternative formulations from literature
#define SPEED_OF_SOUND_A_ALT    331.3f      // Mohammed et al. variant
#define SPEED_OF_SOUND_B_ALT    0.606f      // Mohammed et al. variant

// Reference conditions
#define REFERENCE_TEMP_C        25.0f       // Standard reference temperature
#define REFERENCE_SPEED_MS      346.68f     // Speed at 25°C

// LiDAR temperature coefficients
#define LIDAR_TEMP_COEFF        0.001f      // 0.1% per °C
#define LIDAR_REFERENCE_TEMP    25.0f       // Reference temperature

// ============================================================================
// Compensation Result Structure
// ============================================================================
struct CompensationResult {
    float rawDistance_cm;           // Original uncorrected distance
    float compensatedDistance_cm;   // Temperature-compensated distance
    float correction_cm;            // Applied correction
    float correctionPercent;        // Correction as percentage
    float temperature_C;            // Temperature used for compensation
    float speedOfSound_ms;          // Calculated speed of sound
    bool applied;                   // Whether compensation was applied
};

// ============================================================================
// Validation Metrics Structure
// ============================================================================
struct ValidationMetrics {
    uint16_t sampleCount;           // Number of samples
    float meanRawError_cm;          // Mean error without compensation
    float meanCompError_cm;         // Mean error with compensation
    float stdRawError_cm;           // Std dev of raw error
    float stdCompError_cm;          // Std dev of compensated error
    float rmseRaw_cm;               // RMSE without compensation
    float rmseComp_cm;              // RMSE with compensation
    float maxRawError_cm;           // Maximum raw error
    float maxCompError_cm;          // Maximum compensated error
    float errorReduction_pct;       // Percentage reduction in error
    float tempRange_C;              // Temperature range in dataset
    bool significantImprovement;    // Statistical significance flag
};

// ============================================================================
// Temperature Data Point Structure
// ============================================================================
struct TempDataPoint {
    float measuredDist_cm;          // Measured distance
    float referenceDist_cm;         // True/reference distance
    float temperature_C;            // Temperature at measurement
    float humidity_pct;             // Humidity (if available)
    uint32_t timestamp;             // Timestamp
};

// ============================================================================
// Ultrasonic Temperature Compensation Class
// ============================================================================
class UltrasonicCompensation {
public:
    /**
     * Constructor with reference parameters
     */
    UltrasonicCompensation(float refTemp_C = REFERENCE_TEMP_C,
                            float refSpeed_ms = REFERENCE_SPEED_MS) :
        _refTemp(refTemp_C),
        _refSpeed(refSpeed_ms) {}

    /**
     * Calculate speed of sound at given temperature
     * Based on: v(T) = 331.5 + 0.607 * T (m/s)
     *
     * @param temperature_C Temperature in Celsius
     * @return Speed of sound in m/s
     */
    static float calculateSpeedOfSound(float temperature_C) {
        return SPEED_OF_SOUND_A + (SPEED_OF_SOUND_B * temperature_C);
    }

    /**
     * Alternative speed of sound formula (Mohammed et al.)
     * v(T) = 331.3 + 0.606 * T
     */
    static float calculateSpeedOfSoundMohammed(float temperature_C) {
        return SPEED_OF_SOUND_A_ALT + (SPEED_OF_SOUND_B_ALT * temperature_C);
    }

    /**
     * Compensate distance measurement for temperature
     *
     * The ultrasonic sensor calculates distance assuming a fixed speed of sound
     * (typically at 25°C). The actual measurement is:
     *   measured = actual * (v_ref / v_actual)
     *
     * So the corrected distance is:
     *   actual = measured * (v_actual / v_ref)
     *
     * @param measuredDist_cm Measured distance in cm
     * @param temperature_C Current temperature in Celsius
     * @return CompensationResult with corrected distance
     */
    CompensationResult compensate(float measuredDist_cm, float temperature_C) {
        CompensationResult result = {};
        result.rawDistance_cm = measuredDist_cm;
        result.temperature_C = temperature_C;

        // Calculate speed of sound at current temperature
        float currentSpeed = calculateSpeedOfSound(temperature_C);
        result.speedOfSound_ms = currentSpeed;

        // Calculate correction factor
        float correctionFactor = currentSpeed / _refSpeed;

        // Apply correction
        result.compensatedDistance_cm = measuredDist_cm * correctionFactor;
        result.correction_cm = result.compensatedDistance_cm - measuredDist_cm;
        result.correctionPercent = (result.correction_cm / measuredDist_cm) * 100.0f;
        result.applied = true;

        return result;
    }

    /**
     * Calculate expected error for temperature deviation
     *
     * Error per degree = (v_ref - v(T+1)) / v_ref * distance
     *                  ≈ 0.00175 * distance per °C (at 25°C reference)
     *
     * @param distance_cm Distance at which to calculate error
     * @param tempDeviation_C Temperature deviation from reference
     * @return Expected error in cm
     */
    static float calculateExpectedError(float distance_cm, float tempDeviation_C) {
        // Error coefficient at reference temperature
        float errorCoeff = SPEED_OF_SOUND_B / REFERENCE_SPEED_MS;  // ~0.00175
        return distance_cm * errorCoeff * tempDeviation_C;
    }

    /**
     * Calculate temperature compensation factor (k)
     *
     * k represents cm error per °C at a given reference distance
     * k = D * 0.607 / v(T_ref)
     *
     * @param referenceDistance_cm Reference distance
     * @return k factor in cm/°C
     */
    static float calculateKFactor(float referenceDistance_cm) {
        return referenceDistance_cm * SPEED_OF_SOUND_B / REFERENCE_SPEED_MS;
    }

    /**
     * Simple linear compensation using k factor
     * D_corrected = D_measured - k * (T - T_ref)
     */
    float compensateLinear(float measuredDist_cm, float temperature_C, float k) {
        float tempDelta = temperature_C - _refTemp;
        return measuredDist_cm - (k * tempDelta);
    }

    /**
     * Set reference parameters
     */
    void setReference(float refTemp_C, float refSpeed_ms) {
        _refTemp = refTemp_C;
        _refSpeed = refSpeed_ms;
    }

    /**
     * Get current reference temperature
     */
    float getReferenceTemp() const { return _refTemp; }

    /**
     * Get current reference speed
     */
    float getReferenceSpeed() const { return _refSpeed; }

private:
    float _refTemp;
    float _refSpeed;
};

// ============================================================================
// LiDAR Temperature Compensation Class
// ============================================================================
class LiDARCompensation {
public:
    /**
     * Constructor with reference parameters
     */
    LiDARCompensation(float refTemp_C = LIDAR_REFERENCE_TEMP,
                       float tempCoeff = LIDAR_TEMP_COEFF) :
        _refTemp(refTemp_C),
        _tempCoeff(tempCoeff) {}

    /**
     * Compensate LiDAR distance for temperature
     *
     * LiDAR temperature drift is typically small (~0.1% per °C)
     * Caused by:
     * - Optical alignment changes with thermal expansion
     * - Electronic timing drift
     * - Laser wavelength shift
     *
     * @param measuredDist_cm Measured distance in cm
     * @param sensorTemp_C Internal sensor temperature
     * @return CompensationResult with corrected distance
     */
    CompensationResult compensate(float measuredDist_cm, float sensorTemp_C) {
        CompensationResult result = {};
        result.rawDistance_cm = measuredDist_cm;
        result.temperature_C = sensorTemp_C;

        // Calculate temperature deviation
        float tempDelta = sensorTemp_C - _refTemp;

        // Apply linear correction
        // Positive temp = sensor reads slightly shorter (typically)
        float correctionFactor = 1.0f + (_tempCoeff * tempDelta);
        result.compensatedDistance_cm = measuredDist_cm * correctionFactor;

        result.correction_cm = result.compensatedDistance_cm - measuredDist_cm;
        result.correctionPercent = _tempCoeff * tempDelta * 100.0f;
        result.applied = true;

        return result;
    }

    /**
     * Calculate expected LiDAR error for temperature deviation
     */
    static float calculateExpectedError(float distance_cm, float tempDeviation_C,
                                         float tempCoeff = LIDAR_TEMP_COEFF) {
        return distance_cm * tempCoeff * tempDeviation_C;
    }

    /**
     * Calibrate temperature coefficient from measurements
     *
     * Given pairs of (measured, reference, temperature), calculate optimal coefficient
     */
    float calibrateCoefficient(const TempDataPoint* data, uint16_t count,
                                float refTemp_C = LIDAR_REFERENCE_TEMP) {
        if (count < 5) return _tempCoeff;  // Need minimum data

        // Use least squares to find optimal coefficient
        // Model: error = k * (T - T_ref) * D
        // Minimize sum of (error - k * deltaT * D)^2

        float sumXY = 0.0f;  // sum of (error * deltaT * D)
        float sumX2 = 0.0f;  // sum of (deltaT * D)^2

        for (uint16_t i = 0; i < count; i++) {
            float error = data[i].measuredDist_cm - data[i].referenceDist_cm;
            float deltaT = data[i].temperature_C - refTemp_C;
            float x = deltaT * data[i].referenceDist_cm;

            sumXY += error * x;
            sumX2 += x * x;
        }

        if (sumX2 > 0.001f) {
            return sumXY / sumX2;
        }
        return _tempCoeff;
    }

    /**
     * Set temperature coefficient
     */
    void setTempCoefficient(float coeff) { _tempCoeff = coeff; }

    /**
     * Get temperature coefficient
     */
    float getTempCoefficient() const { return _tempCoeff; }

private:
    float _refTemp;
    float _tempCoeff;
};

// ============================================================================
// Temperature Compensation Validator
// ============================================================================
class CompensationValidator {
public:
    /**
     * Constructor
     */
    CompensationValidator(uint16_t maxSamples = 100) :
        _maxSamples(maxSamples),
        _sampleCount(0) {}

    /**
     * Add a data point for validation
     */
    bool addDataPoint(const TempDataPoint& point) {
        if (_sampleCount >= _maxSamples) return false;
        _data[_sampleCount++] = point;
        return true;
    }

    /**
     * Validate ultrasonic compensation effectiveness
     */
    ValidationMetrics validateUltrasonic() {
        ValidationMetrics metrics = {};
        metrics.sampleCount = _sampleCount;

        if (_sampleCount < 5) return metrics;

        UltrasonicCompensation compensator;

        float sumRawError = 0.0f, sumCompError = 0.0f;
        float sumRawError2 = 0.0f, sumCompError2 = 0.0f;
        float maxRaw = 0.0f, maxComp = 0.0f;
        float minTemp = 100.0f, maxTemp = -100.0f;

        for (uint16_t i = 0; i < _sampleCount; i++) {
            // Raw error
            float rawError = fabs(_data[i].measuredDist_cm - _data[i].referenceDist_cm);
            sumRawError += rawError;
            sumRawError2 += rawError * rawError;
            if (rawError > maxRaw) maxRaw = rawError;

            // Compensated error
            CompensationResult comp = compensator.compensate(
                _data[i].measuredDist_cm, _data[i].temperature_C);
            float compError = fabs(comp.compensatedDistance_cm - _data[i].referenceDist_cm);
            sumCompError += compError;
            sumCompError2 += compError * compError;
            if (compError > maxComp) maxComp = compError;

            // Temperature range
            if (_data[i].temperature_C < minTemp) minTemp = _data[i].temperature_C;
            if (_data[i].temperature_C > maxTemp) maxTemp = _data[i].temperature_C;
        }

        // Calculate metrics
        float n = (float)_sampleCount;
        metrics.meanRawError_cm = sumRawError / n;
        metrics.meanCompError_cm = sumCompError / n;
        metrics.rmseRaw_cm = sqrt(sumRawError2 / n);
        metrics.rmseComp_cm = sqrt(sumCompError2 / n);
        metrics.maxRawError_cm = maxRaw;
        metrics.maxCompError_cm = maxComp;
        metrics.tempRange_C = maxTemp - minTemp;

        // Standard deviations
        float sumRawDev2 = 0.0f, sumCompDev2 = 0.0f;
        for (uint16_t i = 0; i < _sampleCount; i++) {
            float rawError = fabs(_data[i].measuredDist_cm - _data[i].referenceDist_cm);
            float rawDev = rawError - metrics.meanRawError_cm;
            sumRawDev2 += rawDev * rawDev;

            CompensationResult comp = compensator.compensate(
                _data[i].measuredDist_cm, _data[i].temperature_C);
            float compError = fabs(comp.compensatedDistance_cm - _data[i].referenceDist_cm);
            float compDev = compError - metrics.meanCompError_cm;
            sumCompDev2 += compDev * compDev;
        }
        metrics.stdRawError_cm = sqrt(sumRawDev2 / (n - 1));
        metrics.stdCompError_cm = sqrt(sumCompDev2 / (n - 1));

        // Error reduction
        if (metrics.meanRawError_cm > 0.001f) {
            metrics.errorReduction_pct = 100.0f *
                (1.0f - metrics.meanCompError_cm / metrics.meanRawError_cm);
        }

        // Significance (simplified check)
        metrics.significantImprovement =
            (metrics.errorReduction_pct > 20.0f && _sampleCount >= 30);

        return metrics;
    }

    /**
     * Validate LiDAR compensation effectiveness
     */
    ValidationMetrics validateLiDAR() {
        ValidationMetrics metrics = {};
        metrics.sampleCount = _sampleCount;

        if (_sampleCount < 5) return metrics;

        LiDARCompensation compensator;

        float sumRawError = 0.0f, sumCompError = 0.0f;
        float sumRawError2 = 0.0f, sumCompError2 = 0.0f;
        float maxRaw = 0.0f, maxComp = 0.0f;
        float minTemp = 100.0f, maxTemp = -100.0f;

        for (uint16_t i = 0; i < _sampleCount; i++) {
            float rawError = fabs(_data[i].measuredDist_cm - _data[i].referenceDist_cm);
            sumRawError += rawError;
            sumRawError2 += rawError * rawError;
            if (rawError > maxRaw) maxRaw = rawError;

            CompensationResult comp = compensator.compensate(
                _data[i].measuredDist_cm, _data[i].temperature_C);
            float compError = fabs(comp.compensatedDistance_cm - _data[i].referenceDist_cm);
            sumCompError += compError;
            sumCompError2 += compError * compError;
            if (compError > maxComp) maxComp = compError;

            if (_data[i].temperature_C < minTemp) minTemp = _data[i].temperature_C;
            if (_data[i].temperature_C > maxTemp) maxTemp = _data[i].temperature_C;
        }

        float n = (float)_sampleCount;
        metrics.meanRawError_cm = sumRawError / n;
        metrics.meanCompError_cm = sumCompError / n;
        metrics.rmseRaw_cm = sqrt(sumRawError2 / n);
        metrics.rmseComp_cm = sqrt(sumCompError2 / n);
        metrics.maxRawError_cm = maxRaw;
        metrics.maxCompError_cm = maxComp;
        metrics.tempRange_C = maxTemp - minTemp;

        if (metrics.meanRawError_cm > 0.001f) {
            metrics.errorReduction_pct = 100.0f *
                (1.0f - metrics.meanCompError_cm / metrics.meanRawError_cm);
        }

        metrics.significantImprovement =
            (metrics.errorReduction_pct > 10.0f && _sampleCount >= 30);

        return metrics;
    }

    /**
     * Generate validation report
     */
    void printValidationReport(const ValidationMetrics& metrics, const char* sensorType) {
        Serial.println(F("\n========================================"));
        Serial.print(F("Temperature Compensation Validation: "));
        Serial.println(sensorType);
        Serial.println(F("========================================"));

        Serial.print(F("Sample Count: ")); Serial.println(metrics.sampleCount);
        Serial.print(F("Temperature Range: ")); Serial.print(metrics.tempRange_C, 1);
        Serial.println(F(" °C"));

        Serial.println(F("\n--- Raw Measurements ---"));
        Serial.print(F("Mean Error: ")); Serial.print(metrics.meanRawError_cm, 3);
        Serial.println(F(" cm"));
        Serial.print(F("Std Dev: ")); Serial.print(metrics.stdRawError_cm, 3);
        Serial.println(F(" cm"));
        Serial.print(F("RMSE: ")); Serial.print(metrics.rmseRaw_cm, 3);
        Serial.println(F(" cm"));
        Serial.print(F("Max Error: ")); Serial.print(metrics.maxRawError_cm, 3);
        Serial.println(F(" cm"));

        Serial.println(F("\n--- Compensated Measurements ---"));
        Serial.print(F("Mean Error: ")); Serial.print(metrics.meanCompError_cm, 3);
        Serial.println(F(" cm"));
        Serial.print(F("Std Dev: ")); Serial.print(metrics.stdCompError_cm, 3);
        Serial.println(F(" cm"));
        Serial.print(F("RMSE: ")); Serial.print(metrics.rmseComp_cm, 3);
        Serial.println(F(" cm"));
        Serial.print(F("Max Error: ")); Serial.print(metrics.maxCompError_cm, 3);
        Serial.println(F(" cm"));

        Serial.println(F("\n--- Improvement ---"));
        Serial.print(F("Error Reduction: ")); Serial.print(metrics.errorReduction_pct, 1);
        Serial.println(F(" %"));
        Serial.print(F("Statistically Significant: "));
        Serial.println(metrics.significantImprovement ? F("Yes") : F("No"));
        Serial.println(F("========================================\n"));
    }

    /**
     * Clear all data
     */
    void clear() {
        _sampleCount = 0;
    }

    /**
     * Get sample count
     */
    uint16_t getSampleCount() const { return _sampleCount; }

private:
    TempDataPoint _data[100];  // Fixed size for embedded systems
    uint16_t _maxSamples;
    uint16_t _sampleCount;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Estimate measurement error based on temperature deviation
 * Useful for uncertainty quantification
 */
inline float estimateUltrasonicError(float distance_cm, float tempDeviation_C) {
    return UltrasonicCompensation::calculateExpectedError(distance_cm, tempDeviation_C);
}

/**
 * Calculate humidity impact on speed of sound (optional correction)
 * Effect is small (~0.1% at 100% humidity) but can improve precision
 */
inline float humidityCorrection(float speedOfSound_ms, float humidity_pct,
                                  float temperature_C) {
    // Water vapor has lower molecular weight than air
    // Humid air has slightly higher sound speed
    // Correction: v_humid = v_dry * (1 + 0.0016 * h)
    // where h is water vapor mass ratio
    // Simplified: ~0.0003% per 1% humidity
    float correction = 1.0f + (0.0003f * humidity_pct / 100.0f);
    return speedOfSound_ms * correction;
}

#endif // TEMPERATURE_COMPENSATION_H

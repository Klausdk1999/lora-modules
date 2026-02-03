/**
 * SensorFilters.h - Combined Sensor Filtering Helper
 *
 * Integrates multiple filtering techniques for river level sensor readings:
 * 1. Moving Average - Simple statistical smoothing
 * 2. Kalman Filter - Optimal state estimation with uncertainty tracking
 * 3. Adaptive Sampling - Dynamic transmission intervals
 *
 * Provides a unified interface for firmware to apply both filters
 * and transmit filtered values alongside raw readings.
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 *
 * References:
 * - Kabi et al. (2023): Statistical filtering for noise reduction
 * - Wu et al. (2023): Multi-view sensor fusion for water monitoring
 * - Ragnoli et al. (2020): Adaptive duty cycling for smart sensor networks
 */

#ifndef SENSOR_FILTERS_H
#define SENSOR_FILTERS_H

#include <Arduino.h>
#include "MovingAverage.h"
#include "AdaptiveSampling.h"
#include "../fusion/KalmanFilter.h"

// ============================================================================
// Default Configuration
// ============================================================================
#define DEFAULT_MA_WINDOW_SIZE      5     // Moving average window size
#define DEFAULT_OUTLIER_THRESHOLD   20.0f // Outlier threshold (%)

// ============================================================================
// Filtered Reading Result
// ============================================================================
struct FilteredReading {
    // Raw measurement
    float rawValue;             // Raw sensor reading (cm)

    // Moving average filter results
    float movingAverage;        // Moving average value (cm)
    float maStdDev;             // Moving average standard deviation
    uint8_t maSampleCount;      // Number of samples in MA window

    // Kalman filter results
    float kalmanFiltered;       // Kalman filtered value (cm)
    float kalmanUncertainty;    // Kalman filter uncertainty (std dev)
    float kalmanGain;           // Last Kalman gain used

    // Metadata
    bool valid;                 // Whether reading is valid
    int16_t signalStrength;     // Sensor signal strength (for LiDAR)
    int8_t temperature;         // Sensor temperature
    uint32_t timestampMs;       // Measurement timestamp
};

// ============================================================================
// Extended Payload Structure for LoRa Transmission
// ============================================================================
// Extends the base payload with filtered values
// Total size: 16 bytes (fits within LoRaWAN SF7 optimal payload)
struct __attribute__((packed)) FilteredSensorPayload {
    uint8_t sensorType;         // 1 = LiDAR, 2 = Ultrasonic, 0xFF = error
    uint16_t rawDistanceMm;     // Raw distance in millimeters
    uint16_t maDistanceMm;      // Moving average distance (mm)
    uint16_t kalmanDistanceMm;  // Kalman filtered distance (mm)
    int16_t signalStrength;     // Signal strength
    int8_t temperature;         // Temperature (C)
    uint8_t batteryPercent;     // Battery level (0-100)
    uint8_t readingCount;       // Valid readings in average
    uint8_t flags;              // Status flags
    // Flags: bit0=rapid_change, bit1=critical_level, bit2=battery_low,
    //        bit3=sensor_error, bit4=kalman_initialized
};

// ============================================================================
// Single Sensor Filter Chain
// ============================================================================
/**
 * Applies both Moving Average and Kalman filtering to a single sensor.
 * Use one instance per sensor type.
 *
 * @tparam MA_WINDOW Moving average window size (default: 5)
 */
template<uint8_t MA_WINDOW = DEFAULT_MA_WINDOW_SIZE>
class SensorFilterChain {
public:
    /**
     * Constructor
     *
     * @param kalmanMeasurementNoise Measurement noise variance for Kalman filter
     * @param kalmanProcessNoise Process noise variance for Kalman filter
     */
    SensorFilterChain(float kalmanMeasurementNoise = KALMAN_LIDAR_NOISE,
                       float kalmanProcessNoise = KALMAN_PROCESS_NOISE_NORMAL)
        : _kalmanFilter(kalmanMeasurementNoise, kalmanProcessNoise),
          _lastRawValue(0.0f), _outlierThreshold(DEFAULT_OUTLIER_THRESHOLD) {}

    /**
     * Reset all filters to initial state
     */
    void reset() {
        _movingAverage.reset();
        _kalmanFilter.reset();
        _lastRawValue = 0.0f;
    }

    /**
     * Process a new sensor reading through both filters
     *
     * @param rawValue Raw sensor reading (cm)
     * @param signalStrength Sensor signal strength (-1 if not available)
     * @param temperature Sensor temperature (C, -128 if not available)
     * @return FilteredReading with all filter outputs
     */
    FilteredReading process(float rawValue, int16_t signalStrength = -1,
                            int8_t temperature = -128) {
        FilteredReading result = {};
        result.timestampMs = millis();
        result.signalStrength = signalStrength;
        result.temperature = temperature;

        // Check for valid reading
        if (rawValue <= 0) {
            result.valid = false;
            result.rawValue = rawValue;
            return result;
        }

        result.valid = true;
        result.rawValue = rawValue;

        // Check for outlier (if we have enough history)
        bool isOutlier = false;
        if (_movingAverage.isFull()) {
            float currentAvg = _movingAverage.getAverage();
            float deviation = abs(rawValue - currentAvg);
            float threshold = currentAvg * (_outlierThreshold / 100.0f);
            isOutlier = (deviation > threshold);
        }

        // Update moving average (even with outliers, for tracking)
        if (!isOutlier) {
            _movingAverage.update(rawValue);
        }

        result.movingAverage = _movingAverage.getAverage();
        result.maStdDev = _movingAverage.getStdDev();
        result.maSampleCount = _movingAverage.getSampleCount();

        // Update Kalman filter
        KalmanResult kr = _kalmanFilter.filterWithDetails(rawValue, result.timestampMs);
        result.kalmanFiltered = kr.estimate;
        result.kalmanUncertainty = kr.uncertainty;
        result.kalmanGain = kr.kalmanGain;

        _lastRawValue = rawValue;

        return result;
    }

    /**
     * Get current moving average without adding new sample
     */
    float getMovingAverage() const {
        return _movingAverage.getAverage();
    }

    /**
     * Get current Kalman estimate without adding new sample
     */
    float getKalmanEstimate() const {
        return _kalmanFilter.getEstimate();
    }

    /**
     * Get Kalman filter uncertainty
     */
    float getKalmanUncertainty() const {
        return _kalmanFilter.getUncertainty();
    }

    /**
     * Check if filters have been initialized
     */
    bool isInitialized() const {
        return _kalmanFilter.isInitialized() && _movingAverage.getSampleCount() > 0;
    }

    /**
     * Set outlier rejection threshold (percentage)
     */
    void setOutlierThreshold(float thresholdPercent) {
        _outlierThreshold = thresholdPercent;
    }

    /**
     * Enable/disable rapid change mode (increases Kalman process noise)
     */
    void setRapidChangeMode(bool enabled) {
        _kalmanFilter.setRapidChangeMode(enabled);
    }

    /**
     * Get moving average object for direct access
     */
    MovingAverage<MA_WINDOW>& getMovingAverageFilter() {
        return _movingAverage;
    }

    /**
     * Get Kalman filter object for direct access
     */
    KalmanFilterSingle& getKalmanFilter() {
        return _kalmanFilter;
    }

private:
    MovingAverage<MA_WINDOW> _movingAverage;
    KalmanFilterSingle _kalmanFilter;
    float _lastRawValue;
    float _outlierThreshold;
};

// ============================================================================
// Integrated Sensor Processor
// ============================================================================
/**
 * Complete sensor processing chain with adaptive sampling.
 * Combines filtering with transmission interval calculation.
 */
class IntegratedSensorProcessor {
public:
    /**
     * Constructor with default LiDAR configuration
     */
    IntegratedSensorProcessor()
        : _filterChain(KALMAN_LIDAR_NOISE, KALMAN_PROCESS_NOISE_NORMAL),
          _adaptiveSampler() {}

    /**
     * Constructor with custom Kalman parameters
     */
    IntegratedSensorProcessor(float measurementNoise, float processNoise)
        : _filterChain(measurementNoise, processNoise),
          _adaptiveSampler() {}

    /**
     * Reset all state
     */
    void reset() {
        _filterChain.reset();
        _adaptiveSampler.reset();
    }

    /**
     * Process a sensor reading and calculate next interval
     *
     * @param rawDistanceCm Raw sensor distance (cm)
     * @param sensorHeightCm Sensor installation height (cm)
     * @param batteryPercent Current battery level (0-100)
     * @param signalStrength Sensor signal strength (-1 if N/A)
     * @param temperature Sensor temperature (-128 if N/A)
     * @return Tuple of (FilteredReading, SamplingResult)
     */
    struct ProcessResult {
        FilteredReading filtered;
        SamplingResult sampling;
        float riverLevelCm;
    };

    ProcessResult process(float rawDistanceCm, float sensorHeightCm,
                          uint8_t batteryPercent, int16_t signalStrength = -1,
                          int8_t temperature = -128) {
        ProcessResult result = {};

        // Apply filter chain
        result.filtered = _filterChain.process(rawDistanceCm, signalStrength, temperature);

        // Calculate river level (using Kalman estimate for stability)
        float distanceForLevel = result.filtered.valid ?
            result.filtered.kalmanFiltered : rawDistanceCm;
        result.riverLevelCm = (distanceForLevel < sensorHeightCm) ?
            (sensorHeightCm - distanceForLevel) : 0.0f;

        // Calculate adaptive interval
        result.sampling = _adaptiveSampler.calculateInterval(
            distanceForLevel, result.riverLevelCm, batteryPercent);

        return result;
    }

    /**
     * Build a payload for LoRa transmission
     *
     * @param reading Filtered reading from process()
     * @param samplingResult Sampling result from process()
     * @param batteryPercent Battery level
     * @param sensorType Sensor type identifier
     * @return Populated payload structure
     */
    FilteredSensorPayload buildPayload(const FilteredReading& reading,
                                        const SamplingResult& samplingResult,
                                        uint8_t batteryPercent,
                                        uint8_t sensorType = 1) {
        FilteredSensorPayload payload = {};

        if (!reading.valid) {
            payload.sensorType = 0xFF;
            payload.rawDistanceMm = 0xFFFF;
            payload.maDistanceMm = 0xFFFF;
            payload.kalmanDistanceMm = 0xFFFF;
            payload.flags = 0x08;  // Sensor error flag
            payload.batteryPercent = batteryPercent;
            return payload;
        }

        payload.sensorType = sensorType;
        payload.rawDistanceMm = (uint16_t)(reading.rawValue * 10.0f);
        payload.maDistanceMm = (uint16_t)(reading.movingAverage * 10.0f);
        payload.kalmanDistanceMm = (uint16_t)(reading.kalmanFiltered * 10.0f);
        payload.signalStrength = reading.signalStrength;
        payload.temperature = reading.temperature;
        payload.batteryPercent = batteryPercent;
        payload.readingCount = reading.maSampleCount;

        // Build flags
        payload.flags = 0;
        if (samplingResult.rapidChangeDetected) payload.flags |= 0x01;
        if (samplingResult.criticalLevel) payload.flags |= 0x02;
        if (samplingResult.batteryLow) payload.flags |= 0x04;
        if (_filterChain.getKalmanFilter().isInitialized()) payload.flags |= 0x10;

        return payload;
    }

    /**
     * Get the filter chain for direct access
     */
    SensorFilterChain<DEFAULT_MA_WINDOW_SIZE>& getFilterChain() {
        return _filterChain;
    }

    /**
     * Get the adaptive sampler for direct access
     */
    AdaptiveSampler& getAdaptiveSampler() {
        return _adaptiveSampler;
    }

    /**
     * Set adaptive sampling configuration
     */
    void setAdaptiveConfig(const AdaptiveSamplingConfig& config) {
        _adaptiveSampler.setConfig(config);
    }

private:
    SensorFilterChain<DEFAULT_MA_WINDOW_SIZE> _filterChain;
    AdaptiveSampler _adaptiveSampler;
};

// ============================================================================
// Factory Functions for Common Sensor Types
// ============================================================================

/**
 * Create a filter chain configured for TF02-Pro/TF-Nova LiDAR
 */
inline SensorFilterChain<5> createLidarFilterChain() {
    return SensorFilterChain<5>(KALMAN_LIDAR_NOISE, KALMAN_PROCESS_NOISE_NORMAL);
}

/**
 * Create a filter chain configured for JSN-SR04T/AJ-SR04M Ultrasonic
 */
inline SensorFilterChain<5> createUltrasonicFilterChain(bool tempCompensated = false) {
    float noise = tempCompensated ? KALMAN_ULTRASONIC_COMP_NOISE : KALMAN_ULTRASONIC_NOISE;
    return SensorFilterChain<5>(noise, KALMAN_PROCESS_NOISE_NORMAL);
}

/**
 * Create an integrated processor for LiDAR sensors
 */
inline IntegratedSensorProcessor createLidarProcessor() {
    return IntegratedSensorProcessor(KALMAN_LIDAR_NOISE, KALMAN_PROCESS_NOISE_NORMAL);
}

/**
 * Create an integrated processor for Ultrasonic sensors
 */
inline IntegratedSensorProcessor createUltrasonicProcessor(bool tempCompensated = false) {
    float noise = tempCompensated ? KALMAN_ULTRASONIC_COMP_NOISE : KALMAN_ULTRASONIC_NOISE;
    return IntegratedSensorProcessor(noise, KALMAN_PROCESS_NOISE_NORMAL);
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Apply simple median filter to an array
 * Useful for pre-processing before the filter chain
 *
 * @param readings Array of readings
 * @param count Number of readings
 * @return Median value
 */
inline float calculateMedianFilter(float readings[], uint8_t count) {
    if (count == 0) return 0.0f;
    if (count == 1) return readings[0];

    // Simple bubble sort for small arrays
    float sorted[16];  // Max 16 readings
    uint8_t n = (count > 16) ? 16 : count;

    for (uint8_t i = 0; i < n; i++) {
        sorted[i] = readings[i];
    }

    for (uint8_t i = 0; i < n - 1; i++) {
        for (uint8_t j = 0; j < n - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    if (n % 2 == 0) {
        return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0f;
    }
    return sorted[n / 2];
}

/**
 * Calculate rate of change between two readings
 *
 * @param currentCm Current reading (cm)
 * @param previousCm Previous reading (cm)
 * @param elapsedMs Time elapsed (ms)
 * @return Rate of change in cm/min
 */
inline float calculateRateOfChange(float currentCm, float previousCm, uint32_t elapsedMs) {
    if (elapsedMs == 0) return 0.0f;
    float changeCm = currentCm - previousCm;
    return (changeCm / elapsedMs) * 60000.0f;  // Convert to cm/min
}

/**
 * Print filtered reading to Serial (for debugging)
 */
inline void printFilteredReading(const FilteredReading& reading) {
    Serial.println(F("--- Filtered Reading ---"));
    Serial.print(F("  Raw: "));
    Serial.print(reading.rawValue);
    Serial.println(F(" cm"));
    Serial.print(F("  Moving Avg: "));
    Serial.print(reading.movingAverage);
    Serial.print(F(" cm ("));
    Serial.print(reading.maSampleCount);
    Serial.println(F(" samples)"));
    Serial.print(F("  Kalman: "));
    Serial.print(reading.kalmanFiltered);
    Serial.print(F(" cm (+/- "));
    Serial.print(reading.kalmanUncertainty);
    Serial.println(F(" cm)"));
    Serial.print(F("  Signal: "));
    Serial.print(reading.signalStrength);
    Serial.print(F(", Temp: "));
    Serial.print(reading.temperature);
    Serial.println(F(" C"));
}

#endif // SENSOR_FILTERS_H

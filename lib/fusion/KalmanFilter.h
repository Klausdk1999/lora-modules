/**
 * KalmanFilter.h - Kalman Filter for Single-Sensor and Multi-Sensor Fusion
 *
 * Provides flexible Kalman filtering that can be used:
 * 1. Separately for each sensor (LiDAR-only or Ultrasonic-only filtering)
 * 2. For combined multi-sensor fusion (LiDAR + Ultrasonic)
 *
 * The Kalman filter provides optimal state estimation by combining:
 * - Prediction from a simple motion model (assuming slow water level changes)
 * - Updates from sensor measurements weighted by their uncertainties
 *
 * Mathematical Foundation:
 * - State: x = [distance] (1D)
 * - Prediction: x_pred = x, P_pred = P + Q
 * - Update: K = P / (P + R), x = x + K * (z - x), P = (1-K) * P
 *
 * References:
 * - Wu et al. (2023) - Multi-view sensor fusion for water monitoring
 * - Welch & Bishop (1995) - An Introduction to the Kalman Filter
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>
#include <math.h>

// ============================================================================
// Default Noise Parameters (can be overridden)
// ============================================================================

// LiDAR (TF02-Pro, TF-Nova) - High accuracy, lower noise
// Typical accuracy: ±1-3 cm at most ranges
#define KALMAN_LIDAR_NOISE              4.0f    // R = variance (cm^2), ~2cm std dev
#define KALMAN_LIDAR_MIN_SIGNAL         100     // Minimum valid signal strength

// Ultrasonic (JSN-SR04T, AJ-SR04M) - Lower accuracy, higher noise
// Typical accuracy: ±1% of distance or ±1cm, whichever is greater
#define KALMAN_ULTRASONIC_NOISE         25.0f   // R = variance (cm^2), ~5cm std dev
#define KALMAN_ULTRASONIC_COMP_NOISE    16.0f   // Lower noise when temp-compensated

// Process noise - How much we expect measurement to change between readings
#define KALMAN_PROCESS_NOISE_NORMAL     1.0f    // Q = variance (cm^2), ~1cm std dev
#define KALMAN_PROCESS_NOISE_RAPID      9.0f    // Q during rapid change events

// ============================================================================
// Kalman State Structure
// ============================================================================
struct KalmanState {
    float estimate;         // Current state estimate (cm)
    float errorCovariance;  // Estimation error covariance (P)
    float processNoise;     // Process noise variance (Q)
    float measurementNoise; // Measurement noise variance (R)
    uint32_t lastUpdateMs;  // Timestamp of last update
    uint16_t updateCount;   // Number of updates since initialization
    bool initialized;       // Whether filter has been initialized
};

// ============================================================================
// Filter Result Structure
// ============================================================================
struct KalmanResult {
    float estimate;         // Filtered estimate (cm)
    float uncertainty;      // Estimation uncertainty (std dev in cm)
    float kalmanGain;       // Kalman gain used (0-1)
    float innovation;       // Measurement innovation (z - x_pred)
    bool valid;             // Whether result is valid
};

// ============================================================================
// Single Sensor Kalman Filter
// ============================================================================
/**
 * Simple 1D Kalman filter for filtering a single sensor's readings.
 * Use one instance per sensor for individual filtering.
 *
 * Example usage:
 *   KalmanFilterSingle lidarFilter(KALMAN_LIDAR_NOISE);
 *   KalmanFilterSingle ultrasonicFilter(KALMAN_ULTRASONIC_NOISE);
 *
 *   float filteredLidar = lidarFilter.filter(rawLidarReading);
 *   float filteredUltrasonic = ultrasonicFilter.filter(rawUltrasonicReading);
 */
class KalmanFilterSingle {
public:
    /**
     * Constructor with measurement noise
     * @param measurementNoise Measurement noise variance (R) in cm^2
     * @param processNoise Process noise variance (Q) in cm^2
     */
    KalmanFilterSingle(float measurementNoise = KALMAN_ULTRASONIC_NOISE,
                       float processNoise = KALMAN_PROCESS_NOISE_NORMAL) {
        _state.measurementNoise = measurementNoise;
        _state.processNoise = processNoise;
        reset();
    }

    /**
     * Reset filter to initial state
     */
    void reset() {
        _state.estimate = 0.0f;
        _state.errorCovariance = 1000.0f;  // High initial uncertainty
        _state.lastUpdateMs = 0;
        _state.updateCount = 0;
        _state.initialized = false;
    }

    /**
     * Filter a single measurement (predict + update in one call)
     * @param measurement New measurement in cm
     * @param timestamp Optional timestamp (uses millis() if 0)
     * @return Filtered estimate in cm
     */
    float filter(float measurement, uint32_t timestamp = 0) {
        KalmanResult result = filterWithDetails(measurement, timestamp);
        return result.estimate;
    }

    /**
     * Filter with full result details
     * @param measurement New measurement in cm
     * @param timestamp Optional timestamp (uses millis() if 0)
     * @return KalmanResult with estimate, uncertainty, and gain
     */
    KalmanResult filterWithDetails(float measurement, uint32_t timestamp = 0) {
        KalmanResult result = {};

        if (timestamp == 0) timestamp = millis();

        // First measurement - initialize
        if (!_state.initialized) {
            _state.estimate = measurement;
            _state.errorCovariance = _state.measurementNoise;
            _state.lastUpdateMs = timestamp;
            _state.updateCount = 1;
            _state.initialized = true;

            result.estimate = measurement;
            result.uncertainty = sqrt(_state.errorCovariance);
            result.kalmanGain = 1.0f;
            result.innovation = 0.0f;
            result.valid = true;
            return result;
        }

        // Predict step
        float dtSeconds = (timestamp - _state.lastUpdateMs) / 1000.0f;
        if (dtSeconds < 0.001f) dtSeconds = 0.001f;

        // Process noise scales with time
        float scaledQ = _state.processNoise * sqrt(dtSeconds);
        float P_pred = _state.errorCovariance + scaledQ;

        // Update step
        float K = P_pred / (P_pred + _state.measurementNoise);
        K = constrain(K, 0.01f, 0.99f);

        float innovation = measurement - _state.estimate;
        _state.estimate = _state.estimate + K * innovation;
        _state.errorCovariance = (1.0f - K) * P_pred;

        _state.lastUpdateMs = timestamp;
        _state.updateCount++;

        result.estimate = _state.estimate;
        result.uncertainty = sqrt(_state.errorCovariance);
        result.kalmanGain = K;
        result.innovation = innovation;
        result.valid = true;

        return result;
    }

    /**
     * Predict only (increase uncertainty without measurement)
     * Useful when sensor reading is unavailable
     * @param dtSeconds Time since last update in seconds
     */
    void predict(float dtSeconds) {
        if (!_state.initialized) return;

        float scaledQ = _state.processNoise * sqrt(dtSeconds);
        _state.errorCovariance += scaledQ;
        _state.lastUpdateMs = millis();
    }

    /**
     * Get current filtered estimate
     */
    float getEstimate() const {
        return _state.estimate;
    }

    /**
     * Get current uncertainty (standard deviation)
     */
    float getUncertainty() const {
        return sqrt(_state.errorCovariance);
    }

    /**
     * Check if filter is initialized
     */
    bool isInitialized() const {
        return _state.initialized;
    }

    /**
     * Get update count
     */
    uint16_t getUpdateCount() const {
        return _state.updateCount;
    }

    /**
     * Get full state (for debugging/persistence)
     */
    const KalmanState& getState() const {
        return _state;
    }

    /**
     * Set state (for restoring from persistence)
     */
    void setState(const KalmanState& state) {
        _state = state;
    }

    /**
     * Set measurement noise (R)
     */
    void setMeasurementNoise(float R) {
        _state.measurementNoise = R;
    }

    /**
     * Set process noise (Q)
     */
    void setProcessNoise(float Q) {
        _state.processNoise = Q;
    }

    /**
     * Enable/disable rapid change mode
     */
    void setRapidChangeMode(bool enabled) {
        _state.processNoise = enabled ?
            KALMAN_PROCESS_NOISE_RAPID : KALMAN_PROCESS_NOISE_NORMAL;
    }

private:
    KalmanState _state;
};

// ============================================================================
// Sensor Type Enumeration
// ============================================================================
enum class SensorType {
    LIDAR,
    LIDAR_LOW_SIGNAL,
    ULTRASONIC,
    ULTRASONIC_TEMP_COMPENSATED
};

// ============================================================================
// Multi-Sensor Fusion Result
// ============================================================================
struct FusionResult {
    float fusedDistance;        // Fused distance estimate (cm)
    float uncertainty;          // Estimation uncertainty (std dev in cm)
    float lidarWeight;          // Weight given to LiDAR (0-1)
    float ultrasonicWeight;     // Weight given to ultrasonic (0-1)
    float innovation;           // Last measurement innovation
    uint8_t sensorFlags;        // Which sensors contributed (bit 0=LiDAR, bit 1=ultrasonic)
    bool valid;                 // Whether fusion result is valid
};

// Sensor flags for FusionResult
#define FUSION_FLAG_LIDAR       0x01
#define FUSION_FLAG_ULTRASONIC  0x02

// ============================================================================
// Multi-Sensor Fusion Kalman Filter
// ============================================================================
/**
 * Kalman filter for fusing multiple sensor readings.
 * Can work with both sensors or just one.
 *
 * Example usage (fusion mode):
 *   KalmanFilterFusion fusion;
 *   FusionResult result = fusion.fuse(lidarDist, ultrasonicDist, lidarSignal);
 *
 * Example usage (single sensor via fusion):
 *   KalmanFilterFusion lidarOnly;
 *   FusionResult result = lidarOnly.fuse(lidarDist, -1);  // -1 = no ultrasonic
 */
class KalmanFilterFusion {
public:
    /**
     * Constructor with default parameters
     */
    KalmanFilterFusion() :
        _processNoise(KALMAN_PROCESS_NOISE_NORMAL),
        _lidarNoise(KALMAN_LIDAR_NOISE),
        _ultrasonicNoise(KALMAN_ULTRASONIC_NOISE),
        _ultrasonicCompNoise(KALMAN_ULTRASONIC_COMP_NOISE) {
        reset();
    }

    /**
     * Constructor with custom noise parameters
     * @param processNoise Process noise variance (Q)
     * @param lidarNoise LiDAR measurement noise variance (R)
     * @param ultrasonicNoise Ultrasonic measurement noise variance (R)
     */
    KalmanFilterFusion(float processNoise, float lidarNoise, float ultrasonicNoise) :
        _processNoise(processNoise),
        _lidarNoise(lidarNoise),
        _ultrasonicNoise(ultrasonicNoise),
        _ultrasonicCompNoise(ultrasonicNoise * 0.64f) {  // ~20% less noise when compensated
        reset();
    }

    /**
     * Reset the filter
     */
    void reset() {
        _estimate = 0.0f;
        _errorCovariance = 1000.0f;
        _currentProcessNoise = _processNoise;
        _lastUpdateMs = 0;
        _updateCount = 0;
        _initialized = false;
    }

    /**
     * Fuse sensor readings
     *
     * Pass -1 for any sensor that doesn't have a valid reading.
     * The filter will use whatever sensors are available.
     *
     * @param lidarDistCm LiDAR distance in cm (-1 if unavailable)
     * @param ultrasonicDistCm Ultrasonic distance in cm (-1 if unavailable)
     * @param lidarSignal LiDAR signal strength (-1 if unavailable)
     * @param tempCompensated Whether ultrasonic is temperature-compensated
     * @return FusionResult with fused estimate
     */
    FusionResult fuse(float lidarDistCm, float ultrasonicDistCm,
                      int16_t lidarSignal = -1, bool tempCompensated = false) {
        FusionResult result = {};
        result.valid = false;
        result.sensorFlags = 0;

        bool hasLidar = (lidarDistCm > 0);
        bool hasUltrasonic = (ultrasonicDistCm > 0);

        if (!hasLidar && !hasUltrasonic) {
            return result;  // No valid readings
        }

        uint32_t now = millis();

        // Prediction step (if initialized)
        if (_initialized) {
            float dtSeconds = (now - _lastUpdateMs) / 1000.0f;
            if (dtSeconds < 0.001f) dtSeconds = 0.001f;
            float scaledQ = _currentProcessNoise * sqrt(dtSeconds);
            _errorCovariance += scaledQ;
        }

        float totalGain = 0.0f;
        float lidarGain = 0.0f;
        float ultrasonicGain = 0.0f;

        // Initialize with first valid measurement
        if (!_initialized) {
            if (hasLidar) {
                _estimate = lidarDistCm;
                _errorCovariance = getLidarNoise(lidarSignal);
                result.sensorFlags |= FUSION_FLAG_LIDAR;
                lidarGain = 1.0f;
            } else {
                _estimate = ultrasonicDistCm;
                _errorCovariance = getUltrasonicNoise(tempCompensated);
                result.sensorFlags |= FUSION_FLAG_ULTRASONIC;
                ultrasonicGain = 1.0f;
            }
            _initialized = true;
            _updateCount = 1;
        } else {
            // Sequential updates with available sensors

            // Update with LiDAR first (typically more accurate)
            if (hasLidar) {
                float R = getLidarNoise(lidarSignal);
                float K = _errorCovariance / (_errorCovariance + R);
                K = constrain(K, 0.01f, 0.99f);

                result.innovation = lidarDistCm - _estimate;
                _estimate = _estimate + K * result.innovation;
                _errorCovariance = (1.0f - K) * _errorCovariance;

                lidarGain = K;
                result.sensorFlags |= FUSION_FLAG_LIDAR;
            }

            // Update with Ultrasonic
            if (hasUltrasonic) {
                float R = getUltrasonicNoise(tempCompensated);
                float K = _errorCovariance / (_errorCovariance + R);
                K = constrain(K, 0.01f, 0.99f);

                float innovation = ultrasonicDistCm - _estimate;
                if (!hasLidar) result.innovation = innovation;

                _estimate = _estimate + K * innovation;
                _errorCovariance = (1.0f - K) * _errorCovariance;

                ultrasonicGain = K;
                result.sensorFlags |= FUSION_FLAG_ULTRASONIC;
            }

            _updateCount++;
        }

        _lastUpdateMs = now;

        // Calculate relative weights
        totalGain = lidarGain + ultrasonicGain;
        if (totalGain > 0) {
            result.lidarWeight = lidarGain / totalGain;
            result.ultrasonicWeight = ultrasonicGain / totalGain;
        }

        result.fusedDistance = _estimate;
        result.uncertainty = sqrt(_errorCovariance);
        result.valid = true;

        return result;
    }

    /**
     * Update with a single sensor (convenience method)
     * @param measurement Sensor reading in cm
     * @param sensorType Type of sensor
     * @param signalQuality Signal quality (for LiDAR, -1 otherwise)
     * @return Filtered estimate
     */
    float update(float measurement, SensorType sensorType, int16_t signalQuality = -1) {
        float R = getMeasurementNoise(sensorType, signalQuality);

        uint32_t now = millis();

        if (!_initialized) {
            _estimate = measurement;
            _errorCovariance = R;
            _lastUpdateMs = now;
            _updateCount = 1;
            _initialized = true;
            return _estimate;
        }

        // Predict
        float dtSeconds = (now - _lastUpdateMs) / 1000.0f;
        if (dtSeconds < 0.001f) dtSeconds = 0.001f;
        float scaledQ = _currentProcessNoise * sqrt(dtSeconds);
        _errorCovariance += scaledQ;

        // Update
        float K = _errorCovariance / (_errorCovariance + R);
        K = constrain(K, 0.01f, 0.99f);

        _estimate = _estimate + K * (measurement - _estimate);
        _errorCovariance = (1.0f - K) * _errorCovariance;

        _lastUpdateMs = now;
        _updateCount++;

        return _estimate;
    }

    /**
     * Get current estimate
     */
    float getEstimate() const { return _estimate; }

    /**
     * Get current uncertainty
     */
    float getUncertainty() const { return sqrt(_errorCovariance); }

    /**
     * Check if initialized
     */
    bool isInitialized() const { return _initialized; }

    /**
     * Get update count
     */
    uint16_t getUpdateCount() const { return _updateCount; }

    /**
     * Set rapid change mode
     */
    void setRapidChangeMode(bool enabled) {
        _currentProcessNoise = enabled ? KALMAN_PROCESS_NOISE_RAPID : _processNoise;
    }

    /**
     * Set noise parameters
     */
    void setNoiseParameters(float processNoise, float lidarNoise, float ultrasonicNoise) {
        _processNoise = processNoise;
        _currentProcessNoise = processNoise;
        _lidarNoise = lidarNoise;
        _ultrasonicNoise = ultrasonicNoise;
    }

private:
    float _estimate;
    float _errorCovariance;
    float _processNoise;
    float _currentProcessNoise;
    float _lidarNoise;
    float _ultrasonicNoise;
    float _ultrasonicCompNoise;
    uint32_t _lastUpdateMs;
    uint16_t _updateCount;
    bool _initialized;

    float getLidarNoise(int16_t signalQuality) const {
        if (signalQuality >= 0 && signalQuality < KALMAN_LIDAR_MIN_SIGNAL) {
            return _lidarNoise * 4.0f;  // Very low signal
        } else if (signalQuality >= 0 && signalQuality < 500) {
            return _lidarNoise * 2.0f;  // Below average signal
        }
        return _lidarNoise;
    }

    float getUltrasonicNoise(bool tempCompensated) const {
        return tempCompensated ? _ultrasonicCompNoise : _ultrasonicNoise;
    }

    float getMeasurementNoise(SensorType type, int16_t signalQuality) const {
        switch (type) {
            case SensorType::LIDAR:
                return getLidarNoise(signalQuality);
            case SensorType::LIDAR_LOW_SIGNAL:
                return _lidarNoise * 4.0f;
            case SensorType::ULTRASONIC_TEMP_COMPENSATED:
                return _ultrasonicCompNoise;
            case SensorType::ULTRASONIC:
            default:
                return _ultrasonicNoise;
        }
    }
};

// ============================================================================
// Convenience Factory Functions
// ============================================================================

/**
 * Create a Kalman filter configured for LiDAR sensor
 */
inline KalmanFilterSingle createLidarFilter() {
    return KalmanFilterSingle(KALMAN_LIDAR_NOISE, KALMAN_PROCESS_NOISE_NORMAL);
}

/**
 * Create a Kalman filter configured for Ultrasonic sensor
 */
inline KalmanFilterSingle createUltrasonicFilter(bool tempCompensated = false) {
    float noise = tempCompensated ? KALMAN_ULTRASONIC_COMP_NOISE : KALMAN_ULTRASONIC_NOISE;
    return KalmanFilterSingle(noise, KALMAN_PROCESS_NOISE_NORMAL);
}

/**
 * Create a multi-sensor fusion filter
 */
inline KalmanFilterFusion createFusionFilter() {
    return KalmanFilterFusion();
}

// ============================================================================
// Simple Fusion Alternatives (stateless)
// ============================================================================

/**
 * Simple weighted average fusion (no state, single call)
 * @param lidarDistCm LiDAR distance (-1 if invalid)
 * @param ultrasonicDistCm Ultrasonic distance (-1 if invalid)
 * @param lidarWeight Weight for LiDAR (0-1)
 * @return Fused distance, or -1 if no valid readings
 */
inline float simpleWeightedFusion(float lidarDistCm, float ultrasonicDistCm,
                                   float lidarWeight = 0.7f) {
    bool hasLidar = (lidarDistCm > 0);
    bool hasUltrasonic = (ultrasonicDistCm > 0);

    if (hasLidar && hasUltrasonic) {
        return (lidarDistCm * lidarWeight) + (ultrasonicDistCm * (1.0f - lidarWeight));
    } else if (hasLidar) {
        return lidarDistCm;
    } else if (hasUltrasonic) {
        return ultrasonicDistCm;
    }
    return -1.0f;
}

/**
 * Inverse variance weighted fusion (stateless, optimal for independent measurements)
 * @param lidarDistCm LiDAR distance (-1 if invalid)
 * @param ultrasonicDistCm Ultrasonic distance (-1 if invalid)
 * @param lidarVariance LiDAR variance (default from constants)
 * @param ultrasonicVariance Ultrasonic variance (default from constants)
 * @return Fused distance, or -1 if no valid readings
 */
inline float inverseVarianceFusion(float lidarDistCm, float ultrasonicDistCm,
                                    float lidarVariance = KALMAN_LIDAR_NOISE,
                                    float ultrasonicVariance = KALMAN_ULTRASONIC_NOISE) {
    bool hasLidar = (lidarDistCm > 0);
    bool hasUltrasonic = (ultrasonicDistCm > 0);

    if (hasLidar && hasUltrasonic) {
        float wL = 1.0f / lidarVariance;
        float wU = 1.0f / ultrasonicVariance;
        float wTotal = wL + wU;
        return (lidarDistCm * wL + ultrasonicDistCm * wU) / wTotal;
    } else if (hasLidar) {
        return lidarDistCm;
    } else if (hasUltrasonic) {
        return ultrasonicDistCm;
    }
    return -1.0f;
}

// ============================================================================
// Sensor Consistency Check
// ============================================================================

/**
 * Check if two sensor readings are consistent
 * @param reading1 First sensor reading (cm)
 * @param reading2 Second sensor reading (cm)
 * @param toleranceCm Maximum acceptable difference (default: 10cm)
 * @return true if readings are consistent
 */
inline bool areSensorsConsistent(float reading1, float reading2, float toleranceCm = 10.0f) {
    if (reading1 < 0 || reading2 < 0) return true;  // Can't compare invalid
    return fabs(reading1 - reading2) <= toleranceCm;
}

/**
 * Select best reading when sensors disagree
 * @param lidarDistCm LiDAR distance
 * @param ultrasonicDistCm Ultrasonic distance
 * @param lidarSignal LiDAR signal strength
 * @param preferLidar Prefer LiDAR when both valid (default: true)
 * @return Best reading, or -1 if both invalid
 */
inline float selectBestReading(float lidarDistCm, float ultrasonicDistCm,
                                int16_t lidarSignal, bool preferLidar = true) {
    bool lidarValid = (lidarDistCm > 0 && lidarSignal >= KALMAN_LIDAR_MIN_SIGNAL);
    bool ultrasonicValid = (ultrasonicDistCm > 0);

    if (lidarValid && ultrasonicValid) {
        return preferLidar ? lidarDistCm : ultrasonicDistCm;
    } else if (lidarValid) {
        return lidarDistCm;
    } else if (ultrasonicValid) {
        return ultrasonicDistCm;
    }
    return -1.0f;
}

#endif // KALMAN_FILTER_H

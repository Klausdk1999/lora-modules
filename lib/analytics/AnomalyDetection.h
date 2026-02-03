/**
 * AnomalyDetection.h - Statistical Anomaly Detection for Water Level Monitoring
 *
 * Implements multiple anomaly detection algorithms suitable for embedded systems:
 * - Z-Score based detection (statistical outliers)
 * - IQR (Interquartile Range) method
 * - Moving Average Deviation detection
 * - Rate of Change detection
 * - Combined ensemble detection
 *
 * These methods help distinguish:
 * - Sensor faults from real events
 * - Transient noise from actual water level changes
 * - Gradual drift from sudden anomalies
 *
 * References:
 * - Kabi et al. (2023) - Need for ML filtering in noisy environments
 * - Standard statistical methods for time series anomaly detection
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef ANOMALY_DETECTION_H
#define ANOMALY_DETECTION_H

#include <Arduino.h>
#include <math.h>

// ============================================================================
// Configuration Constants
// ============================================================================
#define ANOMALY_HISTORY_SIZE    20      // Number of readings to keep in history
#define MIN_HISTORY_FOR_STATS   5       // Minimum readings before detection works
#define DEFAULT_ZSCORE_THRESHOLD 3.0f   // Standard 3-sigma rule
#define DEFAULT_IQR_MULTIPLIER  1.5f    // Standard IQR outlier multiplier
#define DEFAULT_MA_WINDOW       5       // Moving average window size
#define DEFAULT_RATE_THRESHOLD  10.0f   // cm/minute rate of change threshold

// ============================================================================
// Anomaly Types
// ============================================================================
enum class AnomalyType : uint8_t {
    NONE = 0,               // No anomaly detected
    ZSCORE_HIGH = 1,        // Z-score above threshold (unusually high)
    ZSCORE_LOW = 2,         // Z-score below threshold (unusually low)
    IQR_OUTLIER = 3,        // IQR-based outlier
    RAPID_CHANGE = 4,       // Unusually rapid rate of change
    MA_DEVIATION = 5,       // Large deviation from moving average
    STUCK_SENSOR = 6,       // Sensor returning same value (stuck)
    OUT_OF_RANGE = 7,       // Value outside physical limits
    ENSEMBLE = 8            // Multiple methods agree on anomaly
};

// ============================================================================
// Anomaly Result Structure
// ============================================================================
struct AnomalyResult {
    bool isAnomaly;             // Whether an anomaly was detected
    AnomalyType type;           // Type of anomaly detected
    float anomalyScore;         // Severity score (0-1, higher = more anomalous)
    float zScore;               // Z-score of the reading
    float iqrDeviation;         // Deviation in IQR units
    float rateOfChange;         // Rate of change (cm/min)
    float maDeviation;          // Deviation from moving average
    uint8_t detectionFlags;     // Bit flags for which methods triggered
};

// Detection method flags
#define DETECT_FLAG_ZSCORE      0x01
#define DETECT_FLAG_IQR         0x02
#define DETECT_FLAG_RATE        0x04
#define DETECT_FLAG_MA          0x08
#define DETECT_FLAG_STUCK       0x10
#define DETECT_FLAG_RANGE       0x20

// ============================================================================
// Circular Buffer for History
// ============================================================================
template<typename T, uint8_t SIZE>
class CircularBuffer {
public:
    CircularBuffer() : _head(0), _count(0) {}

    void push(T value) {
        _buffer[_head] = value;
        _head = (_head + 1) % SIZE;
        if (_count < SIZE) _count++;
    }

    T get(uint8_t index) const {
        if (index >= _count) return T();
        uint8_t actualIndex = (_head - _count + index + SIZE) % SIZE;
        return _buffer[actualIndex];
    }

    T getLast() const {
        if (_count == 0) return T();
        return _buffer[(_head - 1 + SIZE) % SIZE];
    }

    T getSecondLast() const {
        if (_count < 2) return T();
        return _buffer[(_head - 2 + SIZE) % SIZE];
    }

    uint8_t count() const { return _count; }
    bool isFull() const { return _count >= SIZE; }

    void clear() {
        _head = 0;
        _count = 0;
    }

    // Get all values in order (oldest to newest)
    void getAll(T* output, uint8_t& outCount) const {
        outCount = _count;
        for (uint8_t i = 0; i < _count; i++) {
            output[i] = get(i);
        }
    }

private:
    T _buffer[SIZE];
    uint8_t _head;
    uint8_t _count;
};

// ============================================================================
// Reading with Timestamp
// ============================================================================
struct TimestampedReading {
    float value;
    uint32_t timestampMs;

    TimestampedReading() : value(0), timestampMs(0) {}
    TimestampedReading(float v, uint32_t t) : value(v), timestampMs(t) {}
};

// ============================================================================
// Statistics Calculator
// ============================================================================
class Statistics {
public:
    /**
     * Calculate mean of an array
     */
    static float mean(const float* values, uint8_t count) {
        if (count == 0) return 0.0f;
        float sum = 0.0f;
        for (uint8_t i = 0; i < count; i++) {
            sum += values[i];
        }
        return sum / count;
    }

    /**
     * Calculate variance of an array
     */
    static float variance(const float* values, uint8_t count, float mean) {
        if (count < 2) return 0.0f;
        float sumSq = 0.0f;
        for (uint8_t i = 0; i < count; i++) {
            float diff = values[i] - mean;
            sumSq += diff * diff;
        }
        return sumSq / (count - 1);  // Sample variance (n-1)
    }

    /**
     * Calculate standard deviation
     */
    static float stdDev(const float* values, uint8_t count, float mean) {
        return sqrt(variance(values, count, mean));
    }

    /**
     * Calculate Z-score
     */
    static float zScore(float value, float mean, float stdDev) {
        if (stdDev < 0.001f) return 0.0f;  // Avoid division by zero
        return (value - mean) / stdDev;
    }

    /**
     * Calculate percentile using linear interpolation
     * Requires sorted array
     */
    static float percentile(const float* sortedValues, uint8_t count, float p) {
        if (count == 0) return 0.0f;
        if (count == 1) return sortedValues[0];

        float index = (p / 100.0f) * (count - 1);
        uint8_t lower = (uint8_t)index;
        uint8_t upper = lower + 1;
        if (upper >= count) upper = count - 1;

        float fraction = index - lower;
        return sortedValues[lower] + fraction * (sortedValues[upper] - sortedValues[lower]);
    }

    /**
     * Sort array in place (bubble sort for small arrays)
     */
    static void sort(float* values, uint8_t count) {
        for (uint8_t i = 0; i < count - 1; i++) {
            for (uint8_t j = 0; j < count - i - 1; j++) {
                if (values[j] > values[j + 1]) {
                    float temp = values[j];
                    values[j] = values[j + 1];
                    values[j + 1] = temp;
                }
            }
        }
    }

    /**
     * Calculate IQR (Interquartile Range)
     * Modifies the values array (sorts it)
     */
    static void iqr(float* values, uint8_t count, float& q1, float& q3, float& iqrValue) {
        if (count < 4) {
            q1 = q3 = iqrValue = 0.0f;
            return;
        }

        sort(values, count);
        q1 = percentile(values, count, 25.0f);
        q3 = percentile(values, count, 75.0f);
        iqrValue = q3 - q1;
    }
};

// ============================================================================
// Anomaly Detector Class
// ============================================================================
class AnomalyDetector {
public:
    /**
     * Constructor with default thresholds
     */
    AnomalyDetector() :
        _zScoreThreshold(DEFAULT_ZSCORE_THRESHOLD),
        _iqrMultiplier(DEFAULT_IQR_MULTIPLIER),
        _maWindow(DEFAULT_MA_WINDOW),
        _rateThreshold(DEFAULT_RATE_THRESHOLD),
        _minRange(0.0f),
        _maxRange(1000.0f),
        _stuckThreshold(0.1f),
        _stuckCount(0) {}

    /**
     * Set detection thresholds
     */
    void setThresholds(float zScoreThreshold, float iqrMultiplier,
                       float rateThreshold, float stuckThreshold = 0.1f) {
        _zScoreThreshold = zScoreThreshold;
        _iqrMultiplier = iqrMultiplier;
        _rateThreshold = rateThreshold;
        _stuckThreshold = stuckThreshold;
    }

    /**
     * Set valid range for out-of-range detection
     */
    void setValidRange(float minRange, float maxRange) {
        _minRange = minRange;
        _maxRange = maxRange;
    }

    /**
     * Set moving average window size
     */
    void setMAWindow(uint8_t window) {
        _maWindow = min(window, (uint8_t)ANOMALY_HISTORY_SIZE);
    }

    /**
     * Add a new reading and check for anomalies
     * @param value The new sensor reading (cm)
     * @param timestampMs Optional timestamp (uses millis() if 0)
     * @return AnomalyResult with detection details
     */
    AnomalyResult addReading(float value, uint32_t timestampMs = 0) {
        AnomalyResult result = {};
        result.isAnomaly = false;
        result.type = AnomalyType::NONE;
        result.anomalyScore = 0.0f;
        result.detectionFlags = 0;

        // Use current time if not provided
        if (timestampMs == 0) {
            timestampMs = millis();
        }

        // Check out of range first (always applies)
        if (value < _minRange || value > _maxRange) {
            result.isAnomaly = true;
            result.type = AnomalyType::OUT_OF_RANGE;
            result.detectionFlags |= DETECT_FLAG_RANGE;
            result.anomalyScore = 1.0f;
            // Still add to history for tracking
            _history.push(TimestampedReading(value, timestampMs));
            return result;
        }

        // Check for stuck sensor
        if (_history.count() > 0) {
            float lastValue = _history.getLast().value;
            if (fabs(value - lastValue) < _stuckThreshold) {
                _stuckCount++;
                if (_stuckCount >= 5) {
                    result.detectionFlags |= DETECT_FLAG_STUCK;
                }
            } else {
                _stuckCount = 0;
            }
        }

        // Add to history
        _history.push(TimestampedReading(value, timestampMs));

        // Need minimum history for statistical methods
        if (_history.count() < MIN_HISTORY_FOR_STATS) {
            return result;
        }

        // Extract values from history
        float values[ANOMALY_HISTORY_SIZE];
        uint8_t count;
        extractValues(values, count);

        // Calculate statistics
        float mean = Statistics::mean(values, count);
        float stdDev = Statistics::stdDev(values, count, mean);

        // Z-Score detection
        result.zScore = Statistics::zScore(value, mean, stdDev);
        if (fabs(result.zScore) > _zScoreThreshold) {
            result.detectionFlags |= DETECT_FLAG_ZSCORE;
            result.type = (result.zScore > 0) ? AnomalyType::ZSCORE_HIGH : AnomalyType::ZSCORE_LOW;
        }

        // IQR detection
        float q1, q3, iqr;
        Statistics::iqr(values, count, q1, q3, iqr);
        if (iqr > 0.001f) {
            float lowerBound = q1 - _iqrMultiplier * iqr;
            float upperBound = q3 + _iqrMultiplier * iqr;
            if (value < lowerBound || value > upperBound) {
                result.detectionFlags |= DETECT_FLAG_IQR;
                result.iqrDeviation = (value < lowerBound) ?
                    (lowerBound - value) / iqr :
                    (value - upperBound) / iqr;
                if (result.type == AnomalyType::NONE) {
                    result.type = AnomalyType::IQR_OUTLIER;
                }
            }
        }

        // Rate of change detection
        if (_history.count() >= 2) {
            TimestampedReading prev = _history.getSecondLast();
            float dtMinutes = (timestampMs - prev.timestampMs) / 60000.0f;
            if (dtMinutes > 0.001f) {
                result.rateOfChange = fabs(value - prev.value) / dtMinutes;
                if (result.rateOfChange > _rateThreshold) {
                    result.detectionFlags |= DETECT_FLAG_RATE;
                    if (result.type == AnomalyType::NONE) {
                        result.type = AnomalyType::RAPID_CHANGE;
                    }
                }
            }
        }

        // Moving average deviation
        float ma = calculateMA(values, count);
        result.maDeviation = fabs(value - ma);
        if (result.maDeviation > stdDev * _zScoreThreshold) {
            result.detectionFlags |= DETECT_FLAG_MA;
            if (result.type == AnomalyType::NONE) {
                result.type = AnomalyType::MA_DEVIATION;
            }
        }

        // Stuck sensor check
        if (result.detectionFlags & DETECT_FLAG_STUCK) {
            if (result.type == AnomalyType::NONE) {
                result.type = AnomalyType::STUCK_SENSOR;
            }
        }

        // Calculate overall anomaly score (0-1)
        result.anomalyScore = calculateAnomalyScore(result);

        // Determine if anomaly based on score or multiple flags
        uint8_t flagCount = countBits(result.detectionFlags);
        if (flagCount >= 2) {
            result.isAnomaly = true;
            if (flagCount >= 3) {
                result.type = AnomalyType::ENSEMBLE;
            }
        } else if (result.anomalyScore >= 0.7f) {
            result.isAnomaly = true;
        }

        return result;
    }

    /**
     * Get current moving average
     */
    float getMovingAverage() const {
        if (_history.count() < _maWindow) return 0.0f;

        float sum = 0.0f;
        for (uint8_t i = _history.count() - _maWindow; i < _history.count(); i++) {
            sum += _history.get(i).value;
        }
        return sum / _maWindow;
    }

    /**
     * Get current statistics
     */
    void getStatistics(float& mean, float& stdDev, float& min, float& max) const {
        if (_history.count() == 0) {
            mean = stdDev = min = max = 0.0f;
            return;
        }

        float values[ANOMALY_HISTORY_SIZE];
        uint8_t count;
        extractValues(values, count);

        mean = Statistics::mean(values, count);
        stdDev = Statistics::stdDev(values, count, mean);

        min = values[0];
        max = values[0];
        for (uint8_t i = 1; i < count; i++) {
            if (values[i] < min) min = values[i];
            if (values[i] > max) max = values[i];
        }
    }

    /**
     * Clear history and reset detector
     */
    void reset() {
        _history.clear();
        _stuckCount = 0;
    }

    /**
     * Get number of readings in history
     */
    uint8_t getHistoryCount() const {
        return _history.count();
    }

    /**
     * Check if detector is ready (has enough history)
     */
    bool isReady() const {
        return _history.count() >= MIN_HISTORY_FOR_STATS;
    }

private:
    CircularBuffer<TimestampedReading, ANOMALY_HISTORY_SIZE> _history;
    float _zScoreThreshold;
    float _iqrMultiplier;
    uint8_t _maWindow;
    float _rateThreshold;
    float _minRange;
    float _maxRange;
    float _stuckThreshold;
    uint8_t _stuckCount;

    /**
     * Extract values from history buffer
     */
    void extractValues(float* values, uint8_t& count) const {
        count = _history.count();
        for (uint8_t i = 0; i < count; i++) {
            values[i] = _history.get(i).value;
        }
    }

    /**
     * Calculate moving average of last N values
     */
    float calculateMA(const float* values, uint8_t count) const {
        uint8_t window = min(_maWindow, count);
        float sum = 0.0f;
        for (uint8_t i = count - window; i < count; i++) {
            sum += values[i];
        }
        return sum / window;
    }

    /**
     * Calculate composite anomaly score
     */
    float calculateAnomalyScore(const AnomalyResult& result) const {
        float score = 0.0f;

        // Z-score contribution (normalized to 0-1)
        if (fabs(result.zScore) > _zScoreThreshold) {
            score += min(1.0f, (fabs(result.zScore) - _zScoreThreshold) / _zScoreThreshold);
        }

        // IQR contribution
        if (result.iqrDeviation > 0) {
            score += min(1.0f, result.iqrDeviation / 2.0f);
        }

        // Rate contribution
        if (result.rateOfChange > _rateThreshold) {
            score += min(1.0f, (result.rateOfChange - _rateThreshold) / _rateThreshold);
        }

        // MA deviation contribution
        // (already factored into other metrics, minor boost)
        if (result.detectionFlags & DETECT_FLAG_MA) {
            score += 0.2f;
        }

        // Stuck sensor penalty
        if (result.detectionFlags & DETECT_FLAG_STUCK) {
            score += 0.3f;
        }

        // Normalize to 0-1
        return min(1.0f, score / 2.0f);
    }

    /**
     * Count set bits in flags
     */
    uint8_t countBits(uint8_t flags) const {
        uint8_t count = 0;
        while (flags) {
            count += flags & 1;
            flags >>= 1;
        }
        return count;
    }
};

// ============================================================================
// Anomaly Type to String Helper
// ============================================================================
inline const char* anomalyTypeToString(AnomalyType type) {
    switch (type) {
        case AnomalyType::NONE:         return "None";
        case AnomalyType::ZSCORE_HIGH:  return "Z-High";
        case AnomalyType::ZSCORE_LOW:   return "Z-Low";
        case AnomalyType::IQR_OUTLIER:  return "IQR";
        case AnomalyType::RAPID_CHANGE: return "Rapid";
        case AnomalyType::MA_DEVIATION: return "MA-Dev";
        case AnomalyType::STUCK_SENSOR: return "Stuck";
        case AnomalyType::OUT_OF_RANGE: return "Range";
        case AnomalyType::ENSEMBLE:     return "Multi";
        default:                        return "Unknown";
    }
}

#endif // ANOMALY_DETECTION_H

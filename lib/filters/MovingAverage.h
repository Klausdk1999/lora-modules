/**
 * MovingAverage.h - Simple Moving Average Filter
 *
 * A lightweight moving average implementation optimized for embedded systems.
 * Uses a circular buffer to efficiently compute rolling averages.
 *
 * Features:
 * - Configurable window size (template parameter)
 * - O(1) update complexity
 * - Handles initial fill gracefully (averages available samples)
 * - Provides access to variance and standard deviation
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 *
 * References:
 * - Kabi et al. (2023): Statistical filtering for noise reduction in water level monitoring
 */

#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <Arduino.h>
#include <math.h>

/**
 * Moving Average Filter with configurable window size
 *
 * @tparam WINDOW_SIZE Number of samples in the moving average window (default: 5)
 */
template<uint8_t WINDOW_SIZE = 5>
class MovingAverage {
public:
    MovingAverage() : _index(0), _count(0), _sum(0.0f), _sumSquares(0.0f) {
        for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
            _buffer[i] = 0.0f;
        }
    }

    /**
     * Reset the filter to initial state
     */
    void reset() {
        _index = 0;
        _count = 0;
        _sum = 0.0f;
        _sumSquares = 0.0f;
        for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
            _buffer[i] = 0.0f;
        }
    }

    /**
     * Add a new sample and update the moving average
     *
     * @param value New sample value
     * @return Current moving average after adding the new sample
     */
    float update(float value) {
        // Remove oldest value from sums if buffer is full
        if (_count >= WINDOW_SIZE) {
            float oldValue = _buffer[_index];
            _sum -= oldValue;
            _sumSquares -= oldValue * oldValue;
        } else {
            _count++;
        }

        // Add new value
        _buffer[_index] = value;
        _sum += value;
        _sumSquares += value * value;

        // Advance circular index
        _index = (_index + 1) % WINDOW_SIZE;

        return getAverage();
    }

    /**
     * Get the current moving average
     *
     * @return Moving average of samples in the window
     */
    float getAverage() const {
        if (_count == 0) return 0.0f;
        return _sum / _count;
    }

    /**
     * Get the variance of samples in the window
     *
     * @return Variance (population variance)
     */
    float getVariance() const {
        if (_count < 2) return 0.0f;
        float mean = getAverage();
        return (_sumSquares / _count) - (mean * mean);
    }

    /**
     * Get the standard deviation of samples in the window
     *
     * @return Standard deviation
     */
    float getStdDev() const {
        return sqrtf(getVariance());
    }

    /**
     * Get the number of samples currently in the buffer
     *
     * @return Number of valid samples (0 to WINDOW_SIZE)
     */
    uint8_t getSampleCount() const {
        return _count;
    }

    /**
     * Check if the buffer is fully filled
     *
     * @return true if buffer has WINDOW_SIZE samples
     */
    bool isFull() const {
        return _count >= WINDOW_SIZE;
    }

    /**
     * Get the most recent sample added
     *
     * @return Last sample value
     */
    float getLastSample() const {
        if (_count == 0) return 0.0f;
        // The last added sample is at (index - 1) in circular buffer
        uint8_t lastIdx = (_index == 0) ? (WINDOW_SIZE - 1) : (_index - 1);
        return _buffer[lastIdx];
    }

    /**
     * Get the oldest sample in the buffer
     *
     * @return Oldest sample value
     */
    float getOldestSample() const {
        if (_count == 0) return 0.0f;
        if (_count < WINDOW_SIZE) {
            return _buffer[0];  // Buffer not full, oldest is at start
        }
        return _buffer[_index];  // Buffer full, oldest is at current index
    }

    /**
     * Get the configured window size
     *
     * @return Window size (WINDOW_SIZE template parameter)
     */
    static constexpr uint8_t getWindowSize() {
        return WINDOW_SIZE;
    }

private:
    float _buffer[WINDOW_SIZE];    // Circular buffer for samples
    uint8_t _index;                // Current write position
    uint8_t _count;                // Number of valid samples
    float _sum;                    // Running sum for O(1) average
    float _sumSquares;             // Running sum of squares for variance
};

/**
 * Exponential Moving Average (EMA) Filter
 *
 * Alternative to simple moving average that gives more weight to recent samples.
 * Uses less memory (no buffer needed) but responds faster to changes.
 *
 * EMA formula: EMA_t = alpha * sample + (1 - alpha) * EMA_(t-1)
 */
class ExponentialMovingAverage {
public:
    /**
     * Constructor
     *
     * @param alpha Smoothing factor (0.0 to 1.0). Higher = more responsive, Lower = smoother
     *              Typical values: 0.1-0.3 for smooth filtering, 0.5-0.7 for responsive
     */
    explicit ExponentialMovingAverage(float alpha = 0.2f)
        : _alpha(alpha), _ema(0.0f), _initialized(false) {
        if (_alpha < 0.0f) _alpha = 0.0f;
        if (_alpha > 1.0f) _alpha = 1.0f;
    }

    /**
     * Reset the filter to initial state
     */
    void reset() {
        _ema = 0.0f;
        _initialized = false;
    }

    /**
     * Update with new sample
     *
     * @param value New sample value
     * @return Current EMA after update
     */
    float update(float value) {
        if (!_initialized) {
            _ema = value;
            _initialized = true;
        } else {
            _ema = _alpha * value + (1.0f - _alpha) * _ema;
        }
        return _ema;
    }

    /**
     * Get current EMA value
     *
     * @return Current exponential moving average
     */
    float getValue() const {
        return _ema;
    }

    /**
     * Check if filter has been initialized with at least one sample
     *
     * @return true if at least one sample has been processed
     */
    bool isInitialized() const {
        return _initialized;
    }

    /**
     * Set alpha (smoothing factor)
     *
     * @param alpha New smoothing factor (0.0 to 1.0)
     */
    void setAlpha(float alpha) {
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        _alpha = alpha;
    }

    /**
     * Get current alpha value
     *
     * @return Current smoothing factor
     */
    float getAlpha() const {
        return _alpha;
    }

private:
    float _alpha;          // Smoothing factor
    float _ema;            // Current EMA value
    bool _initialized;     // Whether first sample has been received
};

#endif // MOVING_AVERAGE_H

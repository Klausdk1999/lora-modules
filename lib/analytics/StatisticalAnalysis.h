/**
 * StatisticalAnalysis.h - Statistical Analysis Tools for Thesis Validation
 *
 * Comprehensive statistical analysis for validating sensor measurements:
 * - Confidence interval calculations (95% CI)
 * - Statistical significance tests (t-test, ANOVA)
 * - Effect size calculations (Cohen's d)
 * - Sample size recommendations
 * - Correlation analysis
 *
 * These tools support the scientific rigor requirements for thesis validation
 * with proper statistical methods and reporting.
 *
 * References:
 * - Mohammed et al. (2019) - Temperature compensation validation
 * - Standard statistical methods (Cohen 1988, Student's t-test)
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef STATISTICAL_ANALYSIS_H
#define STATISTICAL_ANALYSIS_H

#include <Arduino.h>
#include <math.h>

// ============================================================================
// Configuration Constants
// ============================================================================
#define MAX_SAMPLE_SIZE         100     // Maximum samples for analysis
#define MIN_SAMPLE_SIZE_TTEST   2       // Minimum for t-test
#define RECOMMENDED_SAMPLE_SIZE 30      // Recommended for normal approximation

// Common significance levels
#define ALPHA_005   0.05f   // 95% confidence
#define ALPHA_001   0.01f   // 99% confidence
#define ALPHA_0001  0.001f  // 99.9% confidence

// ============================================================================
// Descriptive Statistics Result
// ============================================================================
struct DescriptiveStats {
    float mean;             // Sample mean
    float variance;         // Sample variance
    float stdDev;           // Sample standard deviation
    float stdError;         // Standard error of the mean
    float min;              // Minimum value
    float max;              // Maximum value
    float range;            // Range (max - min)
    float median;           // Median value
    float q1;               // First quartile (25th percentile)
    float q3;               // Third quartile (75th percentile)
    float iqr;              // Interquartile range
    float skewness;         // Skewness coefficient
    float kurtosis;         // Kurtosis coefficient
    uint16_t n;             // Sample size
    bool valid;             // Whether calculations succeeded
};

// ============================================================================
// Confidence Interval Result
// ============================================================================
struct ConfidenceInterval {
    float lower;            // Lower bound
    float upper;            // Upper bound
    float margin;           // Margin of error
    float confidence;       // Confidence level (0.95 for 95%)
    bool valid;
};

// ============================================================================
// T-Test Result
// ============================================================================
struct TTestResult {
    float tStatistic;       // t-value
    float pValue;           // p-value (approximate)
    float meanDiff;         // Difference in means
    float pooledStdDev;     // Pooled standard deviation
    float effectSize;       // Cohen's d
    uint16_t df;            // Degrees of freedom
    bool significant;       // Whether significant at alpha=0.05
    bool valid;
};

// ============================================================================
// Correlation Result
// ============================================================================
struct CorrelationResult {
    float r;                // Pearson correlation coefficient
    float r2;               // R-squared (coefficient of determination)
    float tStatistic;       // t-statistic for significance
    float pValue;           // p-value (approximate)
    uint16_t n;             // Sample size
    bool significant;       // Whether significant at alpha=0.05
    bool valid;
};

// ============================================================================
// Effect Size Interpretation
// ============================================================================
enum class EffectSizeCategory {
    NEGLIGIBLE,     // |d| < 0.2
    SMALL,          // 0.2 <= |d| < 0.5
    MEDIUM,         // 0.5 <= |d| < 0.8
    LARGE           // |d| >= 0.8
};

// ============================================================================
// Statistical Analysis Class
// ============================================================================
class StatisticalAnalysis {
public:
    // ========================================================================
    // Descriptive Statistics
    // ========================================================================

    /**
     * Calculate comprehensive descriptive statistics
     * @param data Array of data values
     * @param n Number of values
     * @return DescriptiveStats structure
     */
    static DescriptiveStats descriptive(const float* data, uint16_t n) {
        DescriptiveStats result = {};
        result.valid = false;
        result.n = n;

        if (n == 0) return result;

        // Calculate mean
        float sum = 0.0f;
        for (uint16_t i = 0; i < n; i++) {
            sum += data[i];
        }
        result.mean = sum / n;

        // Calculate variance, min, max
        float sumSq = 0.0f;
        result.min = data[0];
        result.max = data[0];
        for (uint16_t i = 0; i < n; i++) {
            float diff = data[i] - result.mean;
            sumSq += diff * diff;
            if (data[i] < result.min) result.min = data[i];
            if (data[i] > result.max) result.max = data[i];
        }

        result.variance = (n > 1) ? sumSq / (n - 1) : 0.0f;  // Sample variance
        result.stdDev = sqrt(result.variance);
        result.stdError = result.stdDev / sqrt((float)n);
        result.range = result.max - result.min;

        // Calculate median and quartiles (need sorted copy)
        float sorted[MAX_SAMPLE_SIZE];
        uint16_t copyN = min(n, (uint16_t)MAX_SAMPLE_SIZE);
        for (uint16_t i = 0; i < copyN; i++) {
            sorted[i] = data[i];
        }
        sortArray(sorted, copyN);

        result.median = percentile(sorted, copyN, 50.0f);
        result.q1 = percentile(sorted, copyN, 25.0f);
        result.q3 = percentile(sorted, copyN, 75.0f);
        result.iqr = result.q3 - result.q1;

        // Calculate skewness and kurtosis
        if (n >= 3 && result.stdDev > 0.001f) {
            float sumCubed = 0.0f;
            float sumFourth = 0.0f;
            for (uint16_t i = 0; i < n; i++) {
                float z = (data[i] - result.mean) / result.stdDev;
                sumCubed += z * z * z;
                sumFourth += z * z * z * z;
            }
            result.skewness = sumCubed / n;
            result.kurtosis = (sumFourth / n) - 3.0f;  // Excess kurtosis
        }

        result.valid = true;
        return result;
    }

    // ========================================================================
    // Confidence Intervals
    // ========================================================================

    /**
     * Calculate confidence interval for the mean
     * Uses t-distribution for small samples, z for large
     * @param mean Sample mean
     * @param stdError Standard error of the mean
     * @param n Sample size
     * @param confidence Confidence level (default 0.95 for 95%)
     * @return ConfidenceInterval structure
     */
    static ConfidenceInterval confidenceInterval(float mean, float stdError,
                                                  uint16_t n, float confidence = 0.95f) {
        ConfidenceInterval result = {};
        result.confidence = confidence;
        result.valid = false;

        if (n < 2 || stdError < 0) return result;

        // Get critical value (t or z)
        float critical;
        if (n < 30) {
            // Use t-distribution critical values
            critical = getTCritical(n - 1, confidence);
        } else {
            // Use z critical values (normal approximation)
            critical = getZCritical(confidence);
        }

        result.margin = critical * stdError;
        result.lower = mean - result.margin;
        result.upper = mean + result.margin;
        result.valid = true;

        return result;
    }

    /**
     * Calculate CI from data array
     */
    static ConfidenceInterval confidenceIntervalFromData(const float* data, uint16_t n,
                                                          float confidence = 0.95f) {
        DescriptiveStats stats = descriptive(data, n);
        if (!stats.valid) {
            ConfidenceInterval invalid = {};
            return invalid;
        }
        return confidenceInterval(stats.mean, stats.stdError, n, confidence);
    }

    // ========================================================================
    // T-Tests
    // ========================================================================

    /**
     * Independent samples t-test (two groups)
     * Assumes equal variances (pooled variance)
     * @param data1 First group data
     * @param n1 First group size
     * @param data2 Second group data
     * @param n2 Second group size
     * @return TTestResult structure
     */
    static TTestResult independentTTest(const float* data1, uint16_t n1,
                                         const float* data2, uint16_t n2) {
        TTestResult result = {};
        result.valid = false;

        if (n1 < MIN_SAMPLE_SIZE_TTEST || n2 < MIN_SAMPLE_SIZE_TTEST) return result;

        // Calculate descriptive stats for both groups
        DescriptiveStats stats1 = descriptive(data1, n1);
        DescriptiveStats stats2 = descriptive(data2, n2);

        if (!stats1.valid || !stats2.valid) return result;

        // Calculate pooled variance
        float sp2 = ((n1 - 1) * stats1.variance + (n2 - 1) * stats2.variance) /
                    (n1 + n2 - 2);
        result.pooledStdDev = sqrt(sp2);

        // Calculate t-statistic
        result.meanDiff = stats1.mean - stats2.mean;
        float se = result.pooledStdDev * sqrt(1.0f/n1 + 1.0f/n2);
        result.tStatistic = (se > 0.001f) ? result.meanDiff / se : 0.0f;

        // Degrees of freedom
        result.df = n1 + n2 - 2;

        // Calculate effect size (Cohen's d)
        result.effectSize = (result.pooledStdDev > 0.001f) ?
            result.meanDiff / result.pooledStdDev : 0.0f;

        // Approximate p-value using t-distribution approximation
        result.pValue = approximateTDistPValue(fabs(result.tStatistic), result.df);

        // Significance at alpha=0.05
        result.significant = (result.pValue < ALPHA_005);
        result.valid = true;

        return result;
    }

    /**
     * One-sample t-test (compare to known value)
     * @param data Sample data
     * @param n Sample size
     * @param mu0 Hypothesized population mean
     * @return TTestResult structure
     */
    static TTestResult oneSampleTTest(const float* data, uint16_t n, float mu0) {
        TTestResult result = {};
        result.valid = false;

        if (n < MIN_SAMPLE_SIZE_TTEST) return result;

        DescriptiveStats stats = descriptive(data, n);
        if (!stats.valid) return result;

        // Calculate t-statistic
        result.meanDiff = stats.mean - mu0;
        result.tStatistic = (stats.stdError > 0.001f) ?
            result.meanDiff / stats.stdError : 0.0f;

        result.df = n - 1;
        result.pooledStdDev = stats.stdDev;

        // Effect size
        result.effectSize = (stats.stdDev > 0.001f) ?
            result.meanDiff / stats.stdDev : 0.0f;

        // Approximate p-value
        result.pValue = approximateTDistPValue(fabs(result.tStatistic), result.df);
        result.significant = (result.pValue < ALPHA_005);
        result.valid = true;

        return result;
    }

    /**
     * Paired samples t-test
     * @param data1 First measurements
     * @param data2 Second measurements (paired with first)
     * @param n Number of pairs
     * @return TTestResult structure
     */
    static TTestResult pairedTTest(const float* data1, const float* data2, uint16_t n) {
        if (n < MIN_SAMPLE_SIZE_TTEST) {
            TTestResult invalid = {};
            return invalid;
        }

        // Calculate differences
        float diffs[MAX_SAMPLE_SIZE];
        uint16_t copyN = min(n, (uint16_t)MAX_SAMPLE_SIZE);
        for (uint16_t i = 0; i < copyN; i++) {
            diffs[i] = data1[i] - data2[i];
        }

        // One-sample t-test on differences
        return oneSampleTTest(diffs, copyN, 0.0f);
    }

    // ========================================================================
    // Effect Size
    // ========================================================================

    /**
     * Calculate Cohen's d effect size
     * @param mean1 Mean of group 1
     * @param mean2 Mean of group 2
     * @param pooledStdDev Pooled standard deviation
     * @return Cohen's d value
     */
    static float cohensD(float mean1, float mean2, float pooledStdDev) {
        if (pooledStdDev < 0.001f) return 0.0f;
        return (mean1 - mean2) / pooledStdDev;
    }

    /**
     * Interpret effect size magnitude
     * @param d Cohen's d value
     * @return EffectSizeCategory enum
     */
    static EffectSizeCategory interpretEffectSize(float d) {
        float absD = fabs(d);
        if (absD < 0.2f) return EffectSizeCategory::NEGLIGIBLE;
        if (absD < 0.5f) return EffectSizeCategory::SMALL;
        if (absD < 0.8f) return EffectSizeCategory::MEDIUM;
        return EffectSizeCategory::LARGE;
    }

    /**
     * Get effect size category as string
     */
    static const char* effectSizeToString(EffectSizeCategory cat) {
        switch (cat) {
            case EffectSizeCategory::NEGLIGIBLE: return "Negligible";
            case EffectSizeCategory::SMALL:      return "Small";
            case EffectSizeCategory::MEDIUM:     return "Medium";
            case EffectSizeCategory::LARGE:      return "Large";
            default: return "Unknown";
        }
    }

    // ========================================================================
    // Correlation
    // ========================================================================

    /**
     * Calculate Pearson correlation coefficient
     * @param x First variable data
     * @param y Second variable data
     * @param n Number of data points
     * @return CorrelationResult structure
     */
    static CorrelationResult pearsonCorrelation(const float* x, const float* y, uint16_t n) {
        CorrelationResult result = {};
        result.valid = false;
        result.n = n;

        if (n < 3) return result;

        // Calculate means
        float sumX = 0.0f, sumY = 0.0f;
        for (uint16_t i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
        }
        float meanX = sumX / n;
        float meanY = sumY / n;

        // Calculate covariance and standard deviations
        float sumXY = 0.0f, sumX2 = 0.0f, sumY2 = 0.0f;
        for (uint16_t i = 0; i < n; i++) {
            float dx = x[i] - meanX;
            float dy = y[i] - meanY;
            sumXY += dx * dy;
            sumX2 += dx * dx;
            sumY2 += dy * dy;
        }

        float denominator = sqrt(sumX2 * sumY2);
        if (denominator < 0.001f) {
            result.r = 0.0f;
            result.r2 = 0.0f;
            result.valid = true;
            return result;
        }

        result.r = sumXY / denominator;
        result.r2 = result.r * result.r;

        // Calculate t-statistic for significance
        if (fabs(result.r) < 0.9999f && n > 2) {
            result.tStatistic = result.r * sqrt((n - 2) / (1 - result.r2));
            result.pValue = approximateTDistPValue(fabs(result.tStatistic), n - 2);
            result.significant = (result.pValue < ALPHA_005);
        }

        result.valid = true;
        return result;
    }

    // ========================================================================
    // Sample Size Recommendations
    // ========================================================================

    /**
     * Calculate required sample size for desired precision
     * @param estimatedStdDev Estimated standard deviation
     * @param desiredMargin Desired margin of error
     * @param confidence Confidence level (default 0.95)
     * @return Recommended sample size
     */
    static uint16_t requiredSampleSize(float estimatedStdDev, float desiredMargin,
                                        float confidence = 0.95f) {
        if (desiredMargin <= 0 || estimatedStdDev <= 0) return RECOMMENDED_SAMPLE_SIZE;

        float z = getZCritical(confidence);
        float n = pow((z * estimatedStdDev / desiredMargin), 2);

        return max((uint16_t)RECOMMENDED_SAMPLE_SIZE, (uint16_t)ceil(n));
    }

    /**
     * Calculate required sample size for detecting effect size
     * @param effectSize Expected effect size (Cohen's d)
     * @param power Desired statistical power (default 0.80)
     * @param alpha Significance level (default 0.05)
     * @return Recommended sample size per group
     */
    static uint16_t requiredSampleSizeForEffect(float effectSize, float power = 0.80f,
                                                  float alpha = 0.05f) {
        // Simplified formula: n ≈ 2 * ((z_alpha + z_power) / d)^2
        // Using approximations for common values
        float zAlpha = getZCritical(1.0f - alpha);
        float zPower = (power >= 0.80f) ? 0.84f : 0.52f;  // 80% or 50% power

        if (fabs(effectSize) < 0.1f) return 100;  // Need large sample for small effects

        float n = 2.0f * pow((zAlpha + zPower) / effectSize, 2);
        return max((uint16_t)RECOMMENDED_SAMPLE_SIZE, (uint16_t)ceil(n));
    }

    // ========================================================================
    // Temperature Compensation Validation
    // ========================================================================

    /**
     * Validate temperature compensation effectiveness
     * Compares error with and without compensation
     * @param rawReadings Raw sensor readings
     * @param compensatedReadings Temperature-compensated readings
     * @param referenceValues Reference (true) values
     * @param n Number of readings
     * @return Improvement ratio and statistics
     */
    struct CompensationValidation {
        float rawMeanError;         // Mean error without compensation
        float compensatedMeanError; // Mean error with compensation
        float rawRMSE;              // RMSE without compensation
        float compensatedRMSE;      // RMSE with compensation
        float improvementRatio;     // Ratio of error reduction
        float improvementPercent;   // Percentage improvement
        TTestResult significance;   // Statistical significance of improvement
        bool valid;
    };

    static CompensationValidation validateCompensation(const float* rawReadings,
                                                        const float* compensatedReadings,
                                                        const float* referenceValues,
                                                        uint16_t n) {
        CompensationValidation result = {};
        result.valid = false;

        if (n < 5) return result;

        // Calculate errors
        float rawErrors[MAX_SAMPLE_SIZE];
        float compErrors[MAX_SAMPLE_SIZE];
        uint16_t copyN = min(n, (uint16_t)MAX_SAMPLE_SIZE);

        float rawSumSq = 0.0f, compSumSq = 0.0f;
        float rawSum = 0.0f, compSum = 0.0f;

        for (uint16_t i = 0; i < copyN; i++) {
            rawErrors[i] = rawReadings[i] - referenceValues[i];
            compErrors[i] = compensatedReadings[i] - referenceValues[i];

            rawSum += fabs(rawErrors[i]);
            compSum += fabs(compErrors[i]);

            rawSumSq += rawErrors[i] * rawErrors[i];
            compSumSq += compErrors[i] * compErrors[i];
        }

        result.rawMeanError = rawSum / copyN;
        result.compensatedMeanError = compSum / copyN;
        result.rawRMSE = sqrt(rawSumSq / copyN);
        result.compensatedRMSE = sqrt(compSumSq / copyN);

        // Calculate improvement
        if (result.rawMeanError > 0.001f) {
            result.improvementRatio = result.rawMeanError / result.compensatedMeanError;
            result.improvementPercent = 100.0f * (1.0f - result.compensatedMeanError / result.rawMeanError);
        }

        // Test significance of improvement using paired t-test
        result.significance = pairedTTest(rawErrors, compErrors, copyN);

        result.valid = true;
        return result;
    }

private:
    /**
     * Sort array in place (insertion sort for small arrays)
     */
    static void sortArray(float* arr, uint16_t n) {
        for (uint16_t i = 1; i < n; i++) {
            float key = arr[i];
            int16_t j = i - 1;
            while (j >= 0 && arr[j] > key) {
                arr[j + 1] = arr[j];
                j--;
            }
            arr[j + 1] = key;
        }
    }

    /**
     * Calculate percentile from sorted array
     */
    static float percentile(const float* sorted, uint16_t n, float p) {
        if (n == 0) return 0.0f;
        if (n == 1) return sorted[0];

        float index = (p / 100.0f) * (n - 1);
        uint16_t lower = (uint16_t)index;
        uint16_t upper = lower + 1;
        if (upper >= n) upper = n - 1;

        float fraction = index - lower;
        return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
    }

    /**
     * Get z critical value for common confidence levels
     */
    static float getZCritical(float confidence) {
        // Common z-values (two-tailed)
        if (confidence >= 0.999f) return 3.291f;    // 99.9%
        if (confidence >= 0.99f)  return 2.576f;    // 99%
        if (confidence >= 0.95f)  return 1.960f;    // 95%
        if (confidence >= 0.90f)  return 1.645f;    // 90%
        return 1.960f;  // Default to 95%
    }

    /**
     * Get t critical value for common df and confidence levels
     * Lookup table for common values
     */
    static float getTCritical(uint16_t df, float confidence) {
        // Two-tailed t-critical values for 95% confidence
        // For small df, use lookup; for large df, approximate with z
        if (df >= 120) return getZCritical(confidence);

        // Lookup table for 95% CI
        if (confidence >= 0.95f) {
            static const float tTable95[] = {
                12.706f, 4.303f, 3.182f, 2.776f, 2.571f,  // df 1-5
                2.447f, 2.365f, 2.306f, 2.262f, 2.228f,   // df 6-10
                2.201f, 2.179f, 2.160f, 2.145f, 2.131f,   // df 11-15
                2.120f, 2.110f, 2.101f, 2.093f, 2.086f,   // df 16-20
                2.080f, 2.074f, 2.069f, 2.064f, 2.060f,   // df 21-25
                2.056f, 2.052f, 2.048f, 2.045f, 2.042f    // df 26-30
            };
            if (df <= 30) return tTable95[df - 1];
            return 1.96f + 0.5f / df;  // Approximation for larger df
        }

        // For 99% CI
        if (confidence >= 0.99f) {
            static const float tTable99[] = {
                63.657f, 9.925f, 5.841f, 4.604f, 4.032f,  // df 1-5
                3.707f, 3.499f, 3.355f, 3.250f, 3.169f    // df 6-10
            };
            if (df <= 10) return tTable99[df - 1];
            return 2.576f + 1.0f / df;  // Approximation
        }

        return getZCritical(confidence);
    }

    /**
     * Approximate p-value from t-distribution
     * Uses polynomial approximation
     */
    static float approximateTDistPValue(float t, uint16_t df) {
        if (df < 1) return 1.0f;

        // For large df, use normal approximation
        if (df > 100) {
            // Standard normal CDF approximation
            float z = fabs(t);
            float p = 0.5f * erfc(z / sqrt(2.0f));
            return 2.0f * p;  // Two-tailed
        }

        // For smaller df, use approximation based on Beta distribution
        // This is a simplified approximation
        float x = df / (df + t * t);
        float p = 0.5f * incompleteBeta(df / 2.0f, 0.5f, x);
        return 2.0f * min(p, 1.0f - p);  // Two-tailed
    }

    /**
     * Incomplete beta function approximation
     * Very simplified for embedded use
     */
    static float incompleteBeta(float a, float b, float x) {
        // Simple series approximation
        if (x <= 0.0f) return 0.0f;
        if (x >= 1.0f) return 1.0f;

        // Use a few terms of the series expansion
        float term = pow(x, a) * pow(1.0f - x, b) / a;
        float sum = term;

        for (int i = 1; i < 20; i++) {
            term *= (a + i - 1) * x / i;
            sum += term;
            if (fabs(term) < 0.0001f) break;
        }

        // Normalize (approximate)
        float beta = tgamma(a) * tgamma(b) / tgamma(a + b);
        return sum / beta;
    }
};

// ============================================================================
// Report Formatting Helpers
// ============================================================================

/**
 * Format descriptive statistics for output
 */
inline void printDescriptiveStats(const DescriptiveStats& stats) {
    if (!stats.valid) {
        Serial.println(F("Invalid statistics"));
        return;
    }

    Serial.println(F("\n=== Descriptive Statistics ==="));
    Serial.print(F("N: ")); Serial.println(stats.n);
    Serial.print(F("Mean: ")); Serial.println(stats.mean, 3);
    Serial.print(F("Std Dev: ")); Serial.println(stats.stdDev, 3);
    Serial.print(F("Std Error: ")); Serial.println(stats.stdError, 4);
    Serial.print(F("Min: ")); Serial.println(stats.min, 3);
    Serial.print(F("Max: ")); Serial.println(stats.max, 3);
    Serial.print(F("Range: ")); Serial.println(stats.range, 3);
    Serial.print(F("Median: ")); Serial.println(stats.median, 3);
    Serial.print(F("IQR: ")); Serial.println(stats.iqr, 3);
    Serial.print(F("Skewness: ")); Serial.println(stats.skewness, 3);
    Serial.print(F("Kurtosis: ")); Serial.println(stats.kurtosis, 3);
}

/**
 * Format confidence interval for output
 */
inline void printConfidenceInterval(const ConfidenceInterval& ci, float mean) {
    if (!ci.valid) {
        Serial.println(F("Invalid CI"));
        return;
    }

    Serial.print(F("95% CI: ["));
    Serial.print(ci.lower, 3);
    Serial.print(F(", "));
    Serial.print(ci.upper, 3);
    Serial.print(F("] (±"));
    Serial.print(ci.margin, 3);
    Serial.println(F(")"));
}

/**
 * Format t-test result for output
 */
inline void printTTestResult(const TTestResult& result) {
    if (!result.valid) {
        Serial.println(F("Invalid t-test"));
        return;
    }

    Serial.println(F("\n=== T-Test Result ==="));
    Serial.print(F("t(")); Serial.print(result.df);
    Serial.print(F(") = ")); Serial.println(result.tStatistic, 3);
    Serial.print(F("p-value: ")); Serial.println(result.pValue, 4);
    Serial.print(F("Mean Difference: ")); Serial.println(result.meanDiff, 3);
    Serial.print(F("Cohen's d: ")); Serial.print(result.effectSize, 3);
    Serial.print(F(" (")); Serial.print(StatisticalAnalysis::effectSizeToString(
        StatisticalAnalysis::interpretEffectSize(result.effectSize)));
    Serial.println(F(")"));
    Serial.print(F("Significant (p<0.05): "));
    Serial.println(result.significant ? F("Yes") : F("No"));
}

/**
 * Format correlation result for output
 */
inline void printCorrelationResult(const CorrelationResult& result) {
    if (!result.valid) {
        Serial.println(F("Invalid correlation"));
        return;
    }

    Serial.println(F("\n=== Correlation ==="));
    Serial.print(F("r = ")); Serial.println(result.r, 4);
    Serial.print(F("R² = ")); Serial.println(result.r2, 4);
    Serial.print(F("t = ")); Serial.println(result.tStatistic, 3);
    Serial.print(F("p-value: ")); Serial.println(result.pValue, 4);
    Serial.print(F("Significant (p<0.05): "));
    Serial.println(result.significant ? F("Yes") : F("No"));
}

#endif // STATISTICAL_ANALYSIS_H

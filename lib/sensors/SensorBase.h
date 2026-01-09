/**
 * SensorBase.h - Base class for all distance sensors
 * 
 * This provides a common interface for all sensors used in the
 * river level monitoring project. All sensor implementations should
 * inherit from this class.
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H

#include <Arduino.h>

// ============================================================================
// Sensor Types Enumeration
// ============================================================================
enum class SensorType {
    UNKNOWN = 0,
    // LiDAR sensors
    TF_LUNA,
    TF_MINI_PLUS,
    TF02_PRO,
    TF_NOVA,
    // Ultrasonic sensors
    HC_SR04,
    AJ_SR04M,
    JSN_SR04T
};

// ============================================================================
// Sensor Reading Structure
// ============================================================================
struct SensorReading {
    bool valid;              // Whether the reading is valid
    float distance_cm;       // Distance in centimeters
    float distance_m;        // Distance in meters
    int16_t signal_strength; // Signal strength/quality (sensor-specific)
    int16_t temperature;     // Temperature in Celsius (if available)
    uint32_t timestamp;      // Timestamp in milliseconds
    SensorType sensor_type;  // Type of sensor that produced this reading
    
    // Default constructor
    SensorReading() : 
        valid(false), 
        distance_cm(0), 
        distance_m(0),
        signal_strength(0), 
        temperature(0),
        timestamp(0),
        sensor_type(SensorType::UNKNOWN) {}
    
    // Constructor with distance
    SensorReading(float dist_cm, SensorType type) :
        valid(true),
        distance_cm(dist_cm),
        distance_m(dist_cm / 100.0f),
        signal_strength(0),
        temperature(0),
        timestamp(millis()),
        sensor_type(type) {}
};

// ============================================================================
// Sensor Base Class
// ============================================================================
class SensorBase {
public:
    /**
     * Initialize the sensor
     * @return true if initialization successful, false otherwise
     */
    virtual bool begin() = 0;
    
    /**
     * Read a measurement from the sensor
     * @return SensorReading struct with the measurement data
     */
    virtual SensorReading read() = 0;
    
    /**
     * Check if the sensor is connected and responding
     * @return true if sensor is available, false otherwise
     */
    virtual bool isConnected() = 0;
    
    /**
     * Get the sensor type
     * @return SensorType enum value
     */
    virtual SensorType getType() = 0;
    
    /**
     * Get the sensor name as a string
     * @return Sensor name
     */
    virtual const char* getName() = 0;
    
    /**
     * Get the minimum measurable distance in cm
     * @return Minimum distance in centimeters
     */
    virtual float getMinDistance() = 0;
    
    /**
     * Get the maximum measurable distance in cm
     * @return Maximum distance in centimeters
     */
    virtual float getMaxDistance() = 0;
    
    // Virtual destructor
    virtual ~SensorBase() {}
    
protected:
    bool _initialized = false;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Get sensor type name as string
 */
inline const char* getSensorTypeName(SensorType type) {
    switch(type) {
        case SensorType::TF_LUNA:      return "TF-Luna";
        case SensorType::TF_MINI_PLUS: return "TF-Mini Plus";
        case SensorType::TF02_PRO:     return "TF02 Pro";
        case SensorType::TF_NOVA:      return "TF-Nova";
        case SensorType::HC_SR04:      return "HC-SR04";
        case SensorType::AJ_SR04M:     return "AJ-SR04M";
        case SensorType::JSN_SR04T:    return "JSN-SR04T";
        default:                       return "Unknown";
    }
}

#endif // SENSOR_BASE_H







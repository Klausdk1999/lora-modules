/**
 * TFLuna.h - Benewake TF-Luna LiDAR Sensor Driver
 * 
 * The TF-Luna is a short-range LiDAR sensor with:
 * - Range: 0.2m to 8m
 * - Accuracy: ±6cm (0.2-3m), ±2% (3-8m)
 * - Interface: I2C or UART
 * - Frame rate: up to 250Hz
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef TF_LUNA_H
#define TF_LUNA_H

#include "../SensorBase.h"
#include <Wire.h>

// Default I2C address for TF-Luna
#define TFL_DEFAULT_ADDR    0x10
#define TFL_DEF_ADR         0x10

// TF-Luna I2C Register addresses
#define TFL_DIST_LO         0x00
#define TFL_DIST_HI         0x01
#define TFL_FLUX_LO         0x02
#define TFL_FLUX_HI         0x03
#define TFL_TEMP_LO         0x04
#define TFL_TEMP_HI         0x05
#define TFL_TICK_LO         0x06
#define TFL_TICK_HI         0x07

class TFLuna : public SensorBase {
public:
    /**
     * Constructor with default I2C address
     */
    TFLuna() : _i2cAddr(TFL_DEFAULT_ADDR), _wire(&Wire) {}
    
    /**
     * Constructor with custom I2C address
     * @param addr I2C address of the sensor
     */
    TFLuna(uint8_t addr) : _i2cAddr(addr), _wire(&Wire) {}
    
    /**
     * Constructor with custom I2C bus and address
     * @param wire Pointer to TwoWire instance
     * @param addr I2C address of the sensor
     */
    TFLuna(TwoWire* wire, uint8_t addr = TFL_DEFAULT_ADDR) : 
        _i2cAddr(addr), _wire(wire) {}
    
    /**
     * Initialize the sensor
     * @return true if sensor detected, false otherwise
     */
    bool begin() override;
    
    /**
     * Read distance, signal strength, and temperature
     * @return SensorReading with all data
     */
    SensorReading read() override;
    
    /**
     * Check if sensor is connected
     * @return true if sensor responds to I2C
     */
    bool isConnected() override;
    
    SensorType getType() override { return SensorType::TF_LUNA; }
    const char* getName() override { return "TF-Luna"; }
    float getMinDistance() override { return 20.0f; }  // 20 cm = 0.2m
    float getMaxDistance() override { return 800.0f; } // 800 cm = 8m
    
    // ========================================================================
    // Additional TF-Luna specific methods
    // ========================================================================
    
    /**
     * Read only the distance (faster than full read)
     * @return Distance in centimeters, or -1 if error
     */
    int16_t readDistance();
    
    /**
     * Read signal strength (flux)
     * Higher values indicate better signal quality
     * @return Signal strength value
     */
    int16_t readFlux();
    
    /**
     * Read chip temperature
     * @return Temperature in Celsius
     */
    int16_t readTemperature();
    
    /**
     * Get the last error code
     * @return Error code (0 = no error)
     */
    uint8_t getLastError() { return _lastError; }
    
    /**
     * Set a new I2C address for the sensor
     * @param newAddr New I2C address (0x08-0x77)
     * @return true if successful
     */
    bool setI2CAddress(uint8_t newAddr);
    
private:
    uint8_t _i2cAddr;
    TwoWire* _wire;
    uint8_t _lastError;
    
    int16_t _lastDist;
    int16_t _lastFlux;
    int16_t _lastTemp;
    
    bool readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t count);
    bool writeRegister(uint8_t reg, uint8_t value);
};

#endif // TF_LUNA_H







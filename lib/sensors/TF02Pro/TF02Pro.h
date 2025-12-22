/**
 * TF02Pro.h - Benewake TF02-Pro LiDAR Sensor Driver
 * 
 * The TF02-Pro is a mid-range LiDAR sensor with:
 * - Range: 0.1m to 22m (indoor), 0.1m to 12m (outdoor)
 * - Accuracy: ±1cm (0.1-6m), ±1% (6-22m)
 * - Interface: UART (default) or I2C
 * - Frame rate: 1-1000Hz (default 100Hz)
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef TF02_PRO_H
#define TF02_PRO_H

#include "../SensorBase.h"
#include <HardwareSerial.h>

// TF02-Pro default baud rate
#define TF02_DEFAULT_BAUD   115200

// TF02-Pro frame format
#define TF02_FRAME_HEADER   0x59
#define TF02_FRAME_SIZE     9

class TF02Pro : public SensorBase {
public:
    /**
     * Constructor with hardware serial
     * @param serial Reference to HardwareSerial (Serial1, Serial2, etc.)
     * @param rxPin RX pin number
     * @param txPin TX pin number
     */
    TF02Pro(HardwareSerial& serial, int rxPin = -1, int txPin = -1) :
        _serial(&serial), _rxPin(rxPin), _txPin(txPin), _baud(TF02_DEFAULT_BAUD) {}
    
    /**
     * Constructor with custom baud rate
     */
    TF02Pro(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud) :
        _serial(&serial), _rxPin(rxPin), _txPin(txPin), _baud(baud) {}
    
    /**
     * Initialize the sensor
     * @return true if initialization successful
     */
    bool begin() override;
    
    /**
     * Read distance measurement
     * @return SensorReading with measurement data
     */
    SensorReading read() override;
    
    /**
     * Check if sensor is connected (by trying to read a frame)
     * @return true if valid data received
     */
    bool isConnected() override;
    
    SensorType getType() override { return SensorType::TF02_PRO; }
    const char* getName() override { return "TF02-Pro"; }
    float getMinDistance() override { return 10.0f; }    // 10 cm = 0.1m
    float getMaxDistance() override { return 2200.0f; }  // 2200 cm = 22m
    
    // ========================================================================
    // Additional TF02-Pro specific methods
    // ========================================================================
    
    /**
     * Read only distance (same as read() but returns just the value)
     * @return Distance in centimeters, or -1 if error
     */
    int16_t readDistance();
    
    /**
     * Get signal strength from last reading
     * @return Signal strength (higher = better)
     */
    int16_t getSignalStrength() { return _lastStrength; }
    
    /**
     * Get temperature from last reading
     * @return Temperature in Celsius
     */
    int16_t getTemperature() { return _lastTemp; }
    
    /**
     * Set output frame rate
     * @param frameRate Hz (1-1000)
     * @return true if command successful
     */
    bool setFrameRate(uint16_t frameRate);
    
    /**
     * Reset sensor to factory defaults
     * @return true if command successful
     */
    bool factoryReset();
    
    /**
     * Clear the serial buffer
     */
    void clearBuffer();
    
private:
    HardwareSerial* _serial;
    int _rxPin;
    int _txPin;
    uint32_t _baud;
    
    int16_t _lastDist;
    int16_t _lastStrength;
    int16_t _lastTemp;
    
    bool readFrame(uint8_t* buffer);
    bool parseFrame(uint8_t* buffer);
    bool sendCommand(uint8_t* cmd, uint8_t len);
};

#endif // TF02_PRO_H



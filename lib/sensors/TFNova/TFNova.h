/**
 * TFNova.h - Benewake TF-Nova LiDAR Sensor Driver
 *
 * The TF-Nova is a compact LiDAR sensor with:
 * - Range: 0.1m to 12m (indoor), varies with reflectivity
 * - Accuracy: ±1cm typical
 * - Interface: UART (default) or I2C (via configuration)
 * - Default baud rate: 115200
 * - Frame rate: configurable
 *
 * UART Frame Format (9 bytes):
 * [0x59][0x59][Dist_L][Dist_H][Strength_L][Strength_H][Temp_L][Temp_H][Checksum]
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef TF_NOVA_H
#define TF_NOVA_H

#include "../SensorBase.h"
#include <HardwareSerial.h>

// TF-Nova default baud rate
#define TFNOVA_DEFAULT_BAUD     115200

// TF-Nova frame format
#define TFNOVA_FRAME_HEADER     0x59
#define TFNOVA_FRAME_SIZE       9

// Timeout for reading (ms)
#define TFNOVA_READ_TIMEOUT_MS  100

class TFNova : public SensorBase {
public:
    /**
     * Constructor with hardware serial
     * @param serial Reference to HardwareSerial (Serial1, Serial2, etc.)
     * @param rxPin RX pin number (receives from TF-Nova TX)
     * @param txPin TX pin number (sends to TF-Nova RX)
     */
    TFNova(HardwareSerial& serial, int rxPin = -1, int txPin = -1) :
        _serial(&serial), _rxPin(rxPin), _txPin(txPin), _baud(TFNOVA_DEFAULT_BAUD),
        _lastDist(0), _lastStrength(0), _lastTemp(0) {}

    /**
     * Constructor with custom baud rate
     */
    TFNova(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud) :
        _serial(&serial), _rxPin(rxPin), _txPin(txPin), _baud(baud),
        _lastDist(0), _lastStrength(0), _lastTemp(0) {}

    /**
     * Initialize the sensor
     * @return true if initialization successful and sensor responds
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

    SensorType getType() override { return SensorType::TF_LUNA; }  // Use TF_LUNA as closest match
    const char* getName() override { return "TF-Nova"; }
    float getMinDistance() override { return 10.0f; }    // 10 cm = 0.1m
    float getMaxDistance() override { return 1200.0f; }  // 1200 cm = 12m

    // ========================================================================
    // Additional TF-Nova specific methods
    // ========================================================================

    /**
     * Read only distance (simple version)
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
     * @return Temperature in Celsius (raw sensor value / 8 - 256)
     */
    int16_t getTemperature() { return _lastTemp; }

    /**
     * Set output frame rate
     * @param frameRate Hz (1-500 typical)
     * @return true if command successful
     */
    bool setFrameRate(uint16_t frameRate);

    /**
     * Save current configuration to sensor
     * @return true if command successful
     */
    bool saveConfig();

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

#endif // TF_NOVA_H

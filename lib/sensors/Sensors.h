/**
 * Sensors.h - Combined include file for all sensor drivers
 * 
 * Include this single file to access all sensor drivers in the library.
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef SENSORS_H
#define SENSORS_H

// Base class and common definitions
#include "SensorBase.h"

// LiDAR sensors (Benewake)
#include "TFLuna/TFLuna.h"
#include "TF02Pro/TF02Pro.h"

// Ultrasonic sensors
#include "HCSR04/HCSR04.h"
#include "AJSR04M/AJSR04M.h"
#include "JSNSR04T/JSNSR04T.h"

#endif // SENSORS_H



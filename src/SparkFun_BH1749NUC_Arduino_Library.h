/*
  This is a library written for the BH1749NUC Color Sensor
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/TODO
  Written by Jim Lindblom @ SparkFun Electronics, May 4th, 2018
  The BH1749NUC is a 16-bit RGB and IR color sensor that communicates via an I2C bus.
  The sensor is mounted on our QWIIC RGB Sensor Board, paired with a PCA9536 I/O expander
  which can flip the mounted red, green, blue, and white LED's on or off.
  
  This library handles the initialization of the BH1749NUC and is able to query the sensor
  for different readings.
  https://github.com/sparkfunX/SparkFun_BH1749NUC_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DEBUG_BH1749NUC

#include <Wire.h>

typedef enum {
    BH1749NUC_ADDRESS_CLOSED = 0x38,
    BH1749NUC_ADDRESS_OPEN = 0x39,
    BH1749NUC_ADDRESS_INVALID = 0xFF
} BH1749NUC_Address_t;

const BH1749NUC_Address_t BH1749NUC_ADDRESS_DEFAULT = BH1749NUC_ADDRESS_OPEN;

#define BH1749NUC_PART_ID 0x0D

// Registers
typedef enum {
    BH1749NUC_REGISTER_SYSTEM_CONTROL  = 0x40,
    BH1749NUC_REGISTER_MODE_CONTROL1   = 0x41,
    BH1749NUC_REGISTER_MODE_CONTROL2   = 0x42,
    BH1749NUC_REGISTER_RED_DATA_L      = 0x50,
    BH1749NUC_REGISTER_RED_DATA_H      = 0x51,
    BH1749NUC_REGISTER_GREEN_DATA_L    = 0x52,
    BH1749NUC_REGISTER_GREEN_DATA_H    = 0x53,
    BH1749NUC_REGISTER_BLUE_DATA_L     = 0x54,
    BH1749NUC_REGISTER_BLUE_DATA_H     = 0x55,
    BH1749NUC_REGISTER_RESERVED1       = 0x56,
    BH1749NUC_REGISTER_RESERVED2       = 0x57,
    BH1749NUC_REGISTER_IR_DATA_L       = 0x58,
    BH1749NUC_REGISTER_IR_DATA_H       = 0x59,
    BH1749NUC_REGISTER_GREEN2_DATA_L   = 0x5A,
    BH1749NUC_REGISTER_GREEN2_DATA_H   = 0x5B,
    BH1749NUC_REGISTER_INTERRUPT       = 0x60,
    BH1749NUC_REGISTER_PERSISTENCE     = 0x61,
    BH1749NUC_REGISTER_TH_HIGH_L       = 0x62,
    BH1749NUC_REGISTER_TH_HIGH_H       = 0x63,
    BH1749NUC_REGISTER_TH_LOW_L        = 0x64,
    BH1749NUC_REGISTER_TH_LOW_H        = 0x65,
    BH1749NUC_REGISTER_MANUFACTURER_ID = 0x92
} BH1749NUC_REGISTER_t;

typedef enum {
    BH1749NUC_GAIN_FORBIDDEN_0, // 0
    BH1749NUC_GAIN_X1,          // 1
    BH1749NUC_GAIN_FORBIDDEN_2, // 2
    BH1749NUC_GAIN_X32,         // 3,
    BH1749NUC_GAIN_INVALID
} BH1749NUC_gain_t;

typedef enum {
    BH1749NUC_MEASUREMENT_MODE_FORBIDDEN_0, // 0
    BH1749NUC_MEASUREMENT_MODE_FORBIDDEN_1, // 1
    BH1749NUC_MEASUREMENT_MODE_120_MS,      // 2
    BH1749NUC_MEASUREMENT_MODE_240_MS,      // 3
    BH1749NUC_MEASUREMENT_MODE_FORBIDDEN_4, // 4
    BH1749NUC_MEASUREMENT_MODE_35_MS,       // 5
    BH1749NUC_MEASUREMENT_MODE_FORBIDDEN_6, // 6
    BH1749NUC_MEASUREMENT_MODE_FORBIDDEN_7, // 7
    BH1749NUC_MEASUREMENT_MODE_INVALID,
} BH1749NUC_measurement_mode_t;

typedef enum {
    BH1749NUC_RED,
    BH1749NUC_GREEN,
    BH1749NUC_BLUE,
    BH1749NUC_IR,
    BH1749NUC_GREEN2,
    BH1749NUC_INVALID
} BH1749NUC_color_t;

typedef enum {
    BH1749NUC_INT_SOURCE_RED,
    BH1749NUC_INT_SOURCE_GREEN,
    BH1749NUC_INT_SOURCE_BLUE,
    BH1749NUC_INT_SOURCE_FORBIDDEN,
    BH1749NUC_INT_SOURCE_NEW,
    BH1749NUC_INT_SOURCE_INVALID
} BH1749NUC_int_source_t;

typedef enum {
    BH1749NUC_INT_NEW_DATA,
    BH1749NUC_INT_PERSISTENCE_1,
    BH1749NUC_INT_PERSISTENCE_4,
    BH1749NUC_INT_PERSISTENCE_8,
    BH1749NUC_INT_PERSISTENCE_INVALID
} BH1749NUC_int_persistence_t;

typedef enum {
    BH1749NUC_ERROR_PART_ID         = -5,
    BH1749NUC_ERROR_READ            = -4,
    BH1749NUC_ERROR_WRITE           = -3,
    BH1749NUC_ERROR_INVALID_ADDRESS = -2,
    BH1749NUC_ERROR_UNDEFINED       = -1,
    BH1749NUC_ERROR_SUCCESS         = 1
} BH1749NUC_error_t;
const BH1749NUC_error_t BH1749NUC_SUCCESS = BH1749NUC_ERROR_SUCCESS;

struct rgb_sense{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t ir;
    uint16_t green2;
};

class BH1749NUC {
public:
    rgb_sense colors;

    BH1749NUC(void);

    BH1749NUC_error_t begin(BH1749NUC_Address_t deviceAddress, TwoWire &wirePort = Wire);
    boolean begin();

    void setDebugStream(Stream &debugPort = Serial);

    BH1749NUC_error_t clearInterrupt(void);
    
    BH1749NUC_gain_t readIRGain(void);
    BH1749NUC_error_t setIRGain(BH1749NUC_gain_t gain);
    BH1749NUC_gain_t readRGBGain(void);
    BH1749NUC_error_t setRGBGain(BH1749NUC_gain_t gain);

    BH1749NUC_measurement_mode_t readMeasurementMode(void);
    BH1749NUC_error_t setMeasurementMode(BH1749NUC_measurement_mode_t mode);

    boolean update(void);
    boolean ready(void);
    boolean available(void);

    uint16_t red(void);
    uint16_t green(void);
    uint16_t blue(void);
    uint16_t ir(void);
    uint16_t green2(void);

    BH1749NUC_error_t read(rgb_sense * rgb);
    uint16_t read(BH1749NUC_color_t color);
    uint16_t readRed(void);
    uint16_t readGreen(void);
    uint16_t readGreen2(void);
    uint16_t readBlue(void);
    uint16_t readIR(void);

    boolean readInterrupt(void); // Read interrupt status
    BH1749NUC_int_source_t getInterruptSource(void);
    BH1749NUC_error_t setInterruptSource(BH1749NUC_int_source_t source);
    BH1749NUC_error_t setInterruptSource(BH1749NUC_color_t color, BH1749NUC_int_persistence_t persist = BH1749NUC_INT_PERSISTENCE_1);
    boolean getEnableInterrupt(void);
    BH1749NUC_error_t enableInterrupt(boolean enable = true);

    BH1749NUC_int_persistence_t getInterruptPersistence(void);
    BH1749NUC_error_t setInterruptPersistence(BH1749NUC_int_persistence_t persist);

    uint16_t getThresholdHigh(void);
    uint16_t getThresholdLow(void);
    BH1749NUC_error_t setThresholdHigh(uint16_t highThresh);
    BH1749NUC_error_t setThresholdLow(uint16_t lowThresh);
    BH1749NUC_error_t setThresholds(uint16_t lowThresh, uint16_t highThresh);

    void setInterruptPin(uint8_t pin);

private:

    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
    BH1749NUC_Address_t _deviceAddress;
    Stream * _debugPort;
    boolean _intEnabled;

// System Control
    typedef enum {
        SW_RESET_NOT_DONE,
        SW_RESET_DONE,
        SW_RESET_INVALID
    } sw_reset_status_t;
    sw_reset_status_t swResetStatus(void);
    typedef enum {
        INT_PIN_NOT_CHANGED,
        INT_PIN_INACTIVE,
        INT_PIN_INVALID
    } int_pin_status_t;
    int_pin_status_t intPinStatus(void);
    uint8_t readPartID(void);

// Mode Control 2
    typedef enum {
        BH1749NUC_MEASUREMENT_VALID_INVALID, // 0
        BH1749NUC_MEASUREMENT_VALID_VALID,   // 1
        BH1749NUC_MEASUREMENT_VALID_INVALID_INVALID
    } BH1749NUC_measurement_valid_t;
    BH1749NUC_measurement_valid_t readValid(void);

    typedef enum {
        BH1749NUC_MEASUREMENT_ACTIVE_ACTIVE, // 0
        BH1749NUC_MEASUREMENT_ACTIVE_INACTIVE, // 0
        BH1749NUC_MEASUREMENT_ACTIVE_INVALID
    } BH1749NUC_measurement_active_t;
    BH1749NUC_measurement_active_t readMeasurementActive(void);
    BH1749NUC_error_t setMeasurementActive(boolean active);

// I2C Read/Write
    BH1749NUC_error_t readI2CBuffer(uint8_t * dest, BH1749NUC_REGISTER_t startRegister, uint16_t len);
    BH1749NUC_error_t writeI2CBuffer(uint8_t * src, BH1749NUC_REGISTER_t startRegister, uint16_t len);
    BH1749NUC_error_t readI2CRegister(uint8_t * dest, BH1749NUC_REGISTER_t registerAddress);
    BH1749NUC_error_t writeI2CRegister(uint8_t data, BH1749NUC_REGISTER_t registerAddress);
};
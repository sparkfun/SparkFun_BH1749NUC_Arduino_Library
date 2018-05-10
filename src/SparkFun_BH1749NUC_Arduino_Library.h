/*
  This is a library written for the BH1749NUC Color Sensor
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14733
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


// Valid BH1749NUC I2C addresses:
typedef enum {
    BH1749NUC_ADDRESS_CLOSED = 0x38,
    BH1749NUC_ADDRESS_OPEN = 0x39,
    BH1749NUC_ADDRESS_INVALID = 0xFF
} BH1749NUC_Address_t;
// /Default I2C address:
const BH1749NUC_Address_t BH1749NUC_ADDRESS_DEFAULT = BH1749NUC_ADDRESS_OPEN;

// BH1749NUC registers:
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

// RGB and IR gain settings:
typedef enum {
    BH1749NUC_GAIN_FORBIDDEN_0, // 0
    BH1749NUC_GAIN_X1,          // 1
    BH1749NUC_GAIN_FORBIDDEN_2, // 2
    BH1749NUC_GAIN_X32,         // 3,
    BH1749NUC_GAIN_INVALID
} BH1749NUC_gain_t;

// Measurement mode settings (update rate):
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

// BH1749NUC interrupt sources:
typedef enum {
    BH1749NUC_INT_SOURCE_RED,
    BH1749NUC_INT_SOURCE_GREEN,
    BH1749NUC_INT_SOURCE_BLUE,
    BH1749NUC_INT_SOURCE_FORBIDDEN,
    BH1749NUC_INT_SOURCE_NEW,
    BH1749NUC_INT_SOURCE_INVALID
} BH1749NUC_int_source_t;

// BH1749NUC interrupt persistence:
typedef enum {
    BH1749NUC_INT_NEW_DATA,
    BH1749NUC_INT_PERSISTENCE_1,
    BH1749NUC_INT_PERSISTENCE_4,
    BH1749NUC_INT_PERSISTENCE_8,
    BH1749NUC_INT_PERSISTENCE_INVALID
} BH1749NUC_int_persistence_t;

typedef enum {
    BH1749NUC_MEASUREMENT_ACTIVE_ACTIVE, // 0
    BH1749NUC_MEASUREMENT_ACTIVE_INACTIVE, // 0
    BH1749NUC_MEASUREMENT_ACTIVE_INVALID
} BH1749NUC_measurement_active_t;

// BH1749NUC error codes:
typedef enum {
    BH1749NUC_ERROR_PART_ID         = -5,
    BH1749NUC_ERROR_READ            = -4,
    BH1749NUC_ERROR_WRITE           = -3,
    BH1749NUC_ERROR_INVALID_ADDRESS = -2,
    BH1749NUC_ERROR_UNDEFINED       = -1,
    BH1749NUC_ERROR_SUCCESS         = 1
} BH1749NUC_error_t;
const BH1749NUC_error_t BH1749NUC_SUCCESS = BH1749NUC_ERROR_SUCCESS;

// Container for rgb and IR values:
struct rgb_sense{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t ir;
    uint16_t green2;
};

// Red, green, blue, ir, and green2 register:
typedef enum {
    BH1749NUC_RED,
    BH1749NUC_GREEN,
    BH1749NUC_BLUE,
    BH1749NUC_IR,
    BH1749NUC_GREEN2,
    BH1749NUC_INVALID
} BH1749NUC_color_t;

class BH1749NUC {
public:
    // Publically accessable, updated RGB/IR values:
    rgb_sense colors;

    BH1749NUC(void); // Constructor

    // begin can either use the default values (default I2C address and Wire I2C port),
    // and return a boolean on success/fail.
    // or it can take specific address/TwoWire port values and return an error code
    boolean begin();
    BH1749NUC_error_t begin(BH1749NUC_Address_t deviceAddress, TwoWire &wirePort = Wire);

    // To enable library debugging, call setDebugStream. Use the default Serial object
    // or another stream to print data to (assumes you've already called e.g. Serial.begin).
    void setDebugStream(Stream &debugPort = Serial);

    // Clear interrupt state
    BH1749NUC_error_t clearInterrupt(void);
    
    // Read and set IR and RGB gain to one of BH1749NUC_gain_t's acceptable values.
    // (BH1749NUC_GAIN_X1 or BH1749NUC_GAIN_X32 -- returns error otherwise)
    BH1749NUC_gain_t readIRGain(void);
    BH1749NUC_error_t setIRGain(BH1749NUC_gain_t gain);
    BH1749NUC_gain_t readRGBGain(void);
    BH1749NUC_error_t setRGBGain(BH1749NUC_gain_t gain);

    // Read and set the measurement mode (update rate) to one of the three acceptable
    // values (BH1749NUC_MEASUREMENT_MODE_35_MS, BH1749NUC_MEASUREMENT_MODE_120_MS or
    // BH1749NUC_MEASUREMENT_MODE_240_MS -- returns error otherwise).
    BH1749NUC_measurement_mode_t readMeasurementMode(void);
    BH1749NUC_error_t setMeasurementMode(BH1749NUC_measurement_mode_t mode);

    // if new data is available from the BH1749NUC, read in new data.
    // data is stored in the "colors" struct.
    boolean update(void);

    // available returns true if new data is available from the RGB sensor.
    // This will automaticlly perform an update.
    boolean available(void);
    // ready does the same thing as available() -- just a different way to say it
    boolean ready(void);

    // red, green, blue, ir, and green2 functions return the latest RGB/IR/G2 value
    // that the sensor sees. If new data is available it is automatically updated.
    uint16_t red(void);
    uint16_t green(void);
    uint16_t blue(void);
    uint16_t ir(void);
    uint16_t green2(void);

    // read(rgb_sense*) can be used to read the latest color values into a
    // rgb_sense object.
    BH1749NUC_error_t read(rgb_sense * rgb);

    // read(BH1749NUC_color_t) can be used to read a specific color value.
    // either BH1749NUC_RED, BH1749NUC_GREEN, BH1749NUC_BLUE, BH1749NUC_IR or
    // BH1749NUC_GREEN2. It returns an unsigned 16-bit value (no error indication)
    uint16_t read(BH1749NUC_color_t color);
    
    // readRed, readGreen, readBlue, readIR, and readGreen2 read the most up-to-date
    // color value of question and return an unsigned 16-bit value (no error indication)
    uint16_t readRed(void);
    uint16_t readGreen(void);
    uint16_t readGreen2(void);
    uint16_t readBlue(void);
    uint16_t readIR(void);

    // readInterrupt reads the current interrupt status -- 
    // also clears the interrupt bit if its set
    boolean readInterrupt(void);

    // getInterruptSource and setInterruptSource can be used to read/set the interrupt source.
    // The interrupt source can be BH1749NUC_INT_SOURCE_NEW, BH1749NUC_INT_SOURCE_RED, 
    // BH1749NUC_INT_SOURCE_GREEN or BH1749NUC_INT_SOURCE_BLUE.
    // Additionally -- if the interrupt source is red, green, or blue --  a persistence value
    // can be set to only trigger a new interrupt on 1, 4, or 8 consecutive out-of-threshold readings
    // (BH1749NUC_INT_PERSISTENCE_1, BH1749NUC_INT_PERSISTENCE_4, or BH1749NUC_INT_PERSISTENCE_8)
    BH1749NUC_int_source_t getInterruptSource(void);
    BH1749NUC_error_t setInterruptSource(BH1749NUC_int_source_t source);
    BH1749NUC_error_t setInterruptSource(BH1749NUC_color_t color, BH1749NUC_int_persistence_t persist = BH1749NUC_INT_PERSISTENCE_1);

    // enableInterrupt should be called to enable the interrupt output
    boolean getEnableInterrupt(void);
    BH1749NUC_error_t enableInterrupt(boolean enable = true);

    // setInterruptPersistence can be called to set interrupt persistence if the source is set
    // to red, green, or blue out-of-threshold.
    // (BH1749NUC_INT_PERSISTENCE_1, BH1749NUC_INT_PERSISTENCE_4, or BH1749NUC_INT_PERSISTENCE_8)
    BH1749NUC_int_persistence_t getInterruptPersistence(void);
    BH1749NUC_error_t setInterruptPersistence(BH1749NUC_int_persistence_t persist);

    // setThresholdHigh, setThresholdLow, and setThresholds can be used to set the high/low
    // threshold for a rgb reading. If the current reading is greater than the high or less
    // than the low threshold, an interrupt will be triggered
    uint16_t getThresholdHigh(void);
    uint16_t getThresholdLow(void);
    BH1749NUC_error_t setThresholdHigh(uint16_t highThresh);
    BH1749NUC_error_t setThresholdLow(uint16_t lowThresh);
    BH1749NUC_error_t setThresholds(uint16_t lowThresh, uint16_t highThresh);

    // setMeasurementActive can be used to enable disable active measurements
    BH1749NUC_measurement_active_t readMeasurementActive(void);
    BH1749NUC_error_t setMeasurementActive(boolean active);

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

// I2C Read/Write
    BH1749NUC_error_t readI2CBuffer(uint8_t * dest, BH1749NUC_REGISTER_t startRegister, uint16_t len);
    BH1749NUC_error_t writeI2CBuffer(uint8_t * src, BH1749NUC_REGISTER_t startRegister, uint16_t len);
    BH1749NUC_error_t readI2CRegister(uint8_t * dest, BH1749NUC_REGISTER_t registerAddress);
    BH1749NUC_error_t writeI2CRegister(uint8_t data, BH1749NUC_REGISTER_t registerAddress);
};
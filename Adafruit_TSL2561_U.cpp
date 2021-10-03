/*!
 * @file Adafruit_TSL2561_U.cpp
 *
 * @mainpage Adafruit TSL2561 Light/Lux sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's TSL2561 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit TSL2561 breakout: http://www.adafruit.com/products/439
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit_Sensor"> Adafruit_Sensor</a> being
 * present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 *   @section  HISTORY
 *
 *   v2.0 - Rewrote driver for Adafruit_Sensor and Auto-Gain support, and
 *          added lux clipping check (returns 0 lux on sensor saturation)
 *   v1.0 - First release (previously TSL2561)
 */
/**************************************************************************/

#include "Adafruit_TSL2561_U.h"

#include <espchrono.h>
#include <tickchrono.h>

//#define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT
#include <esp_log.h>
#define DBGPRNT(format, ...) ESP_LOGW("TSL2561", format, ##__VA_ARGS__)
#else
#define DBGPRNT(format, ...)
#endif

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief Initializes I2C and configures the sensor with default Wire I2C
           (call this function before doing anything else)
    @returns True if sensor is found and initialized, false otherwise.
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::begin(bool skipWireBegin) {
  _i2c = &Wire;

  if (!skipWireBegin)
    if (!_i2c->begin())
    { DBGPRNT("fail"); return false; }

  if (!init()) { DBGPRNT("fail"); return false; }

  return true;
}

/**************************************************************************/
/*!
    @brief Initializes I2C and configures the sensor with provided I2C device
           (call this function before doing anything else)
    @param theWire A pointer to any I2C interface (e.g. &Wire1)
    @returns True if sensor is found and initialized, false otherwise.
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::begin(TwoWire *theWire, bool skipWireBegin) {
  _i2c = theWire;

  if (!skipWireBegin)
    if (!_i2c->begin())
    { DBGPRNT("fail"); return false; }

  if (!init()) { DBGPRNT("fail"); return false; }

  return true;
}

/**************************************************************************/
/*!
    @brief  Initializes I2C connection and settings.
    Attempts to determine if the sensor is contactable, then sets up a default
    integration time and gain. Then powers down the chip.
    @returns True if sensor is found and initialized, false otherwise.
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::init() {
  if (!_i2c) { DBGPRNT("fail"); return false; }

  /* Make sure we're actually connected */
  if (std::optional<uint8_t> x = read8(TSL2561_REGISTER_ID)) {
    if (*x & 0x05) { // ID code for TSL2561
      DBGPRNT("fail"); return false;
    }
  } else {
    DBGPRNT("fail"); return false;
  }

  _tsl2561Initialised = true;

  bool succ{true};

  /* Set default integration time and gain */
  if (!setIntegrationTimePriv(_tsl2561IntegrationTime)) { DBGPRNT("fail"); succ = false; }
  if (!setGainPriv(_tsl2561Gain)) { DBGPRNT("fail"); succ = false; }

  /* Note: by default, the device is in power down mode on bootup */
  if (!disable()) { DBGPRNT("fail"); succ = false; }

  return succ;
}

/**************************************************************************/
/*!
    @brief  Enables or disables the auto-gain settings when reading
            data from the sensor
    @param enable Set to true to enable, False to disable
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::enableAutoRange(bool enable) {
  _tsl2561AutoGain = enable;
}

/**************************************************************************/
/*!
    @brief      Sets the integration time for the TSL2561. Higher time means
                more light captured (better for low light conditions) but will
                take longer to run readings.
    @param time The amount of time we'd like to add up values
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::setIntegrationTime(tsl2561IntegrationTime_t time) {
  if (!_tsl2561Initialised)
    if (!init())
    { DBGPRNT("fail"); return false; }

  return setIntegrationTimePriv(time);
}


/**************************************************************************/
/*!
    @brief      Sets the integration time for the TSL2561. Higher time means
                more light captured (better for low light conditions) but will
                take longer to run readings.
    @param time The amount of time we'd like to add up values
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::setIntegrationTimePriv(tsl2561IntegrationTime_t time) {
  /* Enable the device by setting the control bit to 0x03 */
  if (!enable())
  { DBGPRNT("fail"); return false; }

  bool succ{true};

  /* Update the timing register */
  if (!write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, time | _tsl2561Gain))
  { DBGPRNT("fail"); succ = false; }

  /* Update value placeholders */
  _tsl2561IntegrationTime = time;

  /* Turn the device off to save power */
  if (!disable()) { DBGPRNT("fail"); succ = false; }

  return succ;
}

/**************************************************************************/
/*!
    @brief  Adjusts the gain on the TSL2561 (adjusts the sensitivity to light)
    @param gain The value we'd like to set the gain to
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::setGain(tsl2561Gain_t gain) {
  if (!_tsl2561Initialised)
    if (!init())
    { DBGPRNT("fail"); return false; }

  return setGainPriv(gain);
}


/**************************************************************************/
/*!
    @brief  Adjusts the gain on the TSL2561 (adjusts the sensitivity to light)
    @param gain The value we'd like to set the gain to
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::setGainPriv(tsl2561Gain_t gain) {
  /* Enable the device by setting the control bit to 0x03 */
  if (!enable())
  { DBGPRNT("fail"); return false; }

  bool succ{true};

  /* Update the timing register */
  if (!write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,
              _tsl2561IntegrationTime | gain))
  { DBGPRNT("fail"); succ = false; }

  /* Update value placeholders */
  _tsl2561Gain = gain;

  /* Turn the device off to save power */
  if (!disable()) { DBGPRNT("fail"); succ = false; }

  return succ;
}

/**************************************************************************/
/*!
    @brief  Gets the broadband (mixed lighting) and IR only values from
            the TSL2561, adjusting gain if auto-gain is enabled
    @param  broadband Pointer to a uint16_t we will fill with a sensor
                      reading from the IR+visible light diode.
    @param  ir Pointer to a uint16_t we will fill with a sensor the
               IR-only light diode.
*/
/**************************************************************************/
std::optional<Adafruit_TSL2561_Unified::Luminosity> Adafruit_TSL2561_Unified::getLuminosity() {
  if (!_tsl2561Initialised)
    if (!init())
    { DBGPRNT("fail"); return std::nullopt; }

  /* If Auto gain disabled get a single reading and continue */
  if (!_tsl2561AutoGain) {
    if (const auto result = getData())
      return *result;
    else
    { DBGPRNT("fail"); return std::nullopt; }
  }

  /* Read data until we find a valid range */
  bool _agcCheck = false;
  while (true) {
    Luminosity luminosity;
    uint16_t _hi, _lo;
    tsl2561IntegrationTime_t _it = _tsl2561IntegrationTime;

    /* Get the hi/low threshold for the current integration time */
    switch (_it) {
    case TSL2561_INTEGRATIONTIME_13MS:
      _hi = TSL2561_AGC_THI_13MS;
      _lo = TSL2561_AGC_TLO_13MS;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      _hi = TSL2561_AGC_THI_101MS;
      _lo = TSL2561_AGC_TLO_101MS;
      break;
    default:
      _hi = TSL2561_AGC_THI_402MS;
      _lo = TSL2561_AGC_TLO_402MS;
      break;
    }

    if (const auto result = getData())
      luminosity = *result;
    else
    { DBGPRNT("fail"); return std::nullopt; }

    /* Run an auto-gain check if we haven't already done so ... */
    if (!_agcCheck) {
      if ((luminosity.broadband < _lo) && (_tsl2561Gain == TSL2561_GAIN_1X)) {
        /* Increase the gain and try again */
        if (!setGain(TSL2561_GAIN_16X))
        { DBGPRNT("fail"); return std::nullopt; }

        /* Drop the previous conversion results */
        if (const auto result = getData())
          luminosity = *result;
        else
        { DBGPRNT("fail"); return std::nullopt; }

        /* Set a flag to indicate we've adjusted the gain */
        _agcCheck = true;
      } else if ((luminosity.broadband > _hi) && (_tsl2561Gain == TSL2561_GAIN_16X)) {
        /* Drop gain to 1x and try again */
        if (!setGain(TSL2561_GAIN_1X))
        { DBGPRNT("fail"); return std::nullopt; }

        /* Drop the previous conversion results */
        if (const auto result = getData())
          luminosity = *result;
        else
        { DBGPRNT("fail"); return std::nullopt; }

        /* Set a flag to indicate we've adjusted the gain */
        _agcCheck = true;
      } else {
        /* Nothing to look at here, keep moving ....
           Reading is either valid, or we're already at the chips limits */
        return luminosity;
      }
    } else {
      /* If we've already adjusted the gain once, just return the new results.
         This avoids endless loops where a value is at one extreme pre-gain,
         and the the other extreme post-gain */
      return luminosity;
    }
  }
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::enable(void) {
  /* Enable the device by setting the control bit to 0x03 */
  if (!write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
              TSL2561_CONTROL_POWERON))
  { DBGPRNT("fail"); return false; }
  else
    return true;
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::disable(void) {
  /* Turn the device off to save power */
  if (!write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
              TSL2561_CONTROL_POWEROFF))
  { DBGPRNT("fail"); return false; }
  else
    return true;
}

/**************************************************************************/
/*!
    Private function to read luminosity on both channels
*/
/**************************************************************************/
std::optional<Adafruit_TSL2561_Unified::Luminosity> Adafruit_TSL2561_Unified::getData() {
  /* Enable the device by setting the control bit to 0x03 */
  if (!enable())
  { DBGPRNT("fail"); return std::nullopt; }

  /* Wait x ms for ADC to complete */
  switch (_tsl2561IntegrationTime) {
  case TSL2561_INTEGRATIONTIME_13MS:
    espcpputils::delay(TSL2561_DELAY_INTTIME_13MS); // KTOWN: Was 14ms
    break;
  case TSL2561_INTEGRATIONTIME_101MS:
    espcpputils::delay(TSL2561_DELAY_INTTIME_101MS); // KTOWN: Was 102ms
    break;
  default:
    espcpputils::delay(TSL2561_DELAY_INTTIME_402MS); // KTOWN: Was 403ms
    break;
  }

  bool succ{true};
  Luminosity luminosity;

  /* Reads a two byte value from channel 0 (visible + infrared) */
  if (const auto broadband = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW))
    luminosity.broadband = *broadband;
  else
  { DBGPRNT("fail"); succ = false; }

  /* Reads a two byte value from channel 1 (infrared) */
  if (const auto ir = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW))
    luminosity.ir = *ir;
  else
  { DBGPRNT("fail"); succ = false; }

  /* Turn the device off to save power */
  if (!disable()) { DBGPRNT("fail"); succ = false; }

  if (succ)
    return luminosity;
  else
    return std::nullopt;
}

/**************************************************************************/
/*!
    @brief  Converts the raw sensor values to the standard SI lux equivalent.
    @param  broadband The 16-bit sensor reading from the IR+visible light diode.
    @param  ir The 16-bit sensor reading from the IR-only light diode.
    @returns The integer Lux value we calcuated.
             Returns 0 if the sensor is saturated and the values are
             unreliable, or 65536 if the sensor is saturated.
*/
/**************************************************************************/
/**************************************************************************/
/*!

    Returns
*/
/**************************************************************************/
std::optional<uint32_t> Adafruit_TSL2561_Unified::calculateLux(Luminosity luminosity) {
  const uint16_t broadband = luminosity.broadband;
  const uint16_t ir = luminosity.ir;

  /* Make sure the sensor isn't saturated! */
  uint16_t clipThreshold;
  switch (_tsl2561IntegrationTime) {
  case TSL2561_INTEGRATIONTIME_13MS:
    clipThreshold = TSL2561_CLIPPING_13MS;
    break;
  case TSL2561_INTEGRATIONTIME_101MS:
    clipThreshold = TSL2561_CLIPPING_101MS;
    break;
  default:
    clipThreshold = TSL2561_CLIPPING_402MS;
    break;
  }

  /* Return 65536 lux if the sensor is saturated */
  if ((broadband > clipThreshold) || (ir > clipThreshold)) {
    DBGPRNT("saturated");
    return std::nullopt;
  }

  /* Get the correct scale depending on the intergration time */
  unsigned long chScale;
  switch (_tsl2561IntegrationTime) {
  case TSL2561_INTEGRATIONTIME_13MS:
    chScale = TSL2561_LUX_CHSCALE_TINT0;
    break;
  case TSL2561_INTEGRATIONTIME_101MS:
    chScale = TSL2561_LUX_CHSCALE_TINT1;
    break;
  default: /* No scaling ... integration time = 402ms */
    chScale = (1 << TSL2561_LUX_CHSCALE);
    break;
  }

  /* Scale for gain (1x or 16x) */
  if (!_tsl2561Gain)
    chScale = chScale << 4;

  /* Scale the channel values */
  unsigned long channel0 = (broadband * chScale) >> TSL2561_LUX_CHSCALE;
  unsigned long channel1 = (ir * chScale) >> TSL2561_LUX_CHSCALE;

  /* Find the ratio of the channel values (Channel1/Channel0) */
  unsigned long ratio1 = 0;
  if (channel0 != 0)
    ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE + 1)) / channel0;

  /* round the ratio value */
  unsigned long ratio = (ratio1 + 1) >> 1;

  unsigned int b{}, m{};

#ifdef TSL2561_PACKAGE_CS
  if ((ratio <= TSL2561_LUX_K1C)) {
    b = TSL2561_LUX_B1C;
    m = TSL2561_LUX_M1C;
  } else if (ratio <= TSL2561_LUX_K2C) {
    b = TSL2561_LUX_B2C;
    m = TSL2561_LUX_M2C;
  } else if (ratio <= TSL2561_LUX_K3C) {
    b = TSL2561_LUX_B3C;
    m = TSL2561_LUX_M3C;
  } else if (ratio <= TSL2561_LUX_K4C) {
    b = TSL2561_LUX_B4C;
    m = TSL2561_LUX_M4C;
  } else if (ratio <= TSL2561_LUX_K5C) {
    b = TSL2561_LUX_B5C;
    m = TSL2561_LUX_M5C;
  } else if (ratio <= TSL2561_LUX_K6C) {
    b = TSL2561_LUX_B6C;
    m = TSL2561_LUX_M6C;
  } else if (ratio <= TSL2561_LUX_K7C) {
    b = TSL2561_LUX_B7C;
    m = TSL2561_LUX_M7C;
  } else if (ratio > TSL2561_LUX_K8C) {
    b = TSL2561_LUX_B8C;
    m = TSL2561_LUX_M8C;
  }
#else
  if ((ratio <= TSL2561_LUX_K1T)) {
    b = TSL2561_LUX_B1T;
    m = TSL2561_LUX_M1T;
  } else if (ratio <= TSL2561_LUX_K2T) {
    b = TSL2561_LUX_B2T;
    m = TSL2561_LUX_M2T;
  } else if (ratio <= TSL2561_LUX_K3T) {
    b = TSL2561_LUX_B3T;
    m = TSL2561_LUX_M3T;
  } else if (ratio <= TSL2561_LUX_K4T) {
    b = TSL2561_LUX_B4T;
    m = TSL2561_LUX_M4T;
  } else if (ratio <= TSL2561_LUX_K5T) {
    b = TSL2561_LUX_B5T;
    m = TSL2561_LUX_M5T;
  } else if (ratio <= TSL2561_LUX_K6T) {
    b = TSL2561_LUX_B6T;
    m = TSL2561_LUX_M6T;
  } else if (ratio <= TSL2561_LUX_K7T) {
    b = TSL2561_LUX_B7T;
    m = TSL2561_LUX_M7T;
  } else if (ratio > TSL2561_LUX_K8T) {
    b = TSL2561_LUX_B8T;
    m = TSL2561_LUX_M8T;
  }
#endif

  channel0 = channel0 * b;
  channel1 = channel1 * m;

  unsigned long temp{};
  /* Do not allow negative lux value */
  if (channel0 > channel1)
    temp = channel0 - channel1;
  else {
    DBGPRNT("negative lux");
    return std::nullopt;
  }

  /* Round lsb (2^(LUX_SCALE-1)) */
  temp += (1 << (TSL2561_LUX_LUXSCALE - 1));

  /* Strip off fractional portion */
  uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

  /* Signal I2C had no errors */
  return lux;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
    @param  event Pointer to a sensor_event_t type that will be filled
                  with the lux value, timestamp, data type and sensor ID.
    @returns True if sensor reading is between 0 and 65535 lux,
             false if sensor is saturated
*/
/**************************************************************************/
std::optional<sensors_event_t> Adafruit_TSL2561_Unified::getEvent() {
  auto luminosity = getLuminosity();
  if (!luminosity) { DBGPRNT("fail"); return std::nullopt; }

  /* Calculate the actual lux value */
  auto lux = calculateLux(*luminosity);
  if (!lux) { DBGPRNT("fail"); return std::nullopt; }

  sensors_event_t event;
  event.version = sizeof(sensors_event_t);
  event.sensor_id = _tsl2561SensorID;
  event.type = SENSOR_TYPE_LIGHT;
  event.reserved0 = 0;
  event.timestamp = espchrono::millis_clock::now();
  event.light = *lux;
  return event;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
    @param  sensor A pointer to a sensor_t structure that we will fill with
                   details about the TSL2561 and its capabilities
*/
/**************************************************************************/
sensor_t Adafruit_TSL2561_Unified::getSensor() {
  sensor_t sensor;
  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor.name, "TSL2561", sizeof(sensor.name) - 1);
  sensor.name[sizeof(sensor.name) - 1] = 0;

  sensor.version = 1;
  sensor.sensor_id = _tsl2561SensorID;
  sensor.type = SENSOR_TYPE_LIGHT;
  sensor.max_value = 17000.0; /* Based on trial and error ... confirm! */
  sensor.min_value = 1.0;
  sensor.resolution = 1.0;
  sensor.min_delay = 0;
  return sensor;
}

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
    @param  reg I2C register to write the value to
    @param  value The 8-bit value we're writing to the register
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::write8(uint8_t reg, uint8_t value) {
  if (!_i2c) { DBGPRNT("fail"); return false; }

  bool succ{true};

  _i2c->beginTransmission(_addr);
  if (_i2c->write(reg) != 1) { DBGPRNT("fail"); succ = false; }
  if (_i2c->write(value) != 1) { DBGPRNT("fail"); succ = false; }
  if (const auto result = _i2c->endTransmission(); result != I2C_ERROR_OK) { DBGPRNT("fail %hhu", result); succ = false; }

  if (!succ)
    _tsl2561Initialised = false;

  return succ;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
    @param  reg I2C register to read from
    @returns 8-bit value containing single byte data read
*/
/**************************************************************************/
std::optional<uint8_t> Adafruit_TSL2561_Unified::read8(uint8_t reg) {
  if (!_i2c) { DBGPRNT("fail"); return false; }

  bool succ{true};

  _i2c->beginTransmission(_addr);
  if (_i2c->write(reg) != 1) { DBGPRNT("fail"); succ = false; }
  if (const auto result = _i2c->endTransmission(); result != I2C_ERROR_OK) { DBGPRNT("fail %hhu", result); succ = false; }

  if (const auto result = _i2c->requestFrom(_addr, 1); result != 1) { DBGPRNT("fail %hhu", result); succ = false; }
  const auto result = _i2c->read();
  if (result == -1) { DBGPRNT("fail %i", result); succ = false; }
  _i2c->endTransmission();

  if (!succ)
    _tsl2561Initialised = false;

  if (succ)
    return result;
  else
    return std::nullopt;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
    @param  reg I2C register to read from
    @returns 16-bit value containing 2-byte data read
*/
/**************************************************************************/
std::optional<uint16_t> Adafruit_TSL2561_Unified::read16(uint8_t reg) {
  if (!_i2c) { DBGPRNT("fail"); return false; }

  bool succ{true};

  _i2c->beginTransmission(_addr);
  if (_i2c->write(reg) != 1) { DBGPRNT("fail"); succ = false; }
  if (const auto result = _i2c->endTransmission(); result != I2C_ERROR_OK) { DBGPRNT("fail %hhu", result); succ = false; }

  if (const auto result = _i2c->requestFrom(_addr, 2); result != 2) { DBGPRNT("fail %hhu", result); succ = false; }
  const auto result0 = _i2c->read();
  const auto result1 = _i2c->read();
  Wire.endTransmission();

  if (result0 == -1) { DBGPRNT("fail %i", result0); succ = false; }
  else if (result1 == -1) { DBGPRNT("fail %i", result1); succ = false; }

  if (!succ)
      _tsl2561Initialised = false;

  if (succ) {
    uint16_t t = result0, x = result1;
    x <<= 8;
    x |= t;
    return x;
  }
  else
    return std::nullopt;
}

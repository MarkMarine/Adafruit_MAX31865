/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "MAX31865.h"
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstdio>
#include <string>

#include "wiringPi.h"
#include "wiringPiSPI.h"

static int fd;
static int channel;
static int speed;

MAX31865::MAX31865(int _channel, int _speed) {
    wiringPiSetup();
    fd = wiringPiSPISetup(_channel, _speed);
    if (fd < 0) {
        throw std::string("Can't open the SPI bus");
    }
    channel = _channel;
    speed = _speed;
}

bool MAX31865::begin(max31865_numwires_t wires) {
    setWires(wires);
    enableBias(false);
    autoConvert(false);
    clearFault();
    return true;
}

uint8_t MAX31865::readFault() {
    return readRegister8(MAX31856_FAULTSTAT_REG);
}

void MAX31865::clearFault() {
    uint8_t t = readRegister8(MAX31856_CONFIG_REG);
    t &= ~0x2C;
    t |= MAX31856_CONFIG_FAULTSTAT;
    writeRegister8(MAX31856_CONFIG_REG, t);
}

void MAX31865::setWires(max31865_numwires_t wires) {
    uint8_t t = readRegister8(MAX31856_CONFIG_REG);
    if (wires == MAX31865_3WIRE) {
        t |= MAX31856_CONFIG_3WIRE;
    } else {
        // 2 or 4 wire
        t &= ~MAX31856_CONFIG_3WIRE;
    }
    writeRegister8(MAX31856_CONFIG_REG, t);
}

void MAX31865::autoConvert(bool b) {
    uint8_t t = readRegister8(MAX31856_CONFIG_REG);
    if (b) {
        t |= MAX31856_CONFIG_MODEAUTO;       // enable autoconvert
    } else {
        t &= ~MAX31856_CONFIG_MODEAUTO;       // disable autoconvert
    }
    writeRegister8(MAX31856_CONFIG_REG, t);
}

void MAX31865::enableBias(bool b) {
    uint8_t t = readRegister8(MAX31856_CONFIG_REG);
    if (b) {
        t |= MAX31856_CONFIG_BIAS;       // enable bias
    } else {
        t &= ~MAX31856_CONFIG_BIAS;       // disable bias
    }
    writeRegister8(MAX31856_CONFIG_REG, t);
}

float MAX31865::temperature(float RTDnominal, float refResistor) {
    // http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf

    float Z1, Z2, Z3, Z4, Rt, temp;

    Rt = readRTD();
    Rt /= 32768;
    Rt *= refResistor;

    // Serial.print("\nResistance: "); Serial.println(Rt, 8);

    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) / RTDnominal;
    Z4 = 2 * RTD_B;

    temp = Z2 + (Z3 * Rt);
    temp = (sqrt(temp) + Z1) / Z4;

    if (temp >= 0) return temp;

    // ugh.
    Rt /= RTDnominal;
    Rt *= 100;      // normalize to 100 ohm

    float rpoly = Rt;

    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= Rt;  // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rt;  // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rt;  // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rt;  // ^5
    temp += 1.5243e-10 * rpoly;

    return temp;
}

uint16_t MAX31865::readRTD() {
    clearFault();
    enableBias(true);
    delay(10);
    uint8_t t = readRegister8(MAX31856_CONFIG_REG);
    t |= MAX31856_CONFIG_1SHOT;
    writeRegister8(MAX31856_CONFIG_REG, t);
    delay(65);

    uint16_t rtd = readRegister16(MAX31856_RTDMSB_REG);

    // remove fault
    rtd >>= 1;

    return rtd;
}

void MAX31865::readRegisterN(uint8_t addr, uint8_t *buffer, uint8_t n) {
    addr &= 0x7F; // make sure top bit is not set
    // write addr to buffer first 8 bytes
    memcpy(buffer, &addr, sizeof addr);
    wiringPiSPIDataRW(channel, buffer, n);
}

uint8_t MAX31865::readRegister8(uint8_t addr) {
    uint8_t ret = 0;
    readRegisterN(addr, &ret, 1);

    return ret;
}

uint16_t MAX31865::readRegister16(uint8_t addr) {
    uint8_t buffer[2] = {0, 0};
    readRegisterN(addr, buffer, 2);

    uint16_t ret = buffer[0];
    ret <<= 8;
    ret |=  buffer[1];

    return ret;
}

void MAX31865::writeRegister8(uint8_t addr, uint8_t reg) {
    addr |= 0x80; // make sure the top bit is set
    wiringPiSPIDataRW(channel, &addr, 1);
    wiringPiSPIDataRW(channel, &reg, 1);
}

uint8_t MAX31865::spixfer(uint8_t addr) {
    wiringPiSPIDataRW(channel, &addr, 1);
    return addr;
}

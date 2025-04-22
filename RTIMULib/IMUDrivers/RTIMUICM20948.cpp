////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  The ICM-20948 and SPI driver code is based on code generously supplied by
//  swkim01@github.com

#include "RTIMUICM20948.h"
#include "RTIMUSettings.h"

RTIMUICM20948::RTIMUICM20948(RTIMUSettings *settings) : RTIMU(settings)
{

}

RTIMUICM20948::~RTIMUICM20948()
{
}

bool RTIMUICM20948::setSampleRate(int rate)
{
    if ((rate < ICM20948_SAMPLERATE_MIN) || (rate > ICM20948_SAMPLERATE_MAX)) {
        HAL_ERROR1("Illegal sample rate %d\n", rate);
        return false;
    }

    //  Note: rates interact with the lpf settings

    if ((rate < ICM20948_SAMPLERATE_MAX) && (rate >= 9000))
        rate = 9000;

    if ((rate < 9000) && (rate >= 1125))
        rate = 1125;

    if (rate < 1125) {
        m_sampleDiv = int((1125 / rate) - 1);
        m_sampleRate = 1125 / (1 + m_sampleDiv);
    } else {
        m_sampleDiv = 0;
        m_sampleRate = rate;
    }
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    return true;
}

bool RTIMUICM20948::setGyroLpf(unsigned char lpf)
{
    switch (lpf) {
    case ICM20948_GYRO_LPF_196:
    case ICM20948_GYRO_LPF_151:
    case ICM20948_GYRO_LPF_119:
    case ICM20948_GYRO_LPF_51:
    case ICM20948_GYRO_LPF_23:
    case ICM20948_GYRO_LPF_11:
    case ICM20948_GYRO_LPF_5:
    case ICM20948_GYRO_LPF_361:
        m_gyroLpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 gyro lpf %d\n", lpf);
        return false;
    }
}

bool RTIMUICM20948::setAccelLpf(unsigned char lpf)
{
    switch (lpf) {
    case ICM20948_ACCEL_LPF_1130:
    case ICM20948_ACCEL_LPF_460:
    case ICM20948_ACCEL_LPF_184:
    case ICM20948_ACCEL_LPF_92:
    case ICM20948_ACCEL_LPF_41:
    case ICM20948_ACCEL_LPF_20:
    case ICM20948_ACCEL_LPF_10:
    case ICM20948_ACCEL_LPF_5:
        m_accelLpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 accel lpf %d\n", lpf);
        return false;
    }
}


bool RTIMUICM20948::setCompassRate(int rate)
{
    if ((rate < ICM20948_COMPASSRATE_MIN) || (rate > ICM20948_COMPASSRATE_MAX)) {
        HAL_ERROR1("Illegal compass rate %d\n", rate);
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUICM20948::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case ICM20948_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case ICM20948_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (65.5 * 180.0);
        return true;

    case ICM20948_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case ICM20948_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 gyro fsr %d\n", fsr);
        return false;
    }
}

bool RTIMUICM20948::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case ICM20948_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case ICM20948_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case ICM20948_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case ICM20948_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 accel fsr %d\n", fsr);
        return false;
    }
}


bool RTIMUICM20948::IMUInit()
{
    unsigned char result;

    m_firstTime = true;

#ifdef ICM20948_CACHE_MODE
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif

    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_slaveAddr = m_settings->m_I2CSlaveAddress;

    setSampleRate(m_settings->m_ICM20948GyroAccelSampleRate);
    setCompassRate(m_settings->m_ICM20948CompassSampleRate);
    setGyroLpf(m_settings->m_ICM20948GyroLpf);
    setAccelLpf(m_settings->m_ICM20948AccelLpf);
    setGyroFsr(m_settings->m_ICM20948GyroFsr);
    setAccelFsr(m_settings->m_ICM20948AccelFsr);

    setCalibrationData();


    //  enable the bus

    if (!m_settings->HALOpen())
        return false;

    //  reset the ICM20948

    setBank(0);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 0x80, "Failed to initiate ICM20948 reset"))
        return false;

    m_settings->delayMs(100);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 0x01, "Failed to stop ICM20948 reset"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_2, 0x00, "Failed to stop ICM20948 reset"))
        return false;

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_WHO_AM_I, 1, &result, "Failed to read ICM20948 id"))
        return false;

    if (result != ICM20948_ID) {
        HAL_ERROR2("Incorrect %s id %d\n", IMUName(), result);
        return false;
    }

    //  now configure the various components

    if (!setGyroConfig())
        return false;

    if (!setAccelConfig())
        return false;

    if (!setSampleRate())
        return false;

    if(!compassSetup()) {
        return false;
    }

    if (!setCompassRate())
        return false;

    //  enable the sensors

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 1, "Failed to set pwr_mgmt_1"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_2, 0, "Failed to set pwr_mgmt_2"))
         return false;

    //  select the data to go into the FIFO and enable

    if (!resetFifo())
        return false;

    gyroBiasInit();

    HAL_INFO1("%s init complete\n", IMUName());
    return true;
}


bool RTIMUICM20948::resetFifo()
{
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_ENABLE, 0, "Writing int enable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_1, 0, "Writing fifo enable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_2, 0, "Writing fifo enable"))
        return false;
    // Turn on internal clock source
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 0, "Writing fifo enable"))
        return false;
    // Disable I2C master
    // Disable FIFO and I2c master modes
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0, "Writing user control"))
        return false;
    // Rest FIFO and DMP
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x08, "Resetting fifo"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x40, "Enabling the fifo"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_ENABLE, 1, "Writing int enable"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_RST, 0x1F, "Failed to set FIFO enables"))
        return false;
    m_settings->delayMs(10);
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_RST, 0x00, "Failed to set FIFO enables"))
        return false;
    m_settings->delayMs(15);

    return true;
}

bool RTIMUICM20948::setBank(int bank)
{
    if (m_bank != bank) {
        if (!m_settings->HALWrite(m_slaveAddr, ICM20948_BANK_SEL, bank << 4, "Failed to write gyro config"))
             return false;
        m_bank = bank;
    }
    return true;
}

bool RTIMUICM20948::setGyroConfig()
{
    unsigned char value;
    unsigned char gyroFsr = (m_gyroFsr & 7) << 1;
    unsigned char gyroLpf = (m_gyroLpf & 7) << 4;

    setBank(2);

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_GYRO_CONFIG_1, 1, &value, "Failed to read gyro config")) {
        return false;
    }

    value = (value & 0x89) | gyroFsr | gyroLpf | 0x1;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_GYRO_CONFIG_1, value, "Failed to write gyro config"))
         return false;

    return true;
}

bool RTIMUICM20948::setAccelConfig()
{
    unsigned char value;
    unsigned char accelFsr = (m_accelFsr & 7) << 1;
    unsigned char accelLpf = (m_accelLpf & 7) << 4;

    setBank(2);

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_ACCEL_CONFIG, 1, &value, "Failed to read accel config")) {
        return false;
    }

    value = (value & 0x89) | accelFsr | accelLpf | 0x1;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_CONFIG, value, "Failed to write accel config"))
         return false;

    return true;
}

bool RTIMUICM20948::setSampleRate()
{
    if (m_sampleRate > 1125)
        return true;                                        // SMPRT not used above 1000Hz

    setBank(2);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_GYRO_SMPRT_DIV, (unsigned char) m_sampleDiv,
            "Failed to set gyro sample rate"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_SMPRT_DIV1, (unsigned char) ((m_sampleDiv >> 8) & 0xFF),
            "Failed to set accel sample rate"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_SMPRT_DIV2, (unsigned char) (m_sampleDiv & 0xFF),
            "Failed to set accel sample rate"))
        return false;

    return true;
}

unsigned char RTIMUICM20948::compassRead(unsigned char reg) {
    unsigned char value;

    setBank(3);
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_ADDR, AK9916_I2C_ADDR | 0x80, "Failed to set I2C slv0 addr"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_REG, reg, "Failed to set I2C slv0 reg"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_DO, 0xff, "Failed to set I2C slv0 do"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_CTRL, 0x80 | 1, "Failed to set I2C slv0 ctrl"))
        return false;

    masterReset();

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SENS_DATA_00, 1, &value, "Failed to read compass")) {
        return false;
    }
    return value;
}

bool RTIMUICM20948::compassWrite(unsigned char reg, unsigned char value) {

    setBank(3);
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_ADDR, AK9916_I2C_ADDR, "Failed to set I2C slv0 addr"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_REG, reg, "Failed to set I2C slv0 reg"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_DO, value, "Failed to set I2C slv0 do"))
        return false;

    masterReset();

    return true;
}

bool RTIMUICM20948::compassSetup() {

    if (m_settings->m_busIsI2C) {
        // I2C mode

        bypassOff();

        // Master Enable
        setBank(3);
        if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_CTRL, 0x4d, "Failed to set I2C master mode"))
            return false;
        if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x01, "Failed to set I2C master mode"))
            return false;

        // Check magnetometer id
        if ( compassRead(AK9916_WIA) != AK9916_DEVICEID) {
            HAL_ERROR("Unable to find AK9916 of ICM20948\n");
            return false;
        }

        // Reset the magnetometer
        compassWrite(AK9916_CNTL3, 0x01);
        while (compassRead(AK9916_CNTL3) == 0x01) {
            m_settings->delayMs(1);
        }

    } else {
        //  SPI mode

        bypassOff();

        // Master Enable
        setBank(3);
        if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_CTRL, 0x4d, "Failed to set I2C master mode"))
            return false;
        if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x01, "Failed to set I2C master mode"))
            return false;

        // Check magnetometer id
        if ( compassRead(AK9916_WIA) != AK9916_DEVICEID) {
            HAL_ERROR("Unable to find AK9916 of ICM20948\n");
            return false;
        }

        // Reset the magnetometer
        compassWrite(AK9916_CNTL3, 0x01);
        while (compassRead(AK9916_CNTL3) == 0x01) {
            m_settings->delayMs(1);
        }

    }
    //  both interfaces

    // AK9916 mode continuous 100Hz
    compassWrite(AK9916_CNTL2, 0x04 << 1);

    // Set up the control info
    compassWrite(AK9916_ST1|(1<<7), 9 | (1 << 7)/*enable*/);

    return true;
}

bool RTIMUICM20948::setCompassRate()
{
    int ratebit;

    if (m_compassRate == 0)
        ratebit = 0x01 << 0;
    else if (m_compassRate == 10)
        ratebit = 0x01 << 1;
    else if (m_compassRate == 20)
        ratebit = 0x02 << 1;
    else if (m_compassRate == 50)
        ratebit = 0x03 << 1;
    else if (m_compassRate >= 100)
        ratebit = 0x04 << 1;
    else 
        return false;

    compassWrite(AK9916_CNTL2, ratebit);
    return true;
}

bool RTIMUICM20948::masterReset()
{
    unsigned char userControl;

    setBank(0);

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl |= 0x20;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, userControl, "Failed to write user_ctrl reg"))
        return false;

    return true;
}

bool RTIMUICM20948::bypassOn()
{
    unsigned char userControl;
    unsigned char value;

    setBank(0);

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl &= ~0x20;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_INT_PIN_CFG, 1, &value, "Failed to read int_pin_cfg reg"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_PIN_CFG, value | 0x02, "Failed to write int_pin_cfg reg"))
        return false;

    m_settings->delayMs(50);
    return true;
}


bool RTIMUICM20948::bypassOff()
{
    unsigned char userControl;
    unsigned char value;

    setBank(0);

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl |= 0x20;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_INT_PIN_CFG, 1, &value, "Failed to read int_pin_cfg reg"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_PIN_CFG, value & ~0x02, "Failed to write int_pin_cfg reg"))
         return false;

    m_settings->delayMs(50);
    return true;
}


int RTIMUICM20948::IMUGetPollInterval()
{
    if (m_sampleRate > 400)
        return 1;
    else
        return (400 / m_sampleRate);
}

bool RTIMUICM20948::IMURead()
{
    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[12];
    unsigned char compassData[8];

    setBank(0);
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_ACCEL_XOUT_H, 12, fifoData, "Failed to read imu data"))
         return false;

#ifdef ICM20948_CACHE_MODE
    if ((m_cacheCount == 0) && (count  >= ICM20948_FIFO_CHUNK_SIZE) && (count < (ICM20948_CACHE_SIZE * ICM20948_FIFO_CHUNK_SIZE))) {
        // special case of a small fifo and nothing cached - just handle as simple read

        if (!m_settings->HALRead(m_slaveAddr, ICM20948_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
            return false;

        if (!m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SENS_DATA_00, 8, compassData, "Failed to read compass data"))
            return false;
    } else {
        if (count >= (ICM20948_CACHE_SIZE * ICM20948_FIFO_CHUNK_SIZE)) {
            if (m_cacheCount == ICM20948_CACHE_BLOCK_COUNT) {
                // all cache blocks are full - discard oldest and update timestamp to account for lost samples
                m_imuData.timestamp += m_sampleInterval * m_cache[m_cacheOut].count;
                if (++m_cacheOut == ICM20948_CACHE_BLOCK_COUNT)
                    m_cacheOut = 0;
                m_cacheCount--;
            }

            int blockCount = count / ICM20948_FIFO_CHUNK_SIZE;   // number of chunks in fifo

            if (blockCount > ICM20948_CACHE_SIZE)
                blockCount = ICM20948_CACHE_SIZE;

            if (!m_settings->HALRead(m_slaveAddr, ICM20948_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE * blockCount,
                    m_cache[m_cacheIn].data, "Failed to read fifo data"))
                return false;

            if (!m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SENS_DATA_00, 8, m_cache[m_cacheIn].compass, "Failed to read compass data"))
                return false;

            m_cache[m_cacheIn].count = blockCount;
            m_cache[m_cacheIn].index = 0;

            m_cacheCount++;
            if (++m_cacheIn == ICM20948_CACHE_BLOCK_COUNT)
                m_cacheIn = 0;

        }

        //  now fifo has been read if necessary, get something to process

        if (m_cacheCount == 0)
            return false;

        memcpy(fifoData, m_cache[m_cacheOut].data + m_cache[m_cacheOut].index, ICM20948_FIFO_CHUNK_SIZE);
        memcpy(compassData, m_cache[m_cacheOut].compass, 8);

        m_cache[m_cacheOut].index += ICM20948_FIFO_CHUNK_SIZE;

        if (--m_cache[m_cacheOut].count == 0) {
            //  this cache block is now empty

            if (++m_cacheOut == ICM20948_CACHE_BLOCK_COUNT)
                m_cacheOut = 0;
            m_cacheCount--;
        }
    }

#else

    setBank(3);
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_ADDR, AK9916_I2C_ADDR | 0x80, "Failed to set I2C slv0 addr"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_REG, AK9916_HXL, "Failed to set I2C slv0 reg"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_DO, 0xff, "Failed to set I2C slv0 do"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_CTRL, 0x80 | 0x08 | 6, "Failed to set I2C slv0 ctrl"))
        return false;

    masterReset();

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SENS_DATA_00, 6, compassData, "Failed to read compass data"))
        return false;

#endif

    RTMath::convertToVector(fifoData, m_imuData.accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_imuData.gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData, m_imuData.compass, 0.6f, false);

    //  sort out gyro axes

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  use the compass fuse data adjustments

    m_imuData.compass.setX(m_imuData.compass.x());
    m_imuData.compass.setY(m_imuData.compass.y());
    m_imuData.compass.setZ(m_imuData.compass.z());

    //  sort out compass axes

    float temp;

    temp = m_imuData.compass.x();
    m_imuData.compass.setX(m_imuData.compass.y());
    m_imuData.compass.setY(-temp);

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    if (m_firstTime)
        m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    else
        m_imuData.timestamp += m_sampleInterval;

    m_firstTime = false;

    //  now update the filter

    updateFusion();

    return true;
}



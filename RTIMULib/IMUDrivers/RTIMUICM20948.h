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
//  skwim01@github.com


#ifndef _RTIMUICM20948_H
#define	_RTIMUICM20948_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

//#define ICM20948_CACHE_MODE

//  FIFO transfer size

#define ICM20948_FIFO_CHUNK_SIZE     12                      // gyro and accels take 12 bytes

#ifdef ICM20948_CACHE_MODE

//  Cache mode defines

#define ICM20948_CACHE_SIZE          16                      // number of chunks in a block
#define ICM20948_CACHE_BLOCK_COUNT   16                      // number of cache blocks

typedef struct
{
    unsigned char data[ICM20948_FIFO_CHUNK_SIZE * ICM20948_CACHE_SIZE];
    int count;                                              // number of chunks in the cache block
    int index;                                              // current index into the cache
    unsigned char compass[8];                               // the raw compass readings for the block

} ICM20948_CACHE_BLOCK;

#endif


class RTIMUICM20948 : public RTIMU
{
public:
    RTIMUICM20948(RTIMUSettings *settings);
    ~RTIMUICM20948();

    bool setBank(int bank);
    bool setGyroLpf(unsigned char lpf);
    bool setAccelLpf(unsigned char lpf);
    bool setSampleRate(int rate);
    bool setCompassRate(int rate);
    bool setGyroFsr(unsigned char fsr);
    bool setAccelFsr(unsigned char fsr);

    virtual const char *IMUName() { return "ICM-20948"; }
    virtual int IMUType() { return RTIMU_TYPE_ICM20948; }
    virtual bool IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();

protected:

    RTFLOAT m_compassAdjust[3];                             // the compass fuse ROM values converted for use

private:
    bool setGyroConfig();
    bool setAccelConfig();
    bool setSampleRate();
    bool compassSetup();
    bool setCompassRate();
    bool resetFifo();
    bool masterReset();
    bool bypassOn();
    bool bypassOff();
    bool compassWrite(unsigned char reg, unsigned char value);
    unsigned char compassRead(unsigned char reg);

    bool m_firstTime;                                       // if first sample

    unsigned char m_slaveAddr;                              // I2C address of ICM20948

    int m_sampleDiv;
    unsigned char m_gyroLpf;                                // gyro low pass filter setting
    unsigned char m_accelLpf;                               // accel low pass filter setting
    int m_compassRate;                                      // compass sample rate in Hz
    unsigned char m_gyroFsr;
    unsigned char m_accelFsr;
    int m_bank;

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;


#ifdef ICM20948_CACHE_MODE

    ICM20948_CACHE_BLOCK m_cache[ICM20948_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                          // the in index
    int m_cacheOut;                                         // the out index
    int m_cacheCount;                                       // number of used cache blocks

#endif

};

#endif // _RTIMUICM20948_H

#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
#define _MPU6050_6AXIS_MOTIONAPPS20_H_

#include "I2Cdev.h"
#include "helper_3dmath.h"

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#include "MPU6050.h"

uint8_t MPU6050::dmpInitialize()
{
    reset();
    delay(30);

    setSleepEnabled(false);

    setMemoryBank(0x10, true, true);
    setMemoryStartAddress(0x06);
    uint8_t hwRevision = readMemoryByte();
    setMemoryBank(0, false, false);

    uint8_t otpValid = getOTPBankValid();

    int8_t xgOffsetTC = getXGyroOffsetTC();
    int8_t ygOffsetTC = getYGyroOffsetTC();
    int8_t zgOffsetTC = getZGyroOffsetTC();

    setSlaveAddress(0, 0x7F);
    setI2CMasterModeEnabled(false);
    setSlaveAddress(0, 0x68);
    resetI2CMaster();
    delay(20);

    if (writeMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) {

        if (writeDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {

            setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

            setIntEnabled(0x12);

            setRate(4);

            setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

            setDLPFMode(MPU6050_DLPF_BW_42);

            setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

            setDMPConfig1(0x03);
            setDMPConfig2(0x00);

            setOTPBankValid(false);

            setXGyroOffsetTC(xgOffsetTC);
            setYGyroOffsetTC(ygOffsetTC);
            setZGyroOffsetTC(zgOffsetTC);

            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            resetFIFO();

            uint16_t fifoCount = getFIFOCount();
            uint8_t fifoBuffer[128];

            getFIFOBytes(fifoBuffer, fifoCount);

            setMotionDetectionThreshold(2);

            setZeroMotionDetectionThreshold(156);

            setMotionDetectionDuration(80);

            setZeroMotionDetectionDuration(0);

            resetFIFO();

            setFIFOEnabled(true);

            setDMPEnabled(true);

            resetDMP();

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            while ((fifoCount = getFIFOCount()) < 3);

            getFIFOBytes(fifoBuffer, fifoCount);

            uint8_t mpuIntStatus = getIntStatus();

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            while ((fifoCount = getFIFOCount()) < 3);

            getFIFOBytes(fifoBuffer, fifoCount);

            mpuIntStatus = getIntStatus();

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            setDMPEnabled(false);

            dmpPacketSize = 42;

            resetFIFO();
            getIntStatus();
        } else {
            return 2;
        }
    } else {
        return 1;
    }
    return 0;
}

bool MPU6050::dmpPacketAvailable() {
    return getFIFOCount() >= dmpGetFIFOPacketSize();
}

uint8_t MPU6050::dmpGetAccel(int32_t *data, const uint8_t* packet) {
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[28] << 24) + (packet[29] << 16) + (packet[30] << 8) + packet[31]);
    data[1] = ((packet[32] << 24) + (packet[33] << 16) + (packet[34] << 8) + packet[35]);
    data[2] = ((packet[36] << 24) + (packet[37] << 16) + (packet[38] << 8) + packet[39]);
    return 0;
}
uint8_t MPU6050::dmpGetAccel(int16_t *data, const uint8_t* packet) {
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[28] << 8) + packet[29];
    data[1] = (packet[32] << 8) + packet[33];
    data[2] = (packet[36] << 8) + packet[37];
    return 0;
}
uint8_t MPU6050::dmpGetAccel(VectorInt16 *v, const uint8_t* packet) {
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[28] << 8) + packet[29];
    v -> y = (packet[32] << 8) + packet[33];
    v -> z = (packet[36] << 8) + packet[37];
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 24) + (packet[1] << 16) + (packet[2] << 8) + packet[3]);
    data[1] = ((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8) + packet[7]);
    data[2] = ((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11]);
    data[3] = ((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8) + packet[15]);
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) + packet[1]);
    data[1] = ((packet[4] << 8) + packet[5]);
    data[2] = ((packet[8] << 8) + packet[9]);
    data[3] = ((packet[12] << 8) + packet[13]);
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status;
}
uint8_t MPU6050::dmpGetGyro(int32_t *data, const uint8_t* packet) {
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[16] << 24) + (packet[17] << 16) + (packet[18] << 8) + packet[19]);
    data[1] = ((packet[20] << 24) + (packet[21] << 16) + (packet[22] << 8) + packet[23]);
    data[2] = ((packet[24] << 24) + (packet[25] << 16) + (packet[26] << 8) + packet[27]);
    return 0;
}
uint8_t MPU6050::dmpGetGyro(int16_t *data, const uint8_t* packet) {
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) + packet[17];
    data[1] = (packet[20] << 8) + packet[21];
    data[2] = (packet[24] << 8) + packet[25];
    return 0;
}
uint8_t MPU6050::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    v -> x = vRaw -> x - gravity -> x*8192;
    v -> y = vRaw -> y - gravity -> y*8192;
    v -> z = vRaw -> z - gravity -> z*8192;
    return 0;
}
uint8_t MPU6050::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}

uint8_t MPU6050::dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t MPU6050::dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);
    return 0;
}
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

uint8_t MPU6050::dmpProcessFIFOPacket(const uint8_t *dmpData) {
    return 0;
}
uint8_t MPU6050::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        getFIFOBytes(buf, dmpPacketSize);

        if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;

        if (processed != 0) *processed++;
    }
    return 0;
}

uint16_t MPU6050::dmpGetFIFOPacketSize() {
    return dmpPacketSize;
}

#endif

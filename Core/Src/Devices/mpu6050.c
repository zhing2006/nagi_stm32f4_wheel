#include "Devices/mpu6050.h"

/// @brief Read byte from MPU6050 register.
/// @param mpu6050 MPU6050 structure.
/// @param reg Register address.
/// @param data Pointer to store byte value.
/// @return Error code.
static MPU6050_Error_t _MPU6050_ReadByte(MPU6050_t *mpu6050, uint8_t reg, uint8_t *data) {
  int32_t error = mpu6050->read_fn(mpu6050->addr, reg, data, 1);
  if (error != 0) {
    return MPU6050_ERROR_READ_FAILED;
  }

  return MPU6050_ERROR_OK;
}

/// @brief Write byte to MPU6050 register.
/// @param mpu6050 MPU6050 structure.
/// @param reg Register address.
/// @param data Data to write.
/// @return Error code.
static MPU6050_Error_t _MPU6050_WriteByte(MPU6050_t *mpu6050, uint8_t reg, uint8_t data) {
  int32_t error = mpu6050->write_fn(mpu6050->addr, reg, &data, 1);
  if (error != 0) {
    return MPU6050_ERROR_WRITE_FAILED;
  }

  return MPU6050_ERROR_OK;
}

MPU6050_Error_t MPU6050_Init(
  MPU6050_t *mpu6050,
  int32_t (*read_fn)(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size),
  int32_t (*write_fn)(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size),
  uint8_t addr
) {
  if (read_fn == NULL) {
    return MPU6050_ERROR_READ_FN_NULL;
  }
  if (write_fn == NULL) {
    return MPU6050_ERROR_WRITE_FN_NULL;
  }
  if (addr == 0) {
    return MPU6050_ERROR_INVALID_ADDRESS;
  }

  mpu6050->read_fn = read_fn;
  mpu6050->write_fn = write_fn;
  mpu6050->addr = addr;

  return MPU6050_ERROR_OK;
}

MPU6050_Error_t MPU6050_GetDeviceID(MPU6050_t *mpu6050, uint8_t *id) {
  return _MPU6050_ReadByte(mpu6050, MPU6050_RA_WHO_AM_I, id);
}

MPU6050_Error_t MPU6050_WriteReg(MPU6050_t *mpu6050, uint8_t reg, uint8_t data) {
  return _MPU6050_WriteByte(mpu6050, reg, data);
}

MPU6050_Error_t MPU6050_ReadReg(MPU6050_t *mpu6050, uint8_t reg, uint8_t *data) {
  return _MPU6050_ReadByte(mpu6050, reg, data);
}

MPU6050_Error_t MPU6050_GetTemperature(MPU6050_t *mpu6050, float *temperature) {
  uint8_t data[2];
  int error = mpu6050->read_fn(mpu6050->addr, MPU6050_RA_TEMP_OUT_H, data, 2);
  if (error != 0) {
    return MPU6050_ERROR_READ_FAILED;
  }

  *temperature = (int16_t)((data[0] << 8) | data[1]) / 340 + 36.53f;
  return MPU6050_ERROR_OK;
}

MPU6050_Error_t MPU6050_ResetFIFOBuffer(MPU6050_t *mpu6050) {
  return MPU6050_WriteBits(mpu6050, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1, 1);
}

MPU6050_Error_t MPU6050_EnableFIFOBuffer(MPU6050_t *mpu6050, uint8_t enable) {
  return MPU6050_WriteBits(mpu6050, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, 1, enable);
}

MPU6050_Error_t MPU6050_GetFIFOCount(MPU6050_t *mpu6050, uint16_t *count) {
  uint8_t data[2];
  int error = mpu6050->read_fn(mpu6050->addr, MPU6050_RA_FIFO_COUNTH, data, 2);
  if (error != 0) {
    return MPU6050_ERROR_READ_FAILED;
  }

  *count = (data[0] << 8) | data[1];
  return MPU6050_ERROR_OK;
}

MPU6050_Error_t MPU6050_GetFIFOData(MPU6050_t *mpu6050, uint8_t *data, uint16_t size) {
  for (uint16_t i = 0; i < size; i++) {
    int error = mpu6050->read_fn(mpu6050->addr, MPU6050_RA_FIFO_R_W, data + i, 1);
    if (error != 0) {
      return MPU6050_ERROR_READ_FAILED;
    }
  }

  return MPU6050_ERROR_OK;
}

MPU6050_Error_t MPU6050_GetRawAccel(MPU6050_t *mpu6050, int16_t *accel) {
  uint8_t data[6];
  int error = mpu6050->read_fn(mpu6050->addr, MPU6050_RA_ACCEL_XOUT_H, data, 6);
  if (error != 0) {
    return MPU6050_ERROR_READ_FAILED;
  }

  accel[0] = (int16_t)((data[0] << 8) | data[1]);
  accel[1] = (int16_t)((data[2] << 8) | data[3]);
  accel[2] = (int16_t)((data[4] << 8) | data[5]);
  return MPU6050_ERROR_OK;
}

MPU6050_Error_t MPU6050_GetRawGyro(MPU6050_t *mpu6050, int16_t *gyro) {
  uint8_t data[6];
  int error = mpu6050->read_fn(mpu6050->addr, MPU6050_RA_GYRO_XOUT_H, data, 6);
  if (error != 0) {
    return MPU6050_ERROR_READ_FAILED;
  }

  gyro[0] = (int16_t)((data[0] << 8) | data[1]);
  gyro[1] = (int16_t)((data[2] << 8) | data[3]);
  gyro[2] = (int16_t)((data[4] << 8) | data[5]);
  return MPU6050_ERROR_OK;
}

MPU6050_Error_t MPU6050_WriteBits(MPU6050_t *mpu6050, uint8_t reg, uint8_t start_bit, uint8_t length, uint8_t data) {
  uint8_t b;
  MPU6050_Error_t error = _MPU6050_ReadByte(mpu6050, reg, &b);
  if (error != MPU6050_ERROR_OK) {
    return error;
  }

  uint8_t mask = (0xFF << (start_bit + 1)) | 0xFF >> ((8 - start_bit) + length - 1);
  data <<= (8 - length);
  data >>= (7 - start_bit);
  b &= mask;
  b |= data;

  return _MPU6050_WriteByte(mpu6050, reg, b);
}

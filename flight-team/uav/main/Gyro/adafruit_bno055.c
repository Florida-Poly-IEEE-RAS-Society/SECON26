#include "adafruit_bno055.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#define I2C_MASTER_SCL_IO           6       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           5       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

// Private
static int32_t _sensorID;
static adafruit_bno055_opmode_t _mode;

static i2c_master_bus_handle_t _bus_handle;
static i2c_master_dev_handle_t _dev_handle;

static void delay(int ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_config, bus_handle);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO055_ADDRESS_A, // hmmm
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle);
}

static uint8_t bno055_read8(adafruit_bno055_reg_t reg) {
    uint8_t buffer[1] = {reg};
    i2c_master_transmit_receive(_dev_handle, buffer, 1, buffer, 1, I2C_MASTER_TIMEOUT_MS);
    return buffer[0];
}

static bool bno055_readLen(adafruit_bno055_reg_t reg, uint8_t *buffer, uint8_t len) {
    uint8_t reg_buf[1] = {(uint8_t)reg};
    return i2c_master_transmit_receive(_dev_handle, reg_buf, 1, reg_buf, 1, I2C_MASTER_TIMEOUT_MS);
}

static bool bno055_write8(adafruit_bno055_reg_t reg, uint8_t value) {
    uint8_t buffer[2] = {(uint8_t)reg, (uint8_t)value};
    return i2c_master_transmit(_dev_handle, buffer, sizeof(buffer), I2C_MASTER_TIMEOUT_MS);
}
// End Private

// Begin Public
bool bno055_begin(adafruit_bno055_opmode_t mode) {
    delay(850);
    i2c_master_init(&_bus_handle, &_dev_handle);

    delay(1000);
    uint8_t id = bno055_read8(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
        return false;
    }

    bno055_setMode(OPERATION_MODE_CONFIG);

    bno055_write8(BNO055_SYS_TRIGGER_ADDR, 0x20);

    delay(30);
    while (bno055_read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        delay(10);
    }
    delay(50);

    bno055_write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    delay(10);

    bno055_write8(BNO055_PAGE_ID_ADDR, 0);

    bno055_write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
    delay(10);
    bno055_setMode(mode);
    delay(20);

    return true;
}

void bno055_setMode(adafruit_bno055_opmode_t mode) {
    _mode = mode;
    bno055_write8(BNO055_OPR_MODE_ADDR, _mode);
    delay(30);
}

adafruit_bno055_opmode_t getMode() {
    return (adafruit_bno055_opmode_t)bno055_read8(BNO055_OPR_MODE_ADDR);
}

void bno055_setAxisRemap(adafruit_bno055_axis_remap_config_t remapcode) {
    adafruit_bno055_opmode_t modeback = _mode;

    bno055_setMode(OPERATION_MODE_CONFIG);
    delay(25);
    bno055_write8(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
    delay(10);
    /* Set the requested operating mode (see section 3.3) */
    bno055_setMode(modeback);
    delay(20);
}

void bno055_setAxisSign(adafruit_bno055_axis_remap_sign_t remapsign) {
    adafruit_bno055_opmode_t modeback = _mode;

    bno055_setMode(OPERATION_MODE_CONFIG);
    delay(25);
    bno055_write8(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
    delay(10);
    /* Set the requested operating mode (see section 3.3) */
    bno055_setMode(modeback);
    delay(20);
}

void bno055_getRevInfo(adafruit_bno055_rev_info_t *info) {
    uint8_t a, b;

    memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

    /* Check the accelerometer revision */
    info->accel_rev = bno055_read8(BNO055_ACCEL_REV_ID_ADDR);

    /* Check the magnetometer revision */
    info->mag_rev = bno055_read8(BNO055_MAG_REV_ID_ADDR);

    /* Check the gyroscope revision */
    info->gyro_rev = bno055_read8(BNO055_GYRO_REV_ID_ADDR);

    /* Check the SW revision */
    info->bl_rev = bno055_read8(BNO055_BL_REV_ID_ADDR);

    a = bno055_read8(BNO055_SW_REV_ID_LSB_ADDR);
    b = bno055_read8(BNO055_SW_REV_ID_MSB_ADDR);
    info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

void bno055_setExtCrystalUse(bool usextal) {
    adafruit_bno055_opmode_t modeback = _mode;

    /* Switch to config mode (just in case since this is the default) */
    bno055_setMode(OPERATION_MODE_CONFIG);
    delay(25);
    bno055_write8(BNO055_PAGE_ID_ADDR, 0);
    if (usextal) {
        bno055_write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
    } else {
        bno055_write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
    }
    delay(10);
    /* Set the requested operating mode (see section 3.3) */
    bno055_setMode(modeback);
    delay(20);
}

void bno055_getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error) {
    bno055_write8(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms
   */

  if (system_status != 0)
    *system_status = bno055_read8(BNO055_SYS_STAT_ADDR);

  /* Self Test Results
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

  if (self_test_result != 0)
    *self_test_result = bno055_read8(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */

  if (system_error != 0)
    *system_error = bno055_read8(BNO055_SYS_ERR_ADDR);

  delay(200);
}

void bno055_getCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    uint8_t calData = bno055_read8(BNO055_CALIB_STAT_ADDR);
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
}

Vector3 bno055_getVector(adafruit_vector_type_t vector_type) {
    Vector3 xyz;
    uint8_t buffer[6] = {0};

    int16_t x, y, z;
    x = y = z = 0;

    /* Read vector data (6 bytes) */
    bno055_readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);

    x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    /*!
     * Convert the value to an appropriate range (section 3.6.4)
     * and assign the value to the Vector type
     */
    switch (vector_type) {
    case VECTOR_MAGNETOMETER:
        /* 1uT = 16 LSB */
        xyz.x = ((double)x) / 16.0;
        xyz.y = ((double)y) / 16.0;
        xyz.z = ((double)z) / 16.0;
        break;
    case VECTOR_GYROSCOPE:
        /* 1dps = 16 LSB */
        xyz.x = ((double)x) / 16.0;
        xyz.y = ((double)y) / 16.0;
        xyz.z = ((double)z) / 16.0;
        break;
    case VECTOR_EULER:
        /* 1 degree = 16 LSB */
        xyz.x = ((double)x) / 16.0;
        xyz.y = ((double)y) / 16.0;
        xyz.z = ((double)z) / 16.0;
        break;
    case VECTOR_ACCELEROMETER:
        /* 1m/s^2 = 100 LSB */
        xyz.x = ((double)x) / 100.0;
        xyz.y = ((double)y) / 100.0;
        xyz.z = ((double)z) / 100.0;
        break;
    case VECTOR_LINEARACCEL:
        /* 1m/s^2 = 100 LSB */
        xyz.x = ((double)x) / 100.0;
        xyz.y = ((double)y) / 100.0;
        xyz.z = ((double)z) / 100.0;
        break;
    case VECTOR_GRAVITY:
        /* 1m/s^2 = 100 LSB */
        xyz.x = ((double)x) / 100.0;
        xyz.y = ((double)y) / 100.0;
        xyz.z = ((double)z) / 100.0;
        break;
    }
    
    return xyz;
}

Quaternion bno055_getQuat(void) {
    uint8_t buffer[8] = {0};

    int16_t x, y, z, w;
    x = y = z = w = 0;

    /* Read quat data (8 bytes) */
    bno055_readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
    z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

    /*!
     * Assign to Quaternion
     * See
     * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
     * 3.6.5.5 Orientation (Quaternion)
     */
    const double scale = (1.0 / (1 << 14));
    Quaternion quat = {
        .w = scale * w,
        .x = scale * x,
        .y = scale * y,
        .z = scale * z,
    };

    return quat;
}

int8_t bno055_getTemp(void) {
    int8_t temp = (int8_t)(bno055_read8(BNO055_TEMP_ADDR));
    return temp;
}

/* Adafruit_Sensor implementation */
bool bno055_getEvent1(sensors_event_t *event) {
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_ORIENTATION;
    event->timestamp = 0.0f; //millis(); // uhh

    /* Get a Euler angle sample for orientation */
    Vector3 euler = bno055_getVector(VECTOR_EULER);
    event->orientation.x = euler.x;
    event->orientation.y = euler.y;
    event->orientation.z = euler.z;

  return true;
}

bool bno055_getEvent2(sensors_event_t * event, adafruit_vector_type_t vec_type) {
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->timestamp = 0.0f; //millis(); // uhh

    // read the data according to vec_type
    Vector3 vec;
    if (vec_type == VECTOR_LINEARACCEL) {
        event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
        vec = bno055_getVector(VECTOR_LINEARACCEL);

        event->acceleration.x = vec.x;
        event->acceleration.y = vec.y;
        event->acceleration.z = vec.z;
    } else if (vec_type == VECTOR_ACCELEROMETER) {
        event->type = SENSOR_TYPE_ACCELEROMETER;
        vec = bno055_getVector(VECTOR_ACCELEROMETER);

        event->acceleration.x = vec.x;
        event->acceleration.y = vec.y;
        event->acceleration.z = vec.z;
    } else if (vec_type == VECTOR_GRAVITY) {
        event->type = SENSOR_TYPE_GRAVITY;
        vec = bno055_getVector(VECTOR_GRAVITY);

        event->acceleration.x = vec.x;
        event->acceleration.y = vec.y;
        event->acceleration.z = vec.z;
    } else if (vec_type == VECTOR_EULER) {
        event->type = SENSOR_TYPE_ORIENTATION;
        vec = bno055_getVector(VECTOR_EULER);

        event->orientation.x = vec.x;
        event->orientation.y = vec.y;
        event->orientation.z = vec.z;
    } else if (vec_type == VECTOR_GYROSCOPE) {
        event->type = SENSOR_TYPE_GYROSCOPE;
        vec = bno055_getVector(VECTOR_GYROSCOPE);

        event->gyro.x = vec.x * SENSORS_DPS_TO_RADS;
        event->gyro.y = vec.y * SENSORS_DPS_TO_RADS;
        event->gyro.z = vec.z * SENSORS_DPS_TO_RADS;
    } else if (vec_type == VECTOR_MAGNETOMETER) {
        event->type = SENSOR_TYPE_MAGNETIC_FIELD;
        vec = bno055_getVector(VECTOR_MAGNETOMETER);

        event->magnetic.x = vec.x;
        event->magnetic.y = vec.y;
        event->magnetic.z = vec.z;
    }

    return true;
}

/* Functions to deal with raw calibration data */
bool bno055_getSensorOffsets1(uint8_t *calibData) {
    if (bno055_isFullyCalibrated()) {
        adafruit_bno055_opmode_t lastMode = _mode;
        bno055_setMode(OPERATION_MODE_CONFIG);

        bno055_readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

        bno055_setMode(lastMode);
        return true;
    }
    return false;
}

bool bno055_getSensorOffsets2(adafruit_bno055_offsets_t *offsets_type) {
    if (bno055_isFullyCalibrated()) {
        adafruit_bno055_opmode_t lastMode = _mode;
        bno055_setMode(OPERATION_MODE_CONFIG);
        delay(25);

        /* Accel offset range depends on the G-range:
           +/-2g  = +/- 2000 mg
           +/-4g  = +/- 4000 mg
           +/-8g  = +/- 8000 mg
           +/-1§g = +/- 16000 mg */
        offsets_type->accel_offset_x = (bno055_read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) |
                                      (bno055_read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type->accel_offset_y = (bno055_read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |
                                      (bno055_read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type->accel_offset_z = (bno055_read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |
                                      (bno055_read8(ACCEL_OFFSET_Z_LSB_ADDR));

        /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
        offsets_type->mag_offset_x =
            (bno055_read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (bno055_read8(MAG_OFFSET_X_LSB_ADDR));
        offsets_type->mag_offset_y =
            (bno055_read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (bno055_read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type->mag_offset_z =
            (bno055_read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (bno055_read8(MAG_OFFSET_Z_LSB_ADDR));

        /* Gyro offset range depends on the DPS range:
           2000 dps = +/- 32000 LSB
           1000 dps = +/- 16000 LSB
           500 dps = +/- 8000 LSB
           250 dps = +/- 4000 LSB
           125 dps = +/- 2000 LSB
           ... where 1 DPS = 16 LSB */
        offsets_type->gyro_offset_x =
            (bno055_read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (bno055_read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type->gyro_offset_y =
            (bno055_read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (bno055_read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type->gyro_offset_z =
            (bno055_read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (bno055_read8(GYRO_OFFSET_Z_LSB_ADDR));

        /* Accelerometer radius = +/- 1000 LSB */
        offsets_type->accel_radius =
            (bno055_read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (bno055_read8(ACCEL_RADIUS_LSB_ADDR));

        /* Magnetometer radius = +/- 960 LSB */
        offsets_type->mag_radius =
            (bno055_read8(MAG_RADIUS_MSB_ADDR) << 8) | (bno055_read8(MAG_RADIUS_LSB_ADDR));

        bno055_setMode(lastMode);
        return true;
    }
    return false;
}

void bno055_setSensorOffsets3(uint8_t const *calibData) {
    adafruit_bno055_opmode_t lastMode = _mode;
    bno055_setMode(OPERATION_MODE_CONFIG);
    delay(25);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    /* A writeLen() would make this much cleaner */
    bno055_write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    bno055_write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
    bno055_write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
    bno055_write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
    bno055_write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
    bno055_write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

    bno055_write8(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
    bno055_write8(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
    bno055_write8(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
    bno055_write8(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
    bno055_write8(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
    bno055_write8(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

    bno055_write8(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
    bno055_write8(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
    bno055_write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
    bno055_write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
    bno055_write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
    bno055_write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

    bno055_write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
    bno055_write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

    bno055_write8(MAG_RADIUS_LSB_ADDR, calibData[20]);
    bno055_write8(MAG_RADIUS_MSB_ADDR, calibData[21]);

    bno055_setMode(lastMode);
}

void bno055_setSensorOffsets4(adafruit_bno055_offsets_t const *offsets_type) {
     adafruit_bno055_opmode_t lastMode = _mode;
     bno055_setMode(OPERATION_MODE_CONFIG);
     delay(25);

     /* Note: Configuration will take place only when user writes to the last
        byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
        Therefore the last byte must be written whenever the user wants to
        changes the configuration. */

     bno055_write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type->accel_offset_x) & 0x0FF);
     bno055_write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type->accel_offset_x >> 8) & 0x0FF);
     bno055_write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type->accel_offset_y) & 0x0FF);
     bno055_write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type->accel_offset_y >> 8) & 0x0FF);
     bno055_write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type->accel_offset_z) & 0x0FF);
     bno055_write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type->accel_offset_z >> 8) & 0x0FF);

     bno055_write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type->mag_offset_x) & 0x0FF);
     bno055_write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type->mag_offset_x >> 8) & 0x0FF);
     bno055_write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type->mag_offset_y) & 0x0FF);
     bno055_write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type->mag_offset_y >> 8) & 0x0FF);
     bno055_write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type->mag_offset_z) & 0x0FF);
     bno055_write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type->mag_offset_z >> 8) & 0x0FF);

     bno055_write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type->gyro_offset_x) & 0x0FF);
     bno055_write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type->gyro_offset_x >> 8) & 0x0FF);
     bno055_write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type->gyro_offset_y) & 0x0FF);
     bno055_write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type->gyro_offset_y >> 8) & 0x0FF);
     bno055_write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type->gyro_offset_z) & 0x0FF);
     bno055_write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type->gyro_offset_z >> 8) & 0x0FF);

     bno055_write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type->accel_radius) & 0x0FF);
     bno055_write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type->accel_radius >> 8) & 0x0FF);

     bno055_write8(MAG_RADIUS_LSB_ADDR, (offsets_type->mag_radius) & 0x0FF);
     bno055_write8(MAG_RADIUS_MSB_ADDR, (offsets_type->mag_radius >> 8) & 0x0FF);

     bno055_setMode(lastMode);
}

bool bno055_isFullyCalibrated(void) {
    uint8_t system, gyro, accel, mag;
    bno055_getCalibration(&system, &gyro, &accel, &mag);

    switch (_mode) {
    case OPERATION_MODE_ACCONLY:
        return (accel == 3);
    case OPERATION_MODE_MAGONLY:
        return (mag == 3);
    case OPERATION_MODE_GYRONLY:
    case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
        return (gyro == 3);
    case OPERATION_MODE_ACCMAG:
    case OPERATION_MODE_COMPASS:
        return (accel == 3 && mag == 3);
    case OPERATION_MODE_ACCGYRO:
    case OPERATION_MODE_IMUPLUS:
        return (accel == 3 && gyro == 3);
    case OPERATION_MODE_MAGGYRO:
        return (mag == 3 && gyro == 3);
    default:
        return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
}

/* Power managments functions */
void bno055_enterSuspendMode(void) {
    adafruit_bno055_opmode_t modeback = _mode;

    /* Switch to config mode (just in case since this is the default) */
    bno055_setMode(OPERATION_MODE_CONFIG);
    delay(25);
    bno055_write8(BNO055_PWR_MODE_ADDR, 0x02);
    /* Set the requested operating mode (see section 3.3) */
    bno055_setMode(modeback);
    delay(20);
}

void bno055_enterNormalMode(void) {
    adafruit_bno055_opmode_t modeback = _mode;

    /* Switch to config mode (just in case since this is the default) */
    bno055_setMode(OPERATION_MODE_CONFIG);
    delay(25);
    bno055_write8(BNO055_PWR_MODE_ADDR, 0x00);
    /* Set the requested operating mode (see section 3.3) */
    bno055_setMode(modeback);
    delay(20);
}

// End Public

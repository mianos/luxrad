/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "apds9960.h"

// static const char *TAG = "apds9960";

#define APDS9960_TIMEOUT_MS_DEFAULT   1000U

typedef struct apds9960_dev_t {
    i2c_master_dev_handle_t i2c_dev;

    uint8_t                dev_addr;
    uint32_t               timeout; /* milliseconds */
    apds9960_control_t     _control_t;
    apds9960_enable_t      _enable_t;
    apds9960_config1_t     _config1_t;
    apds9960_config2_t     _config2_t;
    apds9960_config3_t     _config3_t;
    apds9960_gconf1_t      _gconf1_t;
    apds9960_gconf2_t      _gconf2_t;
    apds9960_gconf3_t      _gconf3_t;
    apds9960_gconf4_t      _gconf4_t;
    apds9960_status_t      _status_t;
    apds9960_gstatus_t     _gstatus_t;
    apds9960_propulse_t    _ppulse_t;
    apds9960_gespulse_t    _gpulse_t;
    apds9960_pers_t        _pers_t;
    uint8_t                gest_cnt;
    uint8_t                up_cnt;
    uint8_t                down_cnt;
    uint8_t                left_cnt;
    uint8_t                right_cnt;
} apds9960_dev_t;

/* -------------------------------------------------------------------------- */
/* I2C helpers                                                                */
/* -------------------------------------------------------------------------- */

static inline int _timeout_ms(const apds9960_dev_t *d)
{
    return (int)d->timeout;
}

static esp_err_t _i2c_write_byte(const apds9960_dev_t *d, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(d->i2c_dev, buf, sizeof(buf), _timeout_ms(d));
}

/* Write a value to a command/clear register by sending reg + one data byte. */
static esp_err_t _i2c_write_cmd(const apds9960_dev_t *d, uint8_t cmd_reg)
{
    uint8_t buf[2] = { cmd_reg, 0x00 };
    return i2c_master_transmit(d->i2c_dev, buf, sizeof(buf), _timeout_ms(d));
}

static esp_err_t _i2c_read_byte(const apds9960_dev_t *d, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(d->i2c_dev, &reg, 1, val, 1, _timeout_ms(d));
}

static esp_err_t _i2c_read_bytes(const apds9960_dev_t *d, uint8_t reg,
                                 uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(d->i2c_dev, &reg, 1, data, len, _timeout_ms(d));
}

static float __powf(const float x, const float y)
{
    return (float)(pow((double)x, (double)y));
}

/* -------------------------------------------------------------------------- */
/* Small bitfield pack/unpack helpers                                         */
/* -------------------------------------------------------------------------- */

uint8_t apds9960_get_enable(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_enable_t.gen << 6) | (sens->_enable_t.pien << 5) | (sens->_enable_t.aien << 4)
           | (sens->_enable_t.wen << 3) | (sens->_enable_t.pen << 2) | (sens->_enable_t.aen << 1) | sens->_enable_t.pon;
}

uint8_t apds9960_get_pers(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_pers_t.ppers << 4) | sens->_pers_t.apers;
}

uint8_t apds9960_get_ppulse(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_ppulse_t.pplen << 6) | sens->_ppulse_t.ppulse;
}

uint8_t apds9960_get_gpulse(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_gpulse_t.gplen << 6) | sens->_gpulse_t.gpulse;
}

uint8_t apds9960_get_control(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_control_t.leddrive << 6) | (sens->_control_t.pgain << 2) | sens->_control_t.again;
}

uint8_t apds9960_get_config1(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return sens->_config1_t.wlong << 1;
}

uint8_t apds9960_get_config2(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_config2_t.psien << 7) | (sens->_config2_t.cpsien << 6) | (sens->_config2_t.led_boost << 4) | 1;
}

uint8_t apds9960_get_config3(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_config3_t.pcmp << 5) | (sens->_config3_t.sai << 4) | (sens->_config3_t.pmask_u << 3)
           | (sens->_config3_t.pmask_d << 2) | (sens->_config3_t.pmask_l << 1) | sens->_config3_t.pmask_r;
}

void apds9960_set_status(apds9960_handle_t sensor, uint8_t data)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_status_t.avalid = data & 0x01;
    sens->_status_t.pvalid = (data >> 1) & 0x01;
    sens->_status_t.gint   = (data >> 2) & 0x01;
    sens->_status_t.aint   = (data >> 4) & 0x01;
    sens->_status_t.pint   = (data >> 5) & 0x01;
    sens->_status_t.pgsat  = (data >> 6) & 0x01;
    sens->_status_t.cpsat  = (data >> 7) & 0x01;
}

void apds9960_set_gstatus(apds9960_handle_t sensor, uint8_t data)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_gstatus_t.gfov   = (data >> 1) & 0x01;
    sens->_gstatus_t.gvalid = data & 0x01;
}

uint8_t apds9960_get_gconf1(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_gconf1_t.gfifoth << 7) | (sens->_gconf1_t.gexmsk << 5) | sens->_gconf1_t.gexpers;
}

uint8_t apds9960_get_gconf2(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_gconf2_t.ggain << 5) | (sens->_gconf2_t.gldrive << 3) | sens->_gconf2_t.gwtime;
}

uint8_t apds9960_get_gconf3(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return sens->_gconf3_t.gdims;
}

uint8_t apds9960_get_gconf4(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return (sens->_gconf4_t.gien << 1) | sens->_gconf4_t.gmode;
}

void apds9960_set_gconf4(apds9960_handle_t sensor, uint8_t data)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_gconf4_t.gien  = (data >> 1) & 0x01;
    sens->_gconf4_t.gmode = data & 0x01;
}

void apds9960_reset_counts(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->gest_cnt  = 0;
    sens->up_cnt    = 0;
    sens->down_cnt  = 0;
    sens->left_cnt  = 0;
    sens->right_cnt = 0;
}

/* -------------------------------------------------------------------------- */
/* Public driver API                                                          */
/* -------------------------------------------------------------------------- */

esp_err_t apds9960_set_timeout(apds9960_handle_t sensor, uint32_t tout_ms)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->timeout = tout_ms;
    return ESP_OK;
}

esp_err_t apds9960_get_deviceid(apds9960_handle_t sensor, uint8_t *deviceid)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return _i2c_read_byte(sens, APDS9960_WHO_AM_I_REG, deviceid);
}

esp_err_t apds9960_set_mode(apds9960_handle_t sensor, apds9960_mode_t mode)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t tmp;

    if (_i2c_read_byte(sens, APDS9960_MODE_ENABLE, &tmp) != ESP_OK) {
        return ESP_FAIL;
    }

    tmp |= mode;
    return _i2c_write_byte(sens, APDS9960_MODE_ENABLE, tmp);
}

apds9960_mode_t apds9960_get_mode(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t value;

    if (_i2c_read_byte(sens, APDS9960_MODE_ENABLE, &value) != ESP_OK) {
        return (apds9960_mode_t)0xFF;
    }

    return (apds9960_mode_t)value;
}

esp_err_t apds9960_enable_gesture_engine(apds9960_handle_t sensor, bool en)
{
    esp_err_t ret;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;

    if (!en) {
        sens->_gconf4_t.gmode = 0;
        if (_i2c_write_byte(sens, APDS9960_GCONF4,
                            (sens->_gconf4_t.gien << 1) | sens->_gconf4_t.gmode) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    sens->_enable_t.gen = en;
    ret = _i2c_write_byte(sens, APDS9960_MODE_ENABLE,
                          (sens->_enable_t.gen << 6) | (sens->_enable_t.pien << 5) | (sens->_enable_t.aien << 4)
                          | (sens->_enable_t.wen << 3) | (sens->_enable_t.pen << 2) | (sens->_enable_t.aen << 1)
                          | sens->_enable_t.pon);
    apds9960_reset_counts(sensor);
    return ret;
}

esp_err_t apds9960_set_led_drive_boost(apds9960_handle_t sensor, apds9960_leddrive_t drive,
                                       apds9960_ledboost_t boost)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_config2_t.led_boost = boost;

    if (_i2c_write_byte(sens, APDS9960_CONFIG2,
                        (sens->_config2_t.psien << 7) | (sens->_config2_t.cpsien << 6)
                        | (sens->_config2_t.led_boost << 4) | 1) != ESP_OK) {
        return ESP_FAIL;
    }

    sens->_control_t.leddrive = drive;
    return _i2c_write_byte(sens, APDS9960_CONTROL,
                           ((sens->_control_t.leddrive << 6) | (sens->_control_t.pgain << 2) | sens->_control_t.again));
}

esp_err_t apds9960_set_wait_time(apds9960_handle_t sensor, uint8_t time)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return _i2c_write_byte(sens, APDS9960_WTIME, time);
}

esp_err_t apds9960_set_adc_integration_time(apds9960_handle_t sensor, uint16_t iTimeMS)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    float temp = (float)iTimeMS;
    temp /= 2.78f;
    temp = 256.0f - temp;

    if (temp > 255.0f) temp = 255.0f;
    if (temp < 0.0f)   temp = 0.0f;

    return _i2c_write_byte(sens, APDS9960_ATIME, (uint8_t)temp);
}

float apds9960_get_adc_integration_time(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t val;
    if (_i2c_read_byte(sens, APDS9960_ATIME, &val) != ESP_OK) {
        return -1.0f;
    }
    float temp = 256.0f - (float)val;
    temp *= 2.78f;
    return temp;
}

esp_err_t apds9960_set_ambient_light_gain(apds9960_handle_t sensor, apds9960_again_t again)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_control_t.again = again;
    return _i2c_write_byte(sens, APDS9960_CONTROL,
                           ((sens->_control_t.leddrive << 6) | (sens->_control_t.pgain << 2) | sens->_control_t.again));
}

apds9960_again_t apds9960_get_ambient_light_gain(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t val;

    if (_i2c_read_byte(sens, APDS9960_CONTROL, &val) != ESP_OK) {
        return (apds9960_again_t)0xFF;
    }

    return (apds9960_again_t)(val & 0x03);
}

esp_err_t apds9960_enable_gesture_interrupt(apds9960_handle_t sensor, bool en)
{
    esp_err_t ret;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_enable_t.gen = en;
    ret = _i2c_write_byte(sens, APDS9960_MODE_ENABLE,
                          ((sens->_enable_t.gen << 6) | (sens->_enable_t.pien << 5) | (sens->_enable_t.aien << 4)
                           | (sens->_enable_t.wen << 3) | (sens->_enable_t.pen << 2) | (sens->_enable_t.aen << 1)
                           | sens->_enable_t.pon));
    apds9960_clear_interrupt(sensor);
    return ret;
}

esp_err_t apds9960_enable_proximity_engine(apds9960_handle_t sensor, bool en)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_enable_t.pen = en;
    return _i2c_write_byte(sens, APDS9960_MODE_ENABLE,
                           ((sens->_enable_t.gen << 6) | (sens->_enable_t.pien << 5) | (sens->_enable_t.aien << 4)
                            | (sens->_enable_t.wen << 3) | (sens->_enable_t.pen << 2) | (sens->_enable_t.aen << 1)
                            | sens->_enable_t.pon));
}

esp_err_t apds9960_set_proximity_gain(apds9960_handle_t sensor, apds9960_pgain_t pgain)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_control_t.pgain = pgain;
    return _i2c_write_byte(sens, APDS9960_CONTROL,
                           (sens->_control_t.leddrive << 6) | (sens->_control_t.pgain << 2) | (sens->_control_t.again));
}

apds9960_pgain_t apds9960_get_proximity_gain(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t val;

    if (_i2c_read_byte(sens, APDS9960_CONTROL, &val) != ESP_OK) {
        return (apds9960_pgain_t)0xFF;
    }

    return (apds9960_pgain_t)((val >> 2) & 0x03);
}

esp_err_t apds9960_set_proximity_pulse(apds9960_handle_t sensor, apds9960_ppulse_len_t pLen, uint8_t pulses)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;

    if (pulses < 1)  pulses = 1;
    if (pulses > 64) pulses = 64;

    pulses--;
    sens->_ppulse_t.pplen  = pLen;
    sens->_ppulse_t.ppulse = pulses;
    return _i2c_write_byte(sens, APDS9960_PPULSE,
                           (sens->_ppulse_t.pplen << 6) | sens->_ppulse_t.ppulse);
}

esp_err_t apds9960_enable_color_engine(apds9960_handle_t sensor, bool en)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_enable_t.aen = en;
    return _i2c_write_byte(sens, APDS9960_MODE_ENABLE,
                           ((sens->_enable_t.gen << 6) | (sens->_enable_t.pien << 5) | (sens->_enable_t.aien << 4)
                            | (sens->_enable_t.wen << 3) | (sens->_enable_t.pen << 2) | (sens->_enable_t.aen << 1)
                            | sens->_enable_t.pon));
}

bool apds9960_color_data_ready(apds9960_handle_t sensor)
{
    uint8_t data;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    _i2c_read_byte(sens, APDS9960_STATUS, &data);
    apds9960_set_status(sensor, data);
    return sens->_status_t.avalid;
}

esp_err_t apds9960_get_color_data(apds9960_handle_t sensor, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t data[2];

    _i2c_read_bytes(sens, APDS9960_CDATAL, data, 2);
    *c = (uint16_t)((data[1] << 8) | data[0]);
    _i2c_read_bytes(sens, APDS9960_RDATAL, data, 2);
    *r = (uint16_t)((data[1] << 8) | data[0]);
    _i2c_read_bytes(sens, APDS9960_GDATAL, data, 2);
    *g = (uint16_t)((data[1] << 8) | data[0]);
    _i2c_read_bytes(sens, APDS9960_BDATAL, data, 2);
    *b = (uint16_t)((data[1] << 8) | data[0]);
    return ESP_OK;
}

uint16_t apds9960_calculate_color_temperature(apds9960_handle_t sensor, uint16_t r, uint16_t g, uint16_t b)
{
    (void)sensor;
    float X, Y, Z;
    float xc, yc;
    float n;
    float cct;

    X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
    Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
    Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

    xc = X / (X + Y + Z);
    yc = Y / (X + Y + Z);

    n = (xc - 0.3320F) / (0.1858F - yc);

    cct = (449.0F * __powf(n, 3)) + (3525.0F * __powf(n, 2)) + (6823.3F * n) + 5520.33F;
    return (uint16_t)cct;
}

uint16_t apds9960_calculate_lux(apds9960_handle_t sensor, uint16_t r, uint16_t g, uint16_t b)
{
    (void)sensor;
    float illuminance;
    illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
    if (illuminance < 0.0f) illuminance = 0.0f;
    if (illuminance > 65535.0f) illuminance = 65535.0f;
    return (uint16_t)illuminance;
}

esp_err_t apds9960_enable_color_interrupt(apds9960_handle_t sensor, bool en)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_enable_t.aien = en;
    return _i2c_write_byte(sens, APDS9960_MODE_ENABLE,
                           ((sens->_enable_t.gen << 6) | (sens->_enable_t.pien << 5) | (sens->_enable_t.aien << 4)
                            | (sens->_enable_t.wen << 3) | (sens->_enable_t.pen << 2) | (sens->_enable_t.aen << 1)
                            | sens->_enable_t.pon));
}

esp_err_t apds9960_set_int_limits(apds9960_handle_t sensor, uint16_t low, uint16_t high)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    _i2c_write_byte(sens, APDS9960_AILTL, (uint8_t)(low & 0xFF));
    _i2c_write_byte(sens, APDS9960_AILTH, (uint8_t)(low >> 8));
    _i2c_write_byte(sens, APDS9960_AIHTL, (uint8_t)(high & 0xFF));
    return _i2c_write_byte(sens, APDS9960_AIHTH, (uint8_t)(high >> 8));
}

esp_err_t apds9960_enable_proximity_interrupt(apds9960_handle_t sensor, bool en)
{
    esp_err_t ret;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_enable_t.pien = en;
    ret = _i2c_write_byte(sens, APDS9960_MODE_ENABLE,
                          ((sens->_enable_t.gen << 6) | (sens->_enable_t.pien << 5) | (sens->_enable_t.aien << 4)
                           | (sens->_enable_t.wen << 3) | (sens->_enable_t.pen << 2) | (sens->_enable_t.aen << 1)
                           | sens->_enable_t.pon));
    apds9960_clear_interrupt(sensor);
    return ret;
}

uint8_t apds9960_read_proximity(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t data = 0;
    if (_i2c_read_byte(sens, APDS9960_PDATA, &data) != ESP_OK) {
        return ERROR;
    }
    return data;
}

esp_err_t apds9960_set_proximity_interrupt_threshold(apds9960_handle_t sensor, uint8_t low, uint8_t high,
                                                     uint8_t persistence)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    _i2c_write_byte(sens, APDS9960_PILT, low);
    _i2c_write_byte(sens, APDS9960_PIHT, high);

    if (persistence > 7) {
        persistence = 7;
    }
    sens->_pers_t.ppers = persistence;
    return _i2c_write_byte(sens, APDS9960_PERS, (sens->_pers_t.ppers << 4) | sens->_pers_t.apers);
}

bool apds9960_get_proximity_interrupt(apds9960_handle_t sensor)
{
    uint8_t data;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    _i2c_read_byte(sens, APDS9960_STATUS, &data);
    apds9960_set_status(sensor, data);
    return sens->_status_t.pint;
}

esp_err_t apds9960_clear_interrupt(apds9960_handle_t sensor)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return _i2c_write_cmd(sens, APDS9960_AICLEAR);
}

esp_err_t apds9960_enable(apds9960_handle_t sensor, bool en)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_enable_t.pon = en;
    return _i2c_write_byte(sens, APDS9960_MODE_ENABLE,
                           ((sens->_enable_t.gen << 6) | (sens->_enable_t.pien << 5) | (sens->_enable_t.aien << 4)
                            | (sens->_enable_t.wen << 3) | (sens->_enable_t.pen << 2) | (sens->_enable_t.aen << 1)
                            | sens->_enable_t.pon));
}

esp_err_t apds9960_set_gesture_dimensions(apds9960_handle_t sensor, uint8_t dims)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_gconf3_t.gdims = dims;
    return _i2c_write_byte(sens, APDS9960_GCONF3, sens->_gconf3_t.gdims);
}

esp_err_t apds9960_set_light_intlow_threshold(apds9960_handle_t sensor, uint16_t threshold)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t val_low  = (uint8_t)(threshold & 0x00FF);
    uint8_t val_high = (uint8_t)((threshold & 0xFF00) >> 8);

    if (_i2c_write_byte(sens, APDS9960_AILTL, val_low) != ESP_OK) {
        return ESP_FAIL;
    }
    return _i2c_write_byte(sens, APDS9960_AILTH, val_high);
}

esp_err_t apds9960_set_light_inthigh_threshold(apds9960_handle_t sensor, uint16_t threshold)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t val_low  = (uint8_t)(threshold & 0x00FF);
    uint8_t val_high = (uint8_t)((threshold & 0xFF00) >> 8);

    if (_i2c_write_byte(sens, APDS9960_AIHTL, val_low) != ESP_OK) {
        return ESP_FAIL;
    }
    return _i2c_write_byte(sens, APDS9960_AIHTH, val_high);
}

esp_err_t apds9960_set_gesture_fifo_threshold(apds9960_handle_t sensor, uint8_t thresh)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_gconf1_t.gfifoth = thresh;
    return _i2c_write_byte(sens, APDS9960_GCONF1,
                           ((sens->_gconf1_t.gfifoth << 7) | (sens->_gconf1_t.gexmsk << 5) | sens->_gconf1_t.gexpers));
}

esp_err_t apds9960_set_gesture_waittime(apds9960_handle_t sensor, apds9960_gwtime_t time)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    uint8_t val;

    if (_i2c_read_byte(sens, APDS9960_GCONF2, &val) != ESP_OK) {
        return ESP_FAIL;
    }
    time &= 0x07;
    val &= 0xf8;
    val |= time;
    return _i2c_write_byte(sens, APDS9960_GCONF2, val);
}

esp_err_t apds9960_set_gesture_gain(apds9960_handle_t sensor, apds9960_ggain_t gain)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_gconf2_t.ggain = gain;
    return _i2c_write_byte(sens, APDS9960_GCONF2,
                           ((sens->_gconf2_t.ggain << 5) | (sens->_gconf2_t.gldrive << 3) | sens->_gconf2_t.gwtime));
}

esp_err_t apds9960_set_gesture_proximity_threshold(apds9960_handle_t sensor, uint8_t entthresh, uint8_t exitthresh)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;

    if (_i2c_write_byte(sens, APDS9960_GPENTH, entthresh) != ESP_OK) {
        return ESP_FAIL;
    }
    return _i2c_write_byte(sens, APDS9960_GEXTH, exitthresh);
}

esp_err_t apds9960_set_gesture_offset(apds9960_handle_t sensor, uint8_t offset_up, uint8_t offset_down,
                                      uint8_t offset_left, uint8_t offset_right)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    _i2c_write_byte(sens, APDS9960_GOFFSET_U, offset_up);
    _i2c_write_byte(sens, APDS9960_GOFFSET_D, offset_down);
    _i2c_write_byte(sens, APDS9960_GOFFSET_L, offset_left);
    _i2c_write_byte(sens, APDS9960_GOFFSET_R, offset_right);
    return ESP_OK;
}

uint8_t apds9960_read_gesture(apds9960_handle_t sensor)
{
    uint8_t             fifo_level;
    size_t              bytes_to_read;
    uint8_t             buf[256];
    TickType_t          start_tick = 0;
    uint8_t             gesture_received = 0;
    apds9960_dev_t     *dev = (apds9960_dev_t *)sensor;

    for (;;) {
        int up_down_diff    = 0;
        int left_right_diff = 0;
        gesture_received    = 0;

        if (!apds9960_gesture_valid(sensor)) {
            return 0;
        }

        vTaskDelay(pdMS_TO_TICKS(30));

        _i2c_read_byte(dev, APDS9960_GFLVL, &fifo_level);

        /* GFLVL reports datasets; each dataset is 4 bytes (U,D,L,R). */
        bytes_to_read = (size_t)fifo_level * 4U;
        if (bytes_to_read == 0) {
            continue;
        }
        if (bytes_to_read > sizeof(buf)) {
            bytes_to_read = sizeof(buf);
        }
        bytes_to_read -= (bytes_to_read % 4U);
        if (bytes_to_read == 0) {
            continue;
        }

        _i2c_read_bytes(dev, APDS9960_GFIFO_U, buf, bytes_to_read);

        if (abs((int)buf[0] - (int)buf[1]) > 13) {
            up_down_diff += (int)buf[0] - (int)buf[1];
        }

        if (abs((int)buf[2] - (int)buf[3]) > 13) {
            left_right_diff += (int)buf[2] - (int)buf[3];
        }

        if (up_down_diff != 0) {
            if (up_down_diff < 0) {
                if (dev->down_cnt > 0) {
                    gesture_received = APDS9960_UP;
                } else {
                    dev->up_cnt++;
                }
            } else {
                if (dev->up_cnt > 0) {
                    gesture_received = APDS9960_DOWN;
                } else {
                    dev->down_cnt++;
                }
            }
        }

        if (left_right_diff != 0) {
            if (left_right_diff < 0) {
                if (dev->right_cnt > 0) {
                    gesture_received = APDS9960_LEFT;
                } else {
                    dev->left_cnt++;
                }
            } else {
                if (dev->left_cnt > 0) {
                    gesture_received = APDS9960_RIGHT;
                } else {
                    dev->right_cnt++;
                }
            }
        }

        if (up_down_diff != 0 || left_right_diff != 0) {
            start_tick = xTaskGetTickCount();
        }

        if (gesture_received ||
            (xTaskGetTickCount() - start_tick > pdMS_TO_TICKS(300))) {
            apds9960_reset_counts(sensor);
            return gesture_received;
        }
    }
}

bool apds9960_gesture_valid(apds9960_handle_t sensor)
{
    uint8_t data;
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    _i2c_read_byte(sens, APDS9960_GSTATUS, &data);
    sens->_gstatus_t.gfov   = (data >> 1) & 0x01;
    sens->_gstatus_t.gvalid = data & 0x01;
    return sens->_gstatus_t.gvalid;
}

esp_err_t apds9960_set_gesture_pulse(apds9960_handle_t sensor, apds9960_gpulselen_t gpulseLen, uint8_t pulses)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    sens->_gpulse_t.gplen  = gpulseLen;
    sens->_gpulse_t.gpulse = pulses;
    return _i2c_write_byte(sens, APDS9960_GPULSE, (sens->_gpulse_t.gplen << 6) | sens->_gpulse_t.gpulse);
}

esp_err_t apds9960_set_gesture_enter_thresh(apds9960_handle_t sensor, uint8_t threshold)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return _i2c_write_byte(sens, APDS9960_GPENTH, threshold);
}

esp_err_t apds9960_set_gesture_exit_thresh(apds9960_handle_t sensor, uint8_t threshold)
{
    apds9960_dev_t *sens = (apds9960_dev_t *)sensor;
    return _i2c_write_byte(sens, APDS9960_GEXTH, threshold);
}

/* -------------------------------------------------------------------------- */
/* Create / Delete / Init                                                     */
/* -------------------------------------------------------------------------- */

apds9960_handle_t apds9960_create(i2c_master_bus_handle_t bus, uint8_t addr)
{
    apds9960_dev_t *d = (apds9960_dev_t *)calloc(1, sizeof(*d));
    if (!d) return NULL;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &d->i2c_dev);
    if (err != ESP_OK) {
        free(d);
        return NULL;
    }
    d->timeout = APDS9960_TIMEOUT_MS_DEFAULT;
    return (apds9960_handle_t)d;
}

esp_err_t apds9960_delete(apds9960_handle_t *sensor)
{
    if (sensor == NULL || *sensor == NULL) {
        return ESP_OK;
    }
    apds9960_dev_t *d = (apds9960_dev_t *)(*sensor);
    if (d->i2c_dev) {
        i2c_master_bus_rm_device(d->i2c_dev);
        d->i2c_dev = NULL;
    }
    free(d);
    *sensor = NULL;
    return ESP_OK;
}

esp_err_t apds9960_gesture_init(apds9960_handle_t sensor)
{
    apds9960_set_adc_integration_time(sensor, 10);
    apds9960_set_ambient_light_gain(sensor, APDS9960_AGAIN_4X);
    apds9960_enable_gesture_engine(sensor, false);
    apds9960_enable_proximity_engine(sensor, false);
    apds9960_enable_color_engine(sensor, false);
    apds9960_enable_color_interrupt(sensor, false);
    apds9960_enable_proximity_interrupt(sensor, false);
    apds9960_clear_interrupt(sensor);
    apds9960_enable(sensor, false);
    apds9960_enable(sensor, true);
    apds9960_set_gesture_dimensions(sensor, APDS9960_DIMENSIONS_ALL);
    apds9960_set_gesture_fifo_threshold(sensor, APDS9960_GFIFO_4);
    apds9960_set_gesture_gain(sensor, APDS9960_GGAIN_4X);
    apds9960_set_gesture_proximity_threshold(sensor, 50, 0);
    apds9960_reset_counts(sensor);
    apds9960_set_led_drive_boost(sensor, APDS9960_LEDDRIVE_100MA, APDS9960_LEDBOOST_100PCNT);
    apds9960_set_gesture_waittime(sensor, APDS9960_GWTIME_2_8MS);
    apds9960_set_gesture_pulse(sensor, APDS9960_GPULSELEN_32US, 8);
    apds9960_enable_proximity_engine(sensor, true);
    return apds9960_enable_gesture_engine(sensor, true);
}

float apds9960_calc_lux_from_rgb(uint16_t red, uint16_t green, uint16_t blue)
{
    /* Same coefficients your driver uses, but keep as float and avoid truncation. */
    float r = (float)red;
    float g = (float)green;
    float b = (float)blue;

    float illuminance = (-0.32466f * r) + (1.57837f * g) + (-0.73191f * b);
    if (illuminance < 0.0f) {
        illuminance = 0.0f;
    }
    return illuminance;
}


/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define APDS9960_BIT(n)         (1U << (n))

#define APDS9960_I2C_ADDRESS    0x39U
#define WRITE_BIT               I2C_MASTER_WRITE
#define READ_BIT                I2C_MASTER_READ
#define ACK_CHECK_EN            0x1U
#define ACK_CHECK_DIS           0x0U
#define ACK_VAL                 0x0U
#define NACK_VAL                0x1U
#define APDS9960_TIME_MULT      2.78f
#define ERROR                   0xFFU

#define APDS9960_UP             0x01U
#define APDS9960_DOWN           0x02U
#define APDS9960_LEFT           0x03U
#define APDS9960_RIGHT          0x04U

#define GESTURE_THRESHOLD_OUT   10U
#define GESTURE_SENSITIVITY_1   50U
#define GESTURE_SENSITIVITY_2   20U

#define APDS9960_WHO_AM_I_REG   0x92U
#define APDS9960_WHO_AM_I_VAL   0xABU

#define APDS9960_MODE_ENABLE    0x80U
#define APDS9960_GEN_BIT        APDS9960_BIT(6)
#define APDS9960_PIEN_BIT       APDS9960_BIT(5)
#define APDS9960_AIEN_BIT       APDS9960_BIT(4)
#define APDS9960_WEN_BIT        APDS9960_BIT(3)
#define APDS9960_PEN_BIT        APDS9960_BIT(2)
#define APDS9960_AEN_BIT        APDS9960_BIT(1)
#define APDS9960_PON_BIT        APDS9960_BIT(0)

#define APDS9960_GEN_MASK       0x40U
#define APDS9960_PIEN_MASK      0x20U
#define APDS9960_AIEN_MASK      0x10U
#define APDS9960_WEN_MASK       0x80U
#define APDS9960_PEN_MASK       0x04U
#define APDS9960_AEN_MASK       0x02U
#define APDS9960_PON_MASK       0x01U

#define APDS9960_ATIME          0x81U
#define APDS9960_WTIME          0x83U
#define APDS9960_AILTL          0x84U
#define APDS9960_AILTH          0x85U
#define APDS9960_AIHTL          0x86U
#define APDS9960_AIHTH          0x87U
#define APDS9960_PILT           0x89U
#define APDS9960_PIHT           0x8BU
#define APDS9960_PERS           0x8CU
#define APDS9960_CONFIG1        0x8DU
#define APDS9960_PPULSE         0x8EU
#define APDS9960_CONTROL        0x8FU
#define APDS9960_CONFIG2        0x90U
#define APDS9960_ID             0x92U
#define APDS9960_STATUS         0x93U
#define APDS9960_CDATAL         0x94U
#define APDS9960_CDATAH         0x95U
#define APDS9960_RDATAL         0x96U
#define APDS9960_RDATAH         0x97U
#define APDS9960_GDATAL         0x98U
#define APDS9960_GDATAH         0x99U
#define APDS9960_BDATAL         0x9AU
#define APDS9960_BDATAH         0x9BU
#define APDS9960_PDATA          0x9CU
#define APDS9960_POFFSET_UR     0x9DU
#define APDS9960_POFFSET_DL     0x9EU
#define APDS9960_CONFIG3        0x9FU
#define APDS9960_GPENTH         0xA0U
#define APDS9960_GEXTH          0xA1U
#define APDS9960_GCONF1         0xA2U
#define APDS9960_GCONF2         0xA3U
#define APDS9960_GOFFSET_U      0xA4U
#define APDS9960_GOFFSET_D      0xA5U
#define APDS9960_GPULSE         0xA6U
#define APDS9960_GOFFSET_L      0xA7U
#define APDS9960_GOFFSET_R      0xA9U
#define APDS9960_GCONF3         0xAAU
#define APDS9960_GCONF4         0xABU
#define APDS9960_GFLVL          0xAEU
#define APDS9960_GSTATUS        0xAFU
#define APDS9960_IFORCE         0xE4U
#define APDS9960_PICLEAR        0xE5U
#define APDS9960_CICLEAR        0xE6U
#define APDS9960_AICLEAR        0xE7U
#define APDS9960_GFIFO_U        0xFCU
#define APDS9960_GFIFO_D        0xFDU
#define APDS9960_GFIFO_L        0xFEU
#define APDS9960_GFIFO_R        0xFFU

typedef enum {
    APDS9960_AGAIN_1X  = 0x00,
    APDS9960_AGAIN_4X  = 0x01,
    APDS9960_AGAIN_16X = 0x02,
    APDS9960_AGAIN_64X = 0x03
} apds9960_again_t;

typedef enum {
    APDS9960_PGAIN_1X = 0x00,
    APDS9960_PGAIN_2X = 0x01,
    APDS9960_PGAIN_4X = 0x02,
    APDS9960_PGAIN_8X = 0x03
} apds9960_pgain_t;

typedef enum {
    APDS9960_GGAIN_1X = 0x00,
    APDS9960_GGAIN_2X = 0x01,
    APDS9960_GGAIN_4X = 0x02,
    APDS9960_GGAIN_8X = 0x03
} apds9960_ggain_t;

typedef enum {
    APDS9960_PPULSELEN_4US  = 0x00,
    APDS9960_PPULSELEN_8US  = 0x01,
    APDS9960_PPULSELEN_16US = 0x02,
    APDS9960_PPULSELEN_32US = 0x03
} apds9960_ppulse_len_t;

typedef enum {
    APDS9960_LEDDRIVE_100MA = 0x00,
    APDS9960_LEDDRIVE_50MA  = 0x01,
    APDS9960_LEDDRIVE_25MA  = 0x02,
    APDS9960_LEDDRIVE_12MA  = 0x03
} apds9960_leddrive_t;

typedef enum {
    APDS9960_LEDBOOST_100PCNT = 0x00,
    APDS9960_LEDBOOST_150PCNT = 0x01,
    APDS9960_LEDBOOST_200PCNT = 0x02,
    APDS9960_LEDBOOST_300PCNT = 0x03
} apds9960_ledboost_t;

typedef enum {
    APDS9960_DIMENSIONS_ALL        = 0x00,
    APDS9960_DIMENSIONS_UP_DOWM    = 0x01,
    APGS9960_DIMENSIONS_LEFT_RIGHT = 0x02
} apds9960_dimensions_t;

typedef enum {
    APDS9960_GFIFO_1  = 0x00,
    APDS9960_GFIFO_4  = 0x01,
    APDS9960_GFIFO_8  = 0x02,
    APDS9960_GFIFO_16 = 0x03
} apds9960_gfifo_t;

typedef enum {
    APDS9960_GPULSELEN_4US  = 0x00,
    APDS9960_GPULSELEN_8US  = 0x01,
    APDS9960_GPULSELEN_16US = 0x02,
    APDS9960_GPULSELEN_32US = 0x03
} apds9960_gpulselen_t;

typedef enum {
    APDS9960_GWTIME_0MS    = 0,
    APDS9960_GWTIME_2_8MS  = 1,
    APDS9960_GWTIME_5_6MS  = 2,
    APDS9960_GWTIME_8_4MS  = 3,
    APDS9960_GWTIME_14_0MS = 4,
    APDS9960_GWTIME_22_4MS = 5,
    APDS9960_GWTIME_30_8MS = 6,
    APDS9960_GWTIME_39_2MS = 7
} apds9960_gwtime_t;

typedef enum {
    APDS9960_POWER             = 0,
    APDS9960_AMBIENT_LIGHT     = 1,
    APDS9960_PROXIMITY         = 2,
    APDS9960_WAIT              = 3,
    APDS9960_AMBIENT_LIGHT_INT = 4,
    APDS9960_PROXIMITY_INT     = 5,
    APDS9960_GESTURE           = 6,
    APDS9960_ALL               = 7
} apds9960_mode_t;

#define DEFAULT_ATIME           219U
#define DEFAULT_WTIME           246U
#define DEFAULT_PROX_PPULSE     0x87U
#define DEFAULT_GESTURE_PPULSE  0x89U
#define DEFAULT_POFFSET_UR      0U
#define DEFAULT_POFFSET_DL      0U
#define DEFAULT_CONFIG1         0x60U
#define DEFAULT_LDRIVE          APDS9960_LEDDRIVE_100MA
#define DEFAULT_PGAIN           APDS9960_PGAIN_4X
#define DEFAULT_AGAIN           APDS9960_AGAIN_4X
#define DEFAULT_PILT            0U
#define DEFAULT_PIHT            50U
#define DEFAULT_AILT            0xFFFFU
#define DEFAULT_AIHT            0U
#define DEFAULT_PERS            0x11U
#define DEFAULT_CONFIG2         0x01U
#define DEFAULT_CONFIG3         0U
#define DEFAULT_GPENTH          40U
#define DEFAULT_GEXTH           30U
#define DEFAULT_GCONF1          0x40U
#define DEFAULT_GGAIN           APDS9960_GGAIN_4X
#define DEFAULT_GLDRIVE         APDS9960_LEDDRIVE_100MA
#define DEFAULT_GWTIME          APDS9960_GWTIME_2_8MS
#define DEFAULT_GOFFSET         0U
#define DEFAULT_GPULSE          0xC9U
#define DEFAULT_GCONF3          0U
#define DEFAULT_GIEN            0U

typedef struct control {
    uint8_t again     : 2;
    uint8_t pgain     : 2;
    uint8_t leddrive  : 2;
} apds9960_control_t;

typedef struct pers {
    uint8_t apers : 4;
    uint8_t ppers : 4;
} apds9960_pers_t;

typedef struct config1 {
    uint8_t wlong : 1;
} apds9960_config1_t;

typedef struct config2 {
    uint8_t led_boost : 2;
    uint8_t cpsien    : 1;
    uint8_t psien     : 1;
} apds9960_config2_t;

typedef struct config3 {
    uint8_t pmask_r : 1;
    uint8_t pmask_l : 1;
    uint8_t pmask_d : 1;
    uint8_t pmask_u : 1;
    uint8_t sai     : 1;
    uint8_t pcmp    : 1;
} apds9960_config3_t;

typedef struct gconf1 {
    uint8_t gexpers : 2;
    uint8_t gexmsk  : 4;
    uint8_t gfifoth : 2;
} apds9960_gconf1_t;

typedef struct gconf2 {
    uint8_t gwtime  : 3;
    uint8_t gldrive : 2;
    uint8_t ggain   : 2;
} apds9960_gconf2_t;

typedef struct gconf3 {
    uint8_t gdims : 2;
} apds9960_gconf3_t;

typedef struct gconf4 {
    uint8_t gmode : 1;
    uint8_t gien  : 2;
} apds9960_gconf4_t;

typedef struct enable {
    uint8_t pon  : 1;
    uint8_t aen  : 1;
    uint8_t pen  : 1;
    uint8_t wen  : 1;
    uint8_t aien : 1;
    uint8_t pien : 1;
    uint8_t gen  : 1;
} apds9960_enable_t;

typedef struct status {
    uint8_t avalid : 1;
    uint8_t pvalid : 1;
    uint8_t gint   : 1;
    uint8_t aint   : 1;
    uint8_t pint   : 1;
    uint8_t pgsat  : 1;
    uint8_t cpsat  : 1;
} apds9960_status_t;

typedef struct gstatus {
    uint8_t gvalid : 1;
    uint8_t gfov   : 1;
} apds9960_gstatus_t;

typedef struct ppulse {
    uint8_t ppulse : 6;
    uint8_t pplen  : 2;
} apds9960_propulse_t;

typedef struct gpulse {
    uint8_t gpulse : 6;
    uint8_t gplen  : 2;
} apds9960_gespulse_t;

typedef struct apds9960_dev_t *apds9960_handle_t;

#ifdef __cplusplus
extern "C" {
#endif


 apds9960_handle_t apds9960_create(i2c_master_bus_handle_t bus, uint8_t dev_addr);
esp_err_t apds9960_delete(apds9960_handle_t *sensor);

esp_err_t apds9960_set_timeout(apds9960_handle_t sensor, uint32_t tout_ms);

esp_err_t apds9960_gesture_init(apds9960_handle_t sensor);
esp_err_t apds9960_get_deviceid(apds9960_handle_t sensor, uint8_t *deviceid);

esp_err_t apds9960_set_mode(apds9960_handle_t sensor, apds9960_mode_t mode);
apds9960_mode_t apds9960_get_mode(apds9960_handle_t sensor);

esp_err_t apds9960_set_wait_time(apds9960_handle_t sensor, uint8_t time);
esp_err_t apds9960_set_adc_integration_time(apds9960_handle_t sensor, uint16_t iTimeMS);
float     apds9960_get_adc_integration_time(apds9960_handle_t sensor);

esp_err_t apds9960_set_ambient_light_gain(apds9960_handle_t sensor, apds9960_again_t aGain);
apds9960_again_t apds9960_get_ambient_light_gain(apds9960_handle_t sensor);

esp_err_t apds9960_set_led_drive_boost(apds9960_handle_t sensor,
                                       apds9960_leddrive_t drive, apds9960_ledboost_t boost);

esp_err_t apds9960_enable_proximity_engine(apds9960_handle_t sensor, bool en);
esp_err_t apds9960_set_proximity_gain(apds9960_handle_t sensor, apds9960_pgain_t pGain);
apds9960_pgain_t apds9960_get_proximity_gain(apds9960_handle_t sensor);
esp_err_t apds9960_set_proximity_pulse(apds9960_handle_t sensor,
                                       apds9960_ppulse_len_t pLen, uint8_t pulses);
esp_err_t apds9960_enable_proximity_interrupt(apds9960_handle_t sensor, bool en);
bool      apds9960_get_proximity_interrupt(apds9960_handle_t sensor);
uint8_t   apds9960_read_proximity(apds9960_handle_t sensor);
esp_err_t apds9960_set_proximity_interrupt_threshold(apds9960_handle_t sensor,
                                                     uint8_t low, uint8_t high, uint8_t persistence);

esp_err_t apds9960_enable_gesture_engine(apds9960_handle_t sensor, bool en);
bool      apds9960_gesture_valid(apds9960_handle_t sensor);
esp_err_t apds9960_set_gesture_dimensions(apds9960_handle_t sensor, uint8_t dims);
esp_err_t apds9960_set_gesture_fifo_threshold(apds9960_handle_t sensor, uint8_t thresh);
esp_err_t apds9960_set_gesture_gain(apds9960_handle_t sensor, apds9960_ggain_t gGain);
esp_err_t apds9960_set_gesture_proximity_threshold(apds9960_handle_t sensor,
                                                   uint8_t entthresh, uint8_t exitthresh);
esp_err_t apds9960_set_gesture_offset(apds9960_handle_t sensor,
                                      uint8_t offset_up, uint8_t offset_down, uint8_t offset_left,
                                      uint8_t offset_right);
uint8_t   apds9960_read_gesture(apds9960_handle_t sensor);
void      apds9960_reset_counts(apds9960_handle_t sensor);
esp_err_t apds9960_set_gesture_pulse(apds9960_handle_t sensor,
                                     apds9960_gpulselen_t gpulseLen, uint8_t pulses);
esp_err_t apds9960_enable_gesture_interrupt(apds9960_handle_t sensor, bool en);
esp_err_t apds9960_set_gesture_waittime(apds9960_handle_t sensor, apds9960_gwtime_t time);

esp_err_t apds9960_enable_color_engine(apds9960_handle_t sensor, bool en);
bool      apds9960_color_data_ready(apds9960_handle_t sensor);
esp_err_t apds9960_get_color_data(apds9960_handle_t sensor, uint16_t *r,
                                  uint16_t *g, uint16_t *b, uint16_t *c);
uint16_t  apds9960_calculate_color_temperature(apds9960_handle_t sensor,
                                               uint16_t r, uint16_t g, uint16_t b);
uint16_t  apds9960_calculate_lux(apds9960_handle_t sensor, uint16_t r,
                                 uint16_t g, uint16_t b);

esp_err_t apds9960_enable_color_interrupt(apds9960_handle_t sensor, bool en);
esp_err_t apds9960_set_int_limits(apds9960_handle_t sensor, uint16_t l, uint16_t h);
esp_err_t apds9960_clear_interrupt(apds9960_handle_t sensor);
esp_err_t apds9960_enable(apds9960_handle_t sensor, bool en);
esp_err_t apds9960_set_light_intlow_threshold(apds9960_handle_t sensor, uint16_t threshold);
esp_err_t apds9960_set_light_inthigh_threshold(apds9960_handle_t sensor, uint16_t threshold);
float apds9960_calc_lux_from_rgb(uint16_t red, uint16_t green, uint16_t blue);

#ifdef __cplusplus
}
#endif


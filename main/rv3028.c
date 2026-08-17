#include "rv3028.h"
#include "board_config.h"
#include "board_i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define RV3028_REG_SECONDS    0x00
#define RV3028_REG_MINUTES    0x01
#define RV3028_REG_STATUS     0x0E
#define RV3028_REG_CONTROL2   0x10
#define RV3028_REG_EE_BACKUP  0x37
#define RV3028_STATUS_PORF    (1U << 0)
#define RV3028_CONTROL2_12_24 (1U << 1)
#define RV3028_BACKUP_FEDE     (1U << 4)
#define RV3028_BACKUP_BSM_LSM  (3U << 2)
#define RV3028_IO_TIMEOUT_MS  100
#define RV3028_LOG_PERIOD_MS  1000
#define RV3028_INVALID_WARN_PERIOD_MS 60000

static const char *TAG = "RTC";
static i2c_master_dev_handle_t s_dev;
static SemaphoreHandle_t s_lock;
static TickType_t s_last_log;
static TickType_t s_last_invalid_warning;
static uint8_t s_last_status;
static bool s_last_calendar_valid;
static bool s_invalid_warning_active;

static uint8_t bin_to_bcd(uint8_t value)
{
    return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

static uint8_t bcd_to_bin(uint8_t value)
{
    return (uint8_t)(((value >> 4) * 10U) + (value & 0x0FU));
}

static bool is_leap_year(uint16_t year)
{
    return (year % 4U == 0U && year % 100U != 0U) || year % 400U == 0U;
}

static uint8_t days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t days[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };
    if (month < 1 || month > 12) return 0;
    if (month == 2 && is_leap_year(year)) return 29;
    return days[month - 1];
}

static bool datetime_is_valid(const rv3028_datetime_t *dt)
{
    return dt && dt->year >= 2000 && dt->year <= 2099 &&
           dt->month >= 1 && dt->month <= 12 &&
           dt->date >= 1 && dt->date <= days_in_month(dt->year, dt->month) &&
           dt->hour <= 23 && dt->minute <= 59 && dt->second <= 59;
}

/* Returns 0=Sunday through 6=Saturday, matching the RV-3028 weekday field. */
static uint8_t calculate_weekday(uint16_t year, uint8_t month, uint8_t date)
{
    static const uint8_t offset[] = {
        0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4
    };
    uint16_t y = year;
    if (month < 3) --y;
    return (uint8_t)((y + y / 4U - y / 100U + y / 400U +
                      offset[month - 1] + date) % 7U);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len,
                                       RV3028_IO_TIMEOUT_MS);
}

static esp_err_t write_regs(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buffer[8];
    if (len > sizeof(buffer) - 1U) return ESP_ERR_INVALID_SIZE;
    buffer[0] = reg;
    for (size_t i = 0; i < len; ++i) buffer[i + 1] = data[i];
    return i2c_master_transmit(s_dev, buffer, len + 1, RV3028_IO_TIMEOUT_MS);
}

static esp_err_t get_time_locked(rv3028_datetime_t *dt, bool *valid)
{
    uint8_t regs[7];
    uint8_t status;
    esp_err_t err = read_regs(RV3028_REG_SECONDS, regs, sizeof(regs));
    if (err != ESP_OK) return err;
    err = read_regs(RV3028_REG_STATUS, &status, 1);
    if (err != ESP_OK) return err;

    dt->second = bcd_to_bin(regs[0] & 0x7FU);
    dt->minute = bcd_to_bin(regs[1] & 0x7FU);
    dt->hour = bcd_to_bin(regs[2] & 0x3FU);
    dt->weekday = regs[3] & 0x07U;
    dt->date = bcd_to_bin(regs[4] & 0x3FU);
    dt->month = bcd_to_bin(regs[5] & 0x1FU);
    dt->year = 2000U + bcd_to_bin(regs[6]);

    bool values_valid = datetime_is_valid(dt) && dt->weekday <= 6;
    s_last_status = status;
    s_last_calendar_valid = values_valid;
    if (valid) *valid = values_valid && !(status & RV3028_STATUS_PORF);
    return ESP_OK;
}

static esp_err_t set_time_locked(const rv3028_datetime_t *dt)
{
    if (!datetime_is_valid(dt)) return ESP_ERR_INVALID_ARG;

    uint8_t calendar[6] = {
        bin_to_bcd(dt->minute),
        bin_to_bcd(dt->hour),
        calculate_weekday(dt->year, dt->month, dt->date),
        bin_to_bcd(dt->date),
        bin_to_bcd(dt->month),
        bin_to_bcd((uint8_t)(dt->year - 2000U)),
    };
    esp_err_t err = write_regs(RV3028_REG_MINUTES, calendar,
                               sizeof(calendar));
    if (err != ESP_OK) return err;

    /* Writing Seconds last synchronizes the RV-3028 1 Hz prescaler. */
    uint8_t seconds = bin_to_bcd(dt->second);
    err = write_regs(RV3028_REG_SECONDS, &seconds, 1);
    if (err != ESP_OK) return err;

    uint8_t status;
    err = read_regs(RV3028_REG_STATUS, &status, 1);
    if (err != ESP_OK) return err;
    status &= (uint8_t)~RV3028_STATUS_PORF;
    err = write_regs(RV3028_REG_STATUS, &status, 1);
    if (err != ESP_OK) return err;

    /* Verify that calibration really cleared the latched power-loss flag. */
    err = read_regs(RV3028_REG_STATUS, &status, 1);
    if (err != ESP_OK) return err;
    return (status & RV3028_STATUS_PORF) ? ESP_ERR_INVALID_STATE : ESP_OK;
}

static void log_time(void)
{
    static const char *weekday_name[] = {
        "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
    };
    rv3028_datetime_t dt;
    bool valid;
    esp_err_t err = rv3028_get_time(&dt, &valid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RV-3028 read failed: %s", esp_err_to_name(err));
        return;
    }
    if (!valid) {
        TickType_t now = xTaskGetTickCount();
        if (!s_invalid_warning_active ||
            now - s_last_invalid_warning >=
                pdMS_TO_TICKS(RV3028_INVALID_WARN_PERIOD_MS)) {
            if (s_last_status & RV3028_STATUS_PORF) {
                ESP_LOGW(TAG,
                         "RV-3028 PORF is set; synchronize time once to "
                         "clear the latched power-loss flag");
            } else if (!s_last_calendar_valid) {
                ESP_LOGW(TAG,
                         "RV-3028 calendar registers are invalid; "
                         "synchronize time once");
            }
            s_last_invalid_warning = now;
            s_invalid_warning_active = true;
        }
        return;
    }
    s_invalid_warning_active = false;
    ESP_LOGI(TAG, "%04u-%02u-%02u %s %02u:%02u:%02u",
             dt.year, dt.month, dt.date, weekday_name[dt.weekday],
             dt.hour, dt.minute, dt.second);
}

esp_err_t rv3028_init(void)
{
    esp_err_t err = board_i2c_init();
    if (err != ESP_OK) return err;

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BOARD_RV3028_ADDR,
        .scl_speed_hz = BOARD_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(board_i2c_get_bus(), &dev_cfg, &s_dev);
    if (err != ESP_OK) return err;

    uint8_t control2;
    err = read_regs(RV3028_REG_CONTROL2, &control2, 1);
    if (err != ESP_OK) return err;
    if (control2 & RV3028_CONTROL2_12_24) {
        control2 &= (uint8_t)~RV3028_CONTROL2_12_24;
        err = write_regs(RV3028_REG_CONTROL2, &control2, 1);
        if (err != ESP_OK) return err;
    }

    /*
     * A non-rechargeable backup battery is fitted: enable Level Switching
     * Mode and Fast Edge Detection in the active RAM mirror. Keep trickle
     * charging disabled. The setting remains active while VBACKUP keeps the
     * RTC powered and is restored by this initialization after a full loss.
     */
    uint8_t backup;
    err = read_regs(RV3028_REG_EE_BACKUP, &backup, 1);
    if (err != ESP_OK) return err;
    uint8_t backup_lsm = (backup & 0x80U) |
                         RV3028_BACKUP_FEDE |
                         RV3028_BACKUP_BSM_LSM;
    if (backup != backup_lsm) {
        err = write_regs(RV3028_REG_EE_BACKUP, &backup_lsm, 1);
        if (err != ESP_OK) return err;
    }

    ESP_LOGI(TAG,
             "RV-3028 initialized at 0x%02X "
             "(24-hour, LSM battery backup, trickle charge off)",
             BOARD_RV3028_ADDR);
    s_last_log = xTaskGetTickCount();
    log_time();
    return ESP_OK;
}

esp_err_t rv3028_get_time(rv3028_datetime_t *datetime, bool *valid)
{
    if (!datetime || !s_lock) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = get_time_locked(datetime, valid);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t rv3028_set_time(const rv3028_datetime_t *datetime)
{
    if (!datetime || !s_lock) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = set_time_locked(datetime);
    xSemaphoreGive(s_lock);
    if (err == ESP_OK) {
        s_invalid_warning_active = false;
        ESP_LOGI(TAG, "Time set to %04u-%02u-%02u %02u:%02u:%02u",
                 datetime->year, datetime->month, datetime->date,
                 datetime->hour, datetime->minute, datetime->second);
    }
    return err;
}

void rv3028_poll_and_log(void)
{
    TickType_t now = xTaskGetTickCount();
    if (now - s_last_log < pdMS_TO_TICKS(RV3028_LOG_PERIOD_MS)) return;
    s_last_log = now;
    log_time();
}

bool rv3028_modbus_read_register(uint16_t address, uint16_t *value)
{
    if (!value || address < RV3028_MODBUS_REG_YEAR ||
        address >= RV3028_MODBUS_REG_YEAR + RV3028_MODBUS_REG_COUNT) {
        return false;
    }

    rv3028_datetime_t dt;
    if (rv3028_get_time(&dt, NULL) != ESP_OK) return false;
    switch (address) {
    case RV3028_MODBUS_REG_YEAR:   *value = dt.year; break;
    case RV3028_MODBUS_REG_MONTH:  *value = dt.month; break;
    case RV3028_MODBUS_REG_DATE:   *value = dt.date; break;
    case RV3028_MODBUS_REG_HOUR:   *value = dt.hour; break;
    case RV3028_MODBUS_REG_MINUTE: *value = dt.minute; break;
    case RV3028_MODBUS_REG_SECOND: *value = dt.second; break;
    default: return false;
    }
    return true;
}

bool rv3028_modbus_write_registers(uint16_t start, const uint16_t *values,
                                   size_t count)
{
    if (!values || !count || start < RV3028_MODBUS_REG_YEAR ||
        start + count > RV3028_MODBUS_REG_YEAR + RV3028_MODBUS_REG_COUNT) {
        return false;
    }

    rv3028_datetime_t dt;
    if (rv3028_get_time(&dt, NULL) != ESP_OK) return false;
    for (size_t i = 0; i < count; ++i) {
        switch (start + i) {
        case RV3028_MODBUS_REG_YEAR:   dt.year = values[i]; break;
        case RV3028_MODBUS_REG_MONTH:  dt.month = values[i]; break;
        case RV3028_MODBUS_REG_DATE:   dt.date = values[i]; break;
        case RV3028_MODBUS_REG_HOUR:   dt.hour = values[i]; break;
        case RV3028_MODBUS_REG_MINUTE: dt.minute = values[i]; break;
        case RV3028_MODBUS_REG_SECOND: dt.second = values[i]; break;
        default: return false;
        }
    }
    return rv3028_set_time(&dt) == ESP_OK;
}

bool rv3028_modbus_write_register(uint16_t address, uint16_t value)
{
    return rv3028_modbus_write_registers(address, &value, 1);
}

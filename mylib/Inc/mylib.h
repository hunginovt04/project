#ifndef MYLIB_H
#define MYLIB_H

#include "main.h"
#include "stdbool.h"
#include "sht3x.h"
#include "i2c_lcd.h"

// Cấu trúc quản lý các ngoại vi và thông số hệ thống
typedef struct {
    sht3x_handle_t *hsht3x;
    I2C_LCD_HandleTypeDef *hlcd1;
    UART_HandleTypeDef *huart1;
    int period;
} hTask;

// Cấu trúc lưu trữ dữ liệu cảm biến mới nhất
typedef struct {
    float temp;
    float humid;
    bool is_valid;
} SHT3_Data_t;

// Enum và Struct cho hiển thị và UART (Giữ nguyên logic của bạn)
typedef enum { SET_P, HUMID, TEMP } type_value_sht3x;
typedef struct { type_value_sht3x type; float value; } data_sht3x;

typedef enum { SET_PERIOD, ERROR_SHT3x, ERROR_MUTEX, HUMID_UART, TEMP_UART, LOG_UART } type_vale_uart;
typedef struct { type_vale_uart type; float value; } data_uart;

// --- CÁC HÀM TÁCH BIỆT ---

// Nhóm 1: Đọc và nhận dữ liệu
void update_sensor_data(void);   // Đọc từ phần cứng vào struct
void take_uart(void);           // Xử lý lệnh từ máy tính

// Nhóm 2: Xử lý hiển thị (Không lồng ghép)
void display_temp(void);
void display_humid(void);
void display_period(void);

// Nhóm 3: Truyền thông UART (Không lồng ghép)
void send_temp_uart(void);
void send_humid_uart(void);
void send_log_uart(void);

// Hàm bổ trợ (Low level)
void display_to_lcd(data_sht3x val);
void give_uart(data_uart val);
typedef void (*TaskFunction_t)(void);


#endif

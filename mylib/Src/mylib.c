#include "mylib.h"
#include "stdio.h"
#include "string.h"

extern hTask program;
extern uint8_t rx_buffer[15];

// Biến nội bộ quản lý trạng thái
static uint16_t rx_size = 0;
static volatile uint8_t is_data_ready = 0;
static SHT3_Data_t sensor_storage = {0.0f, 0.0f, false};
static data_uart current_log = {LOG_UART, 0};
static bool has_new_log = false;

// Callback ngắt UART
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        rx_size = Size;
        is_data_ready = 1;
    }
}

// --- TÁC VỤ 1: CẬP NHẬT DỮ LIỆU CẢM BIẾN ---
void update_sensor_data(void) {
    if(sht3x_read_temperature_and_humidity(program.hsht3x, &sensor_storage.temp, &sensor_storage.humid)) {
        sensor_storage.is_valid = true;
    } else {
        sensor_storage.is_valid = false;
        data_uart err = {ERROR_SHT3x, 0};
        give_uart(err);
    }
}

// --- TÁC VỤ 2: HIỂN THỊ LCD (Tách biệt) ---
void display_temp(void) {
    if (!sensor_storage.is_valid) return;
    data_sht3x d = {TEMP, sensor_storage.temp};
    display_to_lcd(d);
}

void display_humid(void) {
    if (!sensor_storage.is_valid) return;
    data_sht3x d = {HUMID, sensor_storage.humid};
    display_to_lcd(d);
}

void display_period(void) {
    data_sht3x d = {SET_P, (float)program.period};
    display_to_lcd(d);
}

// --- TÁC VỤ 3: GỬI UART (Tách biệt) ---
void send_temp_uart(void) {
    if (!sensor_storage.is_valid) return;
    data_uart d = {TEMP_UART, sensor_storage.temp};
    give_uart(d);
}

void send_humid_uart(void) {
    if (!sensor_storage.is_valid) return;
    data_uart d = {HUMID_UART, sensor_storage.humid};
    give_uart(d);
}

void send_log_uart(void) {
    if (!has_new_log) return;
    give_uart(current_log);
    has_new_log = false;
}

// --- TÁC VỤ 4: XỬ LÝ LỆNH NHẬN ---
void take_uart(void) {
    if (!is_data_ready) return;

    char data_p[16] = {0};
    int p_val;

    memcpy(data_p, rx_buffer, (rx_size > 15) ? 15 : rx_size);

    if (sscanf(data_p, "SET:%d", &p_val) == 1) {
        if (p_val < 300) {
            current_log.value = 1; // Quá bé
        } else {
            program.period = p_val;
            current_log.value = 3; // OK
            display_period();      // Cập nhật LCD ngay
        }
    } else {
        current_log.value = 2;     // Sai cú pháp
    }

    has_new_log = true;
    is_data_ready = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(program.huart1, rx_buffer, 15);
}

// --- CÁC HÀM GIAO TIẾP THẤP (Low Level) ---
void display_to_lcd(data_sht3x val) {
    char str[16];
    int n = (int)val.value;
    int t = (int)((val.value - n) * 10);

    if (val.type == HUMID) {
        snprintf(str, sizeof(str), "H:%d.%d%% ", n, t);
        lcd_gotoxy(program.hlcd1, 0, 0);
    } else if (val.type == TEMP) {
        snprintf(str, sizeof(str), "T:%d.%d", n, t);
        lcd_gotoxy(program.hlcd1, 8, 0);
    } else {
        snprintf(str, sizeof(str), "P:%dms    ", n);
        lcd_gotoxy(program.hlcd1, 0, 1);
    }
    lcd_puts(program.hlcd1, str);
    if (val.type == TEMP) {
        lcd_putchar(program.hlcd1, 0xDF); lcd_putchar(program.hlcd1, 'C');
    }
}

void give_uart(data_uart val) {
    char msg[45];
    int n = (int)val.value;
    int t = (int)((val.value - n) * 10);

    switch (val.type) {
        case TEMP_UART:  snprintf(msg, sizeof(msg), "Temp: %d.%d C\n", n, t); break;
        case HUMID_UART: snprintf(msg, sizeof(msg), "Humid: %d.%d%%\n", n, t); break;
        case ERROR_SHT3x:snprintf(msg, sizeof(msg), "Sensor Error!\n"); break;
        case LOG_UART:
            if (val.value == 1)      snprintf(msg, sizeof(msg), "P too small (>300)\n");
            else if (val.value == 2) snprintf(msg, sizeof(msg), "Wrong Cmd\n");
            else                     snprintf(msg, sizeof(msg), "Set P OK: %d\n", program.period);
            break;
        default: return;
    }
    HAL_UART_Transmit(program.huart1, (uint8_t *)msg, strlen(msg), 100);
}

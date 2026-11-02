#ifndef NEXTION_SIMPLE_H
#define NEXTION_SIMPLE_H
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

// Definisi konstanta
#define NEXTION_CMD_END     "\xFF\xFF\xFF"
#define NEXTION_BUFFER_SIZE 100
#define NEXTION_TIMEOUT     1000

// Struktur untuk menyimpan data yang diterima
typedef struct {
    uint8_t buffer[NEXTION_BUFFER_SIZE];
    uint16_t length;
    bool data_ready;
} NextionData_t;

// Variabel global
extern UART_HandleTypeDef huart3; // Sesuaikan dengan UART yang digunakan
extern NextionData_t nextion_data;
NextionData_t nextion_data = {0};
static uint8_t rx_buffer[1];

// Fungsi dasar
void Nextion_Init(void);
void Nextion_SendCommand(const char *cmd);
void Nextion_SendText(const char *object, const char *text);
void Nextion_SendNumber(const char *object, int32_t number);
void Nextion_SetProgressBar(const char *object, uint8_t value);
void Nextion_SetSlider(const char *object, uint8_t value);
void Nextion_ChangePage(uint8_t page_id);
void Nextion_SetBacklight(uint8_t brightness);

// Fungsi untuk membaca data
bool Nextion_Available(void);
void Nextion_ProcessData(void);
uint8_t* Nextion_GetBuffer(void);
uint16_t Nextion_GetLength(void);
void Nextion_ClearBuffer(void);

// Callback untuk interrupt UART
void Nextion_UART_RxCallback(void);


void Nextion_Init(void) {
    // Reset buffer
    memset(nextion_data.buffer, 0, NEXTION_BUFFER_SIZE);
    nextion_data.length = 0;
    nextion_data.data_ready = false;

    // Mulai interrupt receive
    HAL_UART_Receive_IT(&huart3, rx_buffer, 1);

    // Kirim perintah reset (opsional)
    HAL_Delay(100);
    Nextion_SendCommand("rest");
    HAL_Delay(500);
}

void Nextion_SendCommand(const char *cmd) {
    char command[NEXTION_BUFFER_SIZE];
    snprintf(command, sizeof(command), "%s%s", cmd, NEXTION_CMD_END);
    HAL_UART_Transmit(&huart3, (uint8_t*)command, strlen(command), NEXTION_TIMEOUT);
}

void Nextion_SendText(const char *object, const char *text) {
    char command[NEXTION_BUFFER_SIZE];
    snprintf(command, sizeof(command), "%s.txt=\"%s\"", object, text);
    Nextion_SendCommand(command);
}

void Nextion_SendNumber(const char *object, int32_t number) {
    char command[NEXTION_BUFFER_SIZE];
    snprintf(command, sizeof(command), "%s.val=%ld", object, number);
    Nextion_SendCommand(command);
}

void Nextion_SetProgressBar(const char *object, uint8_t value) {
    if (value > 100) value = 100;
    char command[NEXTION_BUFFER_SIZE];
    snprintf(command, sizeof(command), "%s.val=%d", object, value);
    Nextion_SendCommand(command);
}

void Nextion_SetSlider(const char *object, uint8_t value) {
    char command[NEXTION_BUFFER_SIZE];
    snprintf(command, sizeof(command), "%s.val=%d", object, value);
    Nextion_SendCommand(command);
}

void Nextion_SetPicture(const char *object, uint8_t pic_id) {
    char command[50];
    snprintf(command, sizeof(command), "%s.pic=%d", object, pic_id);
    Nextion_SendCommand(command);
}

void Nextion_ChangePage(uint8_t page_id) {
    char command[NEXTION_BUFFER_SIZE];
    snprintf(command, sizeof(command), "page %d", page_id);
    Nextion_SendCommand(command);
}

void Nextion_SetBacklight(uint8_t brightness) {
    if (brightness > 100) brightness = 100;
    char command[NEXTION_BUFFER_SIZE];
    snprintf(command, sizeof(command), "dim=%d", brightness);
    Nextion_SendCommand(command);
}

bool Nextion_Available(void) {
    return nextion_data.data_ready;
}

void Nextion_ProcessData(void) {
    if (!nextion_data.data_ready) return;

    // Proses data sesuai kebutuhan aplikasi
    // Contoh: periksa apakah data berisi touch event atau return data

    // Reset flag setelah diproses
    nextion_data.data_ready = false;
    nextion_data.length = 0;
}

uint8_t* Nextion_GetBuffer(void) {
    return nextion_data.buffer;
}

uint16_t Nextion_GetLength(void) {
    return nextion_data.length;
}

void Nextion_ClearBuffer(void) {
    memset(nextion_data.buffer, 0, NEXTION_BUFFER_SIZE);
    nextion_data.length = 0;
    nextion_data.data_ready = false;
}


const char* Nextion_GetString(void) {
    if (nextion_data.data_ready) {
        // Hapus 0xFF 0xFF 0xFF dari akhir
        if (nextion_data.length >= 3) {
            nextion_data.buffer[nextion_data.length - 3] = '\0';
        }
        return (char*)nextion_data.buffer;
    }
    return NULL;
}

void Nextion_UART_RxCallback(void) {
    // Simpan data yang diterima
    if (nextion_data.length < NEXTION_BUFFER_SIZE - 1) {
        nextion_data.buffer[nextion_data.length] = rx_buffer[0];
        nextion_data.length++;

        // Periksa apakah sudah menerima end command (0xFF 0xFF 0xFF)
        if (nextion_data.length >= 3) {
            if (nextion_data.buffer[nextion_data.length-3] == 0xFF &&
                nextion_data.buffer[nextion_data.length-2] == 0xFF &&
                nextion_data.buffer[nextion_data.length-1] == 0xFF) {
                nextion_data.data_ready = true;
            }
        }
    }

    // Restart interrupt receive
    HAL_UART_Receive_IT(&huart3, rx_buffer, 1);
}




#endif // NEXTION_SIMPLE_H

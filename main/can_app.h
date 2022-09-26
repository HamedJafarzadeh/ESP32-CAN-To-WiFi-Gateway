#ifndef CAN_APP_H
#define CAN_APP_H

#include "driver/can.h"
#include "can_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "string.h"
#include "tcp_app.h"

#ifdef __cplusplus
extern "C"
{
#endif

    struct CANPacket
    {
        uint32_t ID;
        uint8_t IDE;
        uint8_t EXT;
        uint8_t RTR;
        uint8_t DLC;
        uint8_t data[8];
    };
    extern QueueHandle_t CANTXMessageQueue;
    void can_app_init(void);
    void can_app_task(void *pvParameters);
#ifdef __cplusplus
}
#endif

#endif
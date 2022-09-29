#include "driver/can.h"
#include "can_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "string.h"
#include "tcp_app.h"

#define TAG "CAN_App"

// typedef struct
// {
//     uint32_t id;
//     uint8_t ext;
//     uint8_t length;
//     uint8_t data[8];
// } CANMessage;

QueueHandle_t CANRXMessageQueue;
QueueHandle_t CANTXMessageQueue;
#define transmitBufferSize 23
uint8_t transmitBuffer[transmitBufferSize];

union
{
    uint8_t preamble_1;
    uint8_t preamble_2;
    uint8_t type; // 0x01 = CAN Message    |    0x02 = System Message
    uint32_t id;
    uint8_t ext;
    uint8_t length;
    uint8_t data[8];
    uint8_t tail1;
    uint8_t tail2;

} gatewayTCPMessage;

// struct CANPacket
// {
//     uint32_t ID;
//     uint8_t IDE;
//     uint8_t EXT;
//     uint8_t RTR;
//     uint8_t DLC;
//     uint8_t data[8];
// };

void can_init()
{

    transmitBuffer[0] = '<';
    transmitBuffer[1] = '{';
    transmitBuffer[2] = 0x01;  // Type
    transmitBuffer[3] = 0x00;  // ID LSB
    transmitBuffer[4] = 0x00;  // ID
    transmitBuffer[5] = 0x00;  // ID
    transmitBuffer[6] = 0x00;  // ID MSB
    transmitBuffer[7] = 0x00;  // EXT
    transmitBuffer[8] = 0x00;  // Length
    transmitBuffer[9] = 0x00;  // Data 0
    transmitBuffer[10] = 0x00; // Data 1
    transmitBuffer[11] = 0x00; // Data 2
    transmitBuffer[12] = 0x00; // Data 3
    transmitBuffer[13] = 0x00; // Data 4
    transmitBuffer[14] = 0x00; // Data 5
    transmitBuffer[15] = 0x00; // Data 6
    transmitBuffer[16] = 0x00; // Data 7
    transmitBuffer[17] = 0x00; // TimeStamp LSB
    transmitBuffer[18] = 0x00; // TimeStamp
    transmitBuffer[19] = 0x00; // TimeStamp
    transmitBuffer[20] = 0x00; // TimeStamp MSB
    transmitBuffer[21] = '}';
    transmitBuffer[22] = '>';

    // gatewayTCPMessage.preamble_1 = '<';
    // gatewayTCPMessage.preamble_2 = '{';
    // gatewayTCPMessage.type = 0x01;
    // gatewayTCPMessage.tail1 = '}';
    // gatewayTCPMessage.tail2 = '>';
    CANRXMessageQueue = xQueueCreate(255, sizeof(transmitBuffer));
    CANTXMessageQueue = xQueueCreate(255, sizeof(struct CANPacket));
    // ~~~~~~~~~~~~~~~ CAN ~~~~~~~~~~~~~~~~~
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_27, GPIO_NUM_14, CAN_MODE_NORMAL);
    can_timing_config_t t_config = CAN_TIMING_CONFIG_125KBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
    // Install CAN driver

    // CAN FD 4 Specific Config
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 0);

    if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        printf("CAN Driver installed\n");
    }
    else
    {
        printf("Failed to install CAN driver\n");
        return;
    }
    // Start CAN driver
    if (can_start() == ESP_OK)
    {
        printf("Driver started\n");
    }
    else
    {
        printf("Failed to start driver\n");
        return;
    }
}

void can_app_receive(void *pvParameters)
{
    int i = 0;
    while (1)
    {
        twai_message_t rcvMsg;
        esp_err_t ret = can_receive(&rcvMsg, portMAX_DELAY);
        // gatewayTCPMessage.id = rcvMsg.identifier;
        // gatewayTCPMessage.ext = 0;
        // gatewayTCPMessage.length = rcvMsg.data_length_code;
        // memcpy(gatewayTCPMessage.data, rcvMsg.data, 8);
        // for (int i = 0; i < 20; i++)
        // {
        //     printf("%d ", ((uint8_t *)&gatewayTCPMessage)[i]);
        // }
        if (ret == ESP_OK)
        {
            transmitBuffer[2] = 0x01;                             // Type
            transmitBuffer[3] = rcvMsg.identifier & 0xFF;         // ID LSB
            transmitBuffer[4] = (rcvMsg.identifier >> 8) & 0xFF;  // ID
            transmitBuffer[5] = (rcvMsg.identifier >> 16) & 0xFF; // ID
            transmitBuffer[6] = (rcvMsg.identifier >> 24) & 0xFF; // ID MSB
            transmitBuffer[7] = 0x00;                             // EXT
            transmitBuffer[8] = rcvMsg.data_length_code;          // Length
            transmitBuffer[9] = rcvMsg.data[0];                   // Data 0
            transmitBuffer[10] = rcvMsg.data[1];                  // Data 1
            transmitBuffer[11] = rcvMsg.data[2];                  // Data 2
            transmitBuffer[12] = rcvMsg.data[3];                  // Data 3
            transmitBuffer[13] = rcvMsg.data[4];                  // Data 4
            transmitBuffer[14] = rcvMsg.data[5];                  // Data 5
            transmitBuffer[15] = rcvMsg.data[6];                  // Data 6
            transmitBuffer[16] = rcvMsg.data[7];                  // Data 7
            uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
            transmitBuffer[17] = timestamp & 0xFF;         // Timestamp LSB
            transmitBuffer[18] = (timestamp >> 8) & 0xFF;  // Timestamp
            transmitBuffer[19] = (timestamp >> 16) & 0xFF; // Timestamp
            transmitBuffer[20] = (timestamp >> 24) & 0xFF; // Timestamp MSB
            xQueueSend(CANRXMessageQueue, &transmitBuffer, 0);
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // ESP_LOGI(TAG, "Size of twai message %d", sizeof(twai_message_t));
        // ESP_LOGI(TAG, "Size of gatewayTCPMessage %d", sizeof(gatewayTCPMessage));
        // tcp_send(&gatewayTCPMessage, sizeof(gatewayTCPMessage), 0);
        // ESP_LOGI(TAG, "CAN Packet received");
    }
}
void can_app_transmit(void *pvParameters)
{

    can_init();

    can_message_t message;
    message.identifier = 0xAAAA;
    message.flags = CAN_MSG_FLAG_EXTD;
    message.data_length_code = 4;
    for (int i = 0; i < 4; i++)
    {
        message.data[i] = 0;
    }
    struct CANPacket txCANPacket;

    while (1)
    {

        while (xQueueReceive(CANTXMessageQueue, &txCANPacket, 0) == pdTRUE)
        {
            can_message_t message;
            message.identifier = txCANPacket.ID;
            message.flags = 0;
            if (txCANPacket.IDE == 1)
            {
                message.flags |= CAN_MSG_FLAG_EXTD;
            }
            message.data_length_code = txCANPacket.DLC;
            memcpy(message.data, txCANPacket.data, 8);
            esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(100));
            if (ret != ESP_OK)
            {
                // ESP_LOG_ERROR(TAG, "Failed to transmit CAN message %d", ret);
                printf("Error in message TX %d\r\n", ret);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void CANToTCP_handler()
{
    uint8_t txBuffer[transmitBufferSize];
    while (1)
    {
        while (xQueueReceive(CANRXMessageQueue, &txBuffer, 0) == pdTRUE)
        {
            tcp_send(&txBuffer, transmitBufferSize, 0);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void can_app_init()
{
    xTaskCreate(can_app_receive, "can_app_receive", 4096 * 4, NULL, configMAX_PRIORITIES - 10, NULL);
    xTaskCreate(can_app_transmit, "can_app_transmit", 4096 * 4, NULL, configMAX_PRIORITIES - 11, NULL);
    xTaskCreate(CANToTCP_handler, "CANToTCP_handler", 4096 * 4, NULL, 5, NULL);
}
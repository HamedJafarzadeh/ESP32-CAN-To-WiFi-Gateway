
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "string.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "tcp_app.h"
#include "can_app.h"
// QueueHandle_t uart_transmit_queue

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN

static const char *TAG = "WiFi-Serial Bridge";

// ##############################################

// TCP Server

#define TCP_SERVER_PORT 3333
#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

int socketNumber;
#define tcpRXBufferSize 8192
uint8_t tcpRXBuffer[tcpRXBufferSize];
uint16_t tcpRXBufferHead = 0;
uint16_t tcpRXBufferTail = 0;

static void tcp_heartbeat()
{
    while (1)
    {
        tcp_send((uint8_t *)"GW-HB\r\n", 7, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

enum packetParserStates
{
    init,
    preamble1Found,
    preamble2Found,
    tail1found
};

void processPacket(uint8_t *packet, uint8_t packetLength)
{
    packet[packetLength] = 0;
    // printf("Packet: %s\r\n", packet);
    if (packetLength > 19 && packet[0] == 60) // '{'
    {
        if (packet[1] == 123)
        {
            // '<'
            if (packet[2] == 1)
            {
                // CAN Packet
                struct CANPacket canPacket;
                canPacket.ID = packet[3] |
                               (packet[4] << 8) |
                               (packet[5] << 16) |
                               (packet[6] << 24);
                canPacket.IDE = packet[8];
                canPacket.RTR = 0;
                canPacket.DLC = packet[9];
                memcpy(canPacket.data, packet + 10, canPacket.DLC);
                // printf("CAN Packet: ID:%d, IDE:%d, RTR:%d, DLC:%d\r\n", canPacket.ID, canPacket.IDE, canPacket.RTR, canPacket.DLC);
                xQueueSend(CANTXMessageQueue, &canPacket, 0);
                return;
            }
        }
        // Process packet
    }
}

static void
tcp_parser()
{
    uint8_t packet[255];
    uint8_t packetIDX = 0;
    uint8_t CANPacketCounter = 0;
    enum packetParserStates stateMachine = init;
    while (1)
    {
        if (tcpRXBufferHead != tcpRXBufferTail)
        {
            uint8_t chr = tcpRXBuffer[tcpRXBufferTail];
            tcpRXBufferTail = (tcpRXBufferTail + 1) % tcpRXBufferSize;
            // printf("p:%d, %d, %d\r\n", chr, tcpRXBufferHead, tcpRXBufferTail);
            switch (stateMachine)
            {
            case init:
                if (chr == 13 && tcpRXBuffer[tcpRXBufferTail] == 10)
                {
                    packet[packetIDX++] = (chr);
                    packet[packetIDX++] = (tcpRXBuffer[tcpRXBufferTail]);
                    tcpRXBufferTail = (tcpRXBufferTail + 1) % tcpRXBufferSize; // Remove next from buffer
                    processPacket(packet, packetIDX);
                    packetIDX = 0; // Remove everything up to here
                }
                else if (chr == 60)
                {
                    stateMachine = preamble1Found;
                    if (packetIDX > 0)
                    {
                        processPacket(packet, packetIDX); // Process what we got so far
                    }
                    packetIDX = 0; // Remove everything up to here
                    packet[packetIDX++] = (chr);
                }
                else
                {
                    packet[packetIDX++] = (chr); // Add everthing else that is not a preamble1 and enter
                }

                break;
            case preamble1Found:
                if (chr == 123)
                {
                    stateMachine = preamble2Found;
                    CANPacketCounter = 0;
                    packet[packetIDX++] = (chr);
                }
                else
                {
                    stateMachine = init;
                    packet[packetIDX++] = (chr);
                }
                break;
            case preamble2Found:
                if (CANPacketCounter == 16 && chr == 125)
                {
                    stateMachine = tail1found;
                    packet[packetIDX++] = (chr);
                }
                else
                {
                    CANPacketCounter++;
                    packet[packetIDX++] = (chr);
                }
                break;
            case tail1found:
                if (chr == 62)
                {
                    // tail2 found !
                    packet[packetIDX++] = (chr);
                    // printf("Full Packet found !\r\n");
                    // printf("Buffer Head: %d, Tail: %d\r\n", tcpRXBufferHead, tcpRXBufferTail);
                    processPacket(packet, packetIDX);
                    packetIDX = 0; // Flush Packet
                    stateMachine = init;
                }
                else
                {
                    // Error : We expect tail 2, but we didn't receive it.
                    stateMachine = preamble2Found;
                    packet[packetIDX++] = (chr);
                }
                break;
            }
        }
        else
        {
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        // if (packetIDX > 0)
        // {
        //     // printf("Buffer Parser remained : %s \r\n", packet);
        //     for (int i = 0; i < packetIDX; i++) // _receivedData.addAll(packet);
        //         tcpRXBuffer[tcpRXBufferHead++] = (packet[i]);
        // }
    }
}
static void tcp_loop(const int sock)
{
    int len;
    char rx_buffer[128];

    socketNumber = sock;
    do
    {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0)
        {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        }
        else if (len == 0)
        {
            ESP_LOGW(TAG, "Connection closed");
        }
        else
        {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            // ESP_LOGI(TAG, "[SOCK] Received %d bytes: %s", len, rx_buffer);
            for (int i = 0; i < len; i++)
            {
                tcpRXBuffer[tcpRXBufferHead] = rx_buffer[i];
                tcpRXBufferHead = (tcpRXBufferHead + 1) % tcpRXBufferSize;
            }

            // printf("Buffer Head: %d, Buffer Tail: %d\r\n", tcpRXBufferHead, tcpRXBufferTail);

            // UART_sendData(TAG, rx_buffer);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            // int to_write = len;
            // while (to_write > 0)
            // {
            //     int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
            //     if (written < 0)
            //     {
            //         ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            //     }
            //     to_write -= written;
            // }
        }
    } while (len > 0);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

int tcp_send(uint8_t *data, uint32_t len, uint16_t flags)
{
    int written = send(socketNumber, data, len, flags);
    // ESP_LOGI(TAG, "[SOCK] Write %d bytes: '%s'", written, data);
    return written;
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET)
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(TCP_SERVER_PORT);
        ip_protocol = IPPROTO_IP;
    }
#ifdef CONFIG_EXAMPLE_IPV6
    else if (addr_family == AF_INET6)
    {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(TCP_SERVER_PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", TCP_SERVER_PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    xTaskCreate(tcp_parser, "tcp_parser", 4096, (void *)NULL, tskIDLE_PRIORITY, NULL);

    while (1)
    {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6)
        {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
        xTaskCreate(tcp_heartbeat, "tcp_heartbeat", 4096, (void *)NULL, 5, NULL);
        tcp_loop(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void tcp_app_init()
{
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

// TCP
#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET6, 5, NULL);
#endif
}
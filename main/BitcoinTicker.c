/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"
#include "cJSON.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "st7735s.h"
#include "fontx.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include <inttypes.h>
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "inttypes.h"

// Define website url and port
#define WEB_SERVER "api.coindesk.com"
#define WEB_PORT "80"
#define WEB_PATH "/v1/bpi/currentprice/BTC.json"

// Define tft SPI pins
#define SCREEN_WIDTH 130
#define SCREEN_HEIGHT 132
#define OFFSET_X 0
#define OFFSET_Y 0
#define GPIO_MOSI 23
#define GPIO_SCLK 18
#define GPIO_CS 5
#define GPIO_DC 19
#define GPIO_RESET 25

FontxFile fx16[2];
FontxFile fx24[2];
FontxFile fx32[2];

SemaphoreHandle_t xSemaphore = NULL;

TaskHandle_t httpTask;
TaskHandle_t saveNVS_Handle;

TaskHandle_t myTaskHandle = NULL;
TaskHandle_t myTaskHandle2 = NULL;

nvs_handle_t my_handle;

ST7735_t dev;

UBaseType_t iFreeStackNum = 0;

int32_t restart_counter = 0;

int32_t BTC_rate_int_atl = 0;
int32_t BTC_rate_int_ath = 1;

// Define tag for LOG
static const char *TAG = "Bitcoin Ticker";

// Creating Request
static const char *REQUEST = "GET " WEB_PATH " HTTP/1.0\r\n"
                             "Host: " WEB_SERVER ":" WEB_PORT "\r\n"
                             "User-Agent: esp-idf/1.0 esp32\r\n"
                             "\r\n";


bool checkInteger(int value)
{
    static int prevValue = -1; // Initialize previous value to an impossible value

    if (value ^ prevValue) // XOR current value with previous value
    {
        prevValue = value; // Update previous value
        printf("BTC current value changed to %d\n", value);
        return true;
    }else{
        printf("Same value\n");
    }
    return false;
}

static void save_nvs(void *pvParameters)
{

    while(1){
     if(xSemaphoreTake(xSemaphore, portMAX_DELAY))
     {
      vTaskDelay(200000 / portTICK_PERIOD_MS);
      ESP_LOGI(TAG, "Saving NVS");

     // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

        // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Write
        printf("Updating BTC_rate_int_ath in NVS ... ");
        restart_counter++;
        err = nvs_set_i32(my_handle, "BTC_ath", BTC_rate_int_ath);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }

       // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Write
        printf("Updating BTC_rate_int_atl in NVS ... ");
        restart_counter++;
        err = nvs_set_i32(my_handle, "BTC_atl", BTC_rate_int_atl);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }


     }
    }
    
}

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];
    int total_len = 0;
    char data[2048];
    int BTC_rate_int_current = 0;
    bool header_complete = false;

    

    while (1)
    {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if (err != 0 || res == NULL)
        {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        // Creating socket which can be used to read and write data from server.
        s = socket(res->ai_family, res->ai_socktype, 0);

        if (s < 0)
        {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if (connect(s, res->ai_addr, res->ai_addrlen) != 0)
        {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        // Writing GET Request to server and checking if server returns <0 bytes.
        if (write(s, REQUEST, strlen(REQUEST)) < 0)
        {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        // Server returned >0 bytes
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;

        // Waiting server to send data back and setting timeout limit 4 seconds
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                       sizeof(receiving_timeout)) < 0)
        {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do
        {

            memset(recv_buf, 0, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf) - 1);
            for (int i = 0; i < r; i++)
            {
                // putchar(recv_buf[i]);
            }

            memcpy(data + total_len, recv_buf, r);
            total_len += r;

            // Check if the HTTP response header is complete
            if (!header_complete)
            {
                char *header_end = strstr(data, "\r\n\r\n");
                if (header_end != NULL)
                {
                    header_complete = true;
                    // Skip over the HTTP response header
                    int header_len = header_end - data + 4;
                    total_len -= header_len;
                    memmove(data, data + header_len, total_len);
                    data[total_len] = '\0';
                }
            }

        } while (r > 0);

        // putchar('\n');
        data[total_len] = '\0';

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);

        cJSON *root = cJSON_Parse(data);
        if (root == NULL)
        {
            ESP_LOGE(TAG, "Failed to parse JSON data");
        }
        else
        {
            // Do something with the parsed data
            // BTCPrice=doc["bpi"]["USD"]["rate_float"].as<float>();

            // Get a pointer to the "bpi" object
            cJSON *bpi_obj = cJSON_GetObjectItemCaseSensitive(root, "bpi");

            // Get a pointer to the "BTC" object
            cJSON *btc_obj = cJSON_GetObjectItemCaseSensitive(bpi_obj, "USD");

            // Get a pointer to the "rate_float" value
            cJSON *BTC_rate_int_current_val = cJSON_GetObjectItemCaseSensitive(btc_obj, "rate_float");

            // Get the value as a double
            BTC_rate_int_current = BTC_rate_int_current_val->valueint;

            // Get Heap size
            size_t free_size = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
            size_t total_size = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
        

            // Print the result
            ESP_LOGW(TAG, "Bitcoin price = %d$", BTC_rate_int_current);
            ESP_LOGW(TAG, "Heap size total = %d, free = %d, Free stacksize = %d", total_size, free_size, iFreeStackNum);
            

            //Check if current price higher than ATH ->update ath value
            if(BTC_rate_int_current>BTC_rate_int_ath)
            {
                BTC_rate_int_ath=BTC_rate_int_current;
            }

            //Check if current price lower than ATL -is zero
            if(BTC_rate_int_atl==0)
            {
                BTC_rate_int_atl=BTC_rate_int_current;
            }

            //Check if current price lower than ATL ->update ath value
            if(BTC_rate_int_current<BTC_rate_int_atl)
            {
                BTC_rate_int_atl=BTC_rate_int_current;
            }
            

                // Convert BTC price integer to uint8_t format

                

            char str[10];
            

            uint8_t ascii[30];

             //Check if BTC price has changed, updated TFT
            if(checkInteger(BTC_rate_int_current) == true)
            {
                sprintf(str, "%d", BTC_rate_int_current);   
                strcpy((char *)ascii, str);
                lcdDrawFillRect(&dev, 0, 20, 130, 40, BLACK);
                lcdDrawString(&dev, fx16, 10, 40, ascii, WHITE);

            }
            
          
                //BTC ATH price
                sprintf(str, "%ld", BTC_rate_int_ath);
                strcpy((char *)ascii, str);
                lcdDrawFillRect(&dev, 0, 80, 130, 100, BLACK);
                lcdDrawString(&dev, fx16, 10, 100, ascii, WHITE);
            
           

             
           

            
            //BTC ATL price
            sprintf(str, "%ld", BTC_rate_int_atl);
            strcpy((char *)ascii, str);
            lcdDrawFillRect(&dev, 0, 100, 130, 120, BLACK);
            lcdDrawString(&dev, fx16, 10, 120, ascii, WHITE);
            
            cJSON_Delete(root);
        }

        total_len = 0;
        header_complete = false;
        close(s);
       
        xSemaphoreGive(xSemaphore);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        
        ESP_LOGI(TAG, "Starting again!");
    }
}

static void SPIFFS_Directory(char *path)
{
    DIR *dir = opendir(path);
    assert(dir != NULL);
    while (true)
    {
        struct dirent *pe = readdir(dir);
        if (!pe)
            break;
        ESP_LOGI(TAG, "d_name=%s d_ino=%d d_type=%x", pe->d_name, pe->d_ino, pe->d_type);
    }
    closedir(dir);
}


esp_err_t tft(void)
{
    // set font file


    InitFontx(fx16, "/spiffs/ILGH16XB.FNT", ""); // 8x16Dot Gothic
 

    spi_master_init(&dev, GPIO_MOSI, GPIO_SCLK, GPIO_CS, GPIO_DC, GPIO_RESET);
    lcdInit(&dev, SCREEN_WIDTH, SCREEN_HEIGHT, OFFSET_X, OFFSET_Y);

    lcdFillScreen(&dev, BLACK);

    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx16, 0, buffer, &fontWidth, &fontHeight);
    ESP_LOGE(TAG, "fontWidth=%d fontHeight=%d", fontWidth, fontHeight);

    uint8_t ascii[30];

    strcpy((char *)ascii, "Bitcoin Price");
    lcdDrawString(&dev, fx16, 10, 20, ascii, WHITE);

    strcpy((char *)ascii, "Startup times");
    lcdDrawString(&dev, fx16, 10, 60, ascii, WHITE);

    sprintf((char*)ascii,"%ld",restart_counter);
    lcdDrawString(&dev, fx16, 10, 80, ascii, WHITE);

    // Normal colors
    spi_master_write_command(&dev, 0x20);

    return ESP_OK;
}

esp_err_t update_rates(void)
{
    return ESP_OK;
}

esp_err_t save_rates_nvs(void)
{
    return ESP_OK;
}


void app_main(void)
{

    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 10,
        .format_if_mount_failed = true};

    // Use settings defined above toinitialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is anall-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

     // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Read
        printf("Reading restart counter from NVS ... ");
        
        err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("Restart counter = %" PRIu32 "\n", restart_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Write
        printf("Updating restart counter in NVS ... ");
        restart_counter++;
        err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }

    //vTaskDelay(100 / portTICK_PERIOD_MS);

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Read
        printf("Reading BTC_rate_int_ath from NVS ... ");
        
        err = nvs_get_i32(my_handle, "BTC_ath", &BTC_rate_int_ath);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("BTC_rate_int_ath = %" PRIu32 "\n", BTC_rate_int_ath);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Write
        printf("Updating BTC_rate_int_ath in NVS ... ");
        restart_counter++;
        err = nvs_set_i32(my_handle, "BTC_ath", BTC_rate_int_ath);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }


     // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Read
        printf("Reading BTC_rate_int_atl from NVS ... ");
        
        err = nvs_get_i32(my_handle, "BTC_atl", &BTC_rate_int_atl);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("BTC_rate_int_atl = %" PRIu32 "\n", BTC_rate_int_atl);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Write
        printf("Updating BTC_rate_int_atl in NVS ... ");
        restart_counter++;
        err = nvs_set_i32(my_handle, "BTC_atl", BTC_rate_int_atl);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }



    SPIFFS_Directory("/spiffs");


    //normal code
    ESP_ERROR_CHECK(tft());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    

    xSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(&http_get_task, "http_get_task", 1024*8, NULL, 5, &httpTask);
    xTaskCreate(&save_nvs, "save_nvs", 1024*8, NULL, 5, &saveNVS_Handle);

    while(1)
    {
        iFreeStackNum = uxTaskGetStackHighWaterMark(httpTask);
        vTaskDelay(1000 /portTICK_PERIOD_MS);
        
    }
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "DShotRMT.h" // include anche dshot_send_command + DSHOT_CMD_BEACONx
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "string.h"
#include "assert.h"
#include "driver/i2c.h"

#define WIFI_SSID "Iphone di Prato"
#define WIFI_PASS "Ciaoo111"

static const char *TAG = "DRONE_CONTROL";
static httpd_handle_t server = NULL;
static TaskHandle_t motor_task_handle = NULL;
static TaskHandle_t mpu6050_task_handle = NULL;
static bool motor_running = false;
static DShotRMT *M1, *M2, *M3, *M4;

// Funzione per fermare tutti i motori
void stop_all_motors() {
    M1->setThrottle(0);
    M2->setThrottle(0);
    M3->setThrottle(0);
    M4->setThrottle(0);
    printf("[INFO] Tutti i motori fermati\n");
}

// Configurazione I2C per MPU6050
#define PIN_SDA                     21
#define PIN_CLK                     22
#define I2C_ADDRESS                 0x68  // Indirizzo I2C del MPU6050
#define I2C_MASTER_NUM              I2C_NUM_0

// Registri MPU6050
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_PWR_MGMT_1          0x6B

// Funzione per inizializzare il MPU-6050
esp_err_t mpu6050_init_sensor() {
    ESP_LOGI(TAG, "Initializing MPU6050...");
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_CLK,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,
        },
        .clk_flags = 0,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C driver installed successfully");
    
    // Scansiona il bus I2C per vedere se ci sono dispositivi
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int device_count = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at address 0x%02X", addr);
            device_count++;
        }
    }
    
    if (device_count == 0) {
        ESP_LOGE(TAG, "No I2C devices found! Check connections and pull-up resistors.");
        ESP_LOGE(TAG, "Expected connections:");
        ESP_LOGE(TAG, "  VCC -> 3.3V");
        ESP_LOGE(TAG, "  GND -> GND");
        ESP_LOGE(TAG, "  SDA -> GPIO %d", PIN_SDA);
        ESP_LOGE(TAG, "  SCL -> GPIO %d", PIN_CLK);
        ESP_LOGE(TAG, "  Add 4.7kΩ pull-up resistors on SDA and SCL to 3.3V");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s)", device_count);
    }
    
    // Wake up the MPU6050
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0, true); // Wake up
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

// Funzione per leggere i dati dal MPU-6050
void read_mpu6050_data() {
    uint8_t data[6];
    int16_t accel_x, accel_y, accel_z;
    
    // Posiziona il puntatore del registro interno al MPU6050 al registro MPU6050_ACCEL_XOUT_H
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set register pointer: %s", esp_err_to_name(ret));
        return;
    }
    
    // Leggi i dati dell'accelerometro
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data,     I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 2, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 3, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 4, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 5, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data: %s", esp_err_to_name(ret));
        return;
    }
    
    // Converti i dati raw
    accel_x = (data[0] << 8) | data[1];
    accel_y = (data[2] << 8) | data[3];
    accel_z = (data[4] << 8) | data[5];
    
    // Stampa i dati raw
    ESP_LOGI(TAG, "Accel raw: X=%d, Y=%d, Z=%d", accel_x, accel_y, accel_z);
}

// Task dedicato per leggere i dati del MPU-6050 ogni 500ms
void mpu6050_task(void *pvParameters) {
    ESP_LOGI(TAG, "MPU6050 task started");
    int read_count = 0;
    
    while (true) {
        read_count++;
        ESP_LOGI(TAG, "MPU6050 read attempt #%d", read_count);
        
        // Leggi i dati dal MPU-6050
        read_mpu6050_data();
        
        // Aspetta 500ms prima della prossima lettura
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task per controllo motori
void motor_control_task(void *pvParameters) {
    int test_count = 0;
    
    while (true) {
        if (!motor_running) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        test_count++;
        printf("[INFO] === CICLO #%d ===\n", test_count);

        // Arming all'inizio di ogni ciclo per sicurezza
        printf("[INFO] Arming ESC (2 secondi)...\n");
        for (int i = 0; i < 40000; i++) {
            if (!motor_running) break;
            
            M1->setThrottle(0);
            M2->setThrottle(0);
            M3->setThrottle(0);
            M4->setThrottle(0);

            if (i % 500 == 0) {
                printf("[DEBUG] arming... %d ms\n", i * 2);
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }        

        // Salita da 100 a 2000
        for (int throttle = 100; throttle <= 2000; throttle += 20) {
            if (!motor_running) break;
            
            M1->setThrottle(throttle);
            M2->setThrottle(throttle);
            M3->setThrottle(throttle);
            M4->setThrottle(throttle);
            
            for (int second = 1; second <= 1; second++) {
                if (!motor_running) break;
                vTaskDelay(pdMS_TO_TICKS(1000));
                printf("[DEBUG] Throttle %d [SALITA]\n", throttle);
            }
        }

        // Discesa da 2000 a 100
        for (int throttle = 2000; throttle >= 100; throttle -= 20) {
            if (!motor_running) break;
            
            M1->setThrottle(throttle);
            M2->setThrottle(throttle);
            M3->setThrottle(throttle);
            M4->setThrottle(throttle);
            
            for (int second = 1; second <= 1; second++) {
                if (!motor_running) break;
                vTaskDelay(pdMS_TO_TICKS(1000));
                printf("[DEBUG] Throttle %d [DISCESA]\n", throttle);
            }
        }

        if (motor_running) {
            printf("[INFO] Fine ciclo completo. Pausa lunga 5 secondi...\n");
            stop_all_motors();
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

// Handler per pagina principale
esp_err_t root_handler(httpd_req_t *req) {
    const char* html_page = 
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "    <title>Drone Control</title>"
        "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "    <style>"
        "        body { font-family: Arial; text-align: center; margin: 50px; }"
        "        button { "
        "            font-size: 24px; "
        "            padding: 20px 40px; "
        "            margin: 20px; "
        "            border: none; "
        "            border-radius: 10px;"
        "            cursor: pointer;"
        "        }"
        "        .start { background-color: #4CAF50; color: white; }"
        "        .stop { background-color: #f44336; color: white; }"
        "        .status { font-size: 18px; margin: 20px; }"
        "    </style>"
        "</head>"
        "<body>"
        "    <h1>Drone Control</h1>"
        "    <div class=\"status\" id=\"status\">Stato: Fermo</div>"
        "    <button class=\"start\" onclick=\"startMotors()\">START</button>"
        "    <button class=\"stop\" onclick=\"stopMotors()\">STOP</button>"
        "    "
        "    <script>"
        "        function startMotors() {"
        "            fetch('/start')"
        "                .then(response => response.text())"
        "                .then(data => {"
        "                    document.getElementById('status').textContent = 'Stato: Avviato';"
        "                    console.log(data);"
        "                });"
        "        }"
        "        "
        "        function stopMotors() {"
        "            fetch('/stop')"
        "                .then(response => response.text())"
        "                .then(data => {"
        "                    document.getElementById('status').textContent = 'Stato: Fermo';"
        "                    console.log(data);"
        "                });"
        "        }"
        "    </script>"
        "</body>"
        "</html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_page, strlen(html_page));
}

// Handler per start
esp_err_t start_handler(httpd_req_t *req) {
    motor_running = true;
    printf("[INFO] Motori avviati via web\n");
    httpd_resp_send(req, "Motori avviati", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler per stop
esp_err_t stop_handler(httpd_req_t *req) {
    motor_running = false;
    stop_all_motors();
    printf("[INFO] Motori fermati via web\n");
    httpd_resp_send(req, "Motori fermati", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Inizializza WiFi
void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    printf("[INFO] WiFi avviato, connessione a %s...\n", WIFI_SSID);
    
    // Attendi connessione
    int retry = 0;
    while (esp_wifi_connect() != ESP_OK && retry < 10) {
        printf("[WARN] Connessione WiFi fallita, riprovo... (%d/10)\n", retry + 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
        retry++;
    }
    
    if (retry >= 10) {
        printf("[ERROR] Impossibile connettersi al WiFi\n");
        return;
    }
    
    printf("[INFO] Connesso al WiFi!\n");
}

// Inizializza webserver
void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler
        };
        httpd_register_uri_handler(server, &root_uri);
        
        httpd_uri_t start_uri = {
            .uri = "/start",
            .method = HTTP_GET,
            .handler = start_handler
        };
        httpd_register_uri_handler(server, &start_uri);
        
        httpd_uri_t stop_uri = {
            .uri = "/stop",
            .method = HTTP_GET,
            .handler = stop_handler
        };
        httpd_register_uri_handler(server, &stop_uri);
        
        printf("[INFO] Webserver avviato su porta 80\n");
    }
}

extern "C" void app_main()
{
    printf("[BOOT] ESP32 avviato\n");

    // Inizializza NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Disabilita il task watchdog per evitare reset durante il debug
    esp_task_wdt_deinit();
    printf("[INFO] Task watchdog disabilitato\n");

    // Inizializza WiFi
    wifi_init();

    // Inizializza ESC su di un GPIO con protocollo DSHOT600
    M1 = new DShotRMT(GPIO_NUM_18, DSHOT600); //M1
    M2 = new DShotRMT(GPIO_NUM_14, DSHOT600); //M2
    M3 = new DShotRMT(GPIO_NUM_19, DSHOT600); //M3
    M4 = new DShotRMT(GPIO_NUM_27, DSHOT600); //M4
    M1->begin();
    M2->begin();
    M3->begin();
    M4->begin();
    printf("[INFO] ESC inizializzato con DShot600\n");

    // Inizializza MPU-6050
    ret = mpu6050_init_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU-6050");
    }

    // Crea task per controllo motori
    xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 5, &motor_task_handle);
    printf("[INFO] Task controllo motori creato\n");

    // Crea task per MPU-6050
    xTaskCreate(mpu6050_task, "mpu6050_sensor", 4096, NULL, 4, &mpu6050_task_handle);
    printf("[INFO] Task MPU-6050 creato\n");

    // Avvia webserver
    start_webserver();

    printf("[INFO] Sistema pronto! Connettiti all'IP dell'ESP32\n");

    // Loop principale
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }


}

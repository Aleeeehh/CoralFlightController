#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "DShotRMT.h" // include anche dshot_send_command + DSHOT_CMD_BEACONx

extern "C" void app_main()
{
    printf("[BOOT] ESP32 avviato\n");

    // Disabilita il task watchdog per evitare reset durante il debug
    esp_task_wdt_deinit();
    printf("[INFO] Task watchdog disabilitato\n");

    // Inizializza ESC su di un GPIO con protocollo DSHOT600
    DShotRMT M1(GPIO_NUM_18, DSHOT600); //M1
    DShotRMT M4(GPIO_NUM_27, DSHOT600); //M4
    M1.begin();
    M4.begin();
    printf("[INFO] ESC inizializzato con DShot600 su GPIO 18 (motore M1)\n");

    int test_count = 0;
    while (true)
    {
        test_count++;
        printf("[INFO] === CICLO #%d ===\n", test_count);

        //printf("senza arming\n");


        // Arming all'inizio di ogni ciclo per sicurezza
        printf("[INFO] Arming ESC (2 secondi)...\n");
        for (int i = 0; i < 40000; i++)
        {
            M1.setThrottle(0);
            M4.setThrottle(0);
            if (i % 500 == 0)
            {
                printf("[DEBUG] arming... %d ms\n", i * 2);
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }        
        

        // Salita da 100 a 500
        for (int throttle = 100; throttle <= 1100; throttle += 20) 
        {
            M1.setThrottle(throttle);
            M4.setThrottle(throttle);
           
            //1 secondo per velocità
            for (int second = 1; second <= 1; second++)
            {
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1 secondo
                printf("[DEBUG] Throttle %d [SALITA]\n", throttle);
            }
        }

        // Discesa da 500 a 100
        for (int throttle = 1100; throttle >= 100; throttle -= 20) 
        {   
            M1.setThrottle(throttle);
            M4.setThrottle(throttle);
            
            //1 secondo per velocità
            for (int second = 1; second <= 1; second++)
            {
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1 secondo
                printf("[DEBUG] Throttle %d [DISCESA]\n", throttle);
            }
        }

        printf("[INFO] Fine ciclo completo. Pausa lunga 5 secondi...\n");
        M1.setThrottle(0);
        M4.setThrottle(0);
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 secondi di pausa tra i cicli
    }

    printf("[INFO] Test completato. Loop idle.\n");

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // idle per evitare reboot
    }
}

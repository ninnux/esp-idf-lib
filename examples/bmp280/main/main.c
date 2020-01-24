#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "esp_event.h"
#include "nvs_flash.h"
#include <bmp280.h>

#define SDA_GPIO 16
#define SCL_GPIO 17

SemaphoreHandle_t xSemaphore = NULL;

static RTC_DATA_ATTR struct timeval sleep_enter_time;

float t;
float p;
float h;

void sleeppa(int sec)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
	    //deepsleep=1;
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:{
	    //deepsleep=0;
            printf("Not a deep sleep reset\n");
	    //cayenne_lpp_reset(&tlpp);
	    //cayenne_lpp_reset(&hlpp);
	    //cayenne_lpp_reset(&plpp);
            //cayenne_lpp_add_analog_input(&tlpp,0,SLEEP_INTERVAL);
            //cayenne_lpp_add_analog_input(&hlpp,0,SLEEP_INTERVAL);
            //cayenne_lpp_add_analog_input(&plpp,0,SLEEP_INTERVAL);
  	    //sensordata_init2((unsigned char **) &rtc_buffer, &rtc_buffer_len);
        }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = sec;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    //const int ext_wakeup_pin_1 = 25;
    //const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    //const int ext_wakeup_pin_2 = 26;
    //const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    //printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    //esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    //rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);
    //deepsleep=1;
    esp_deep_sleep_start();
}

void bmp280_status(void *pvParamters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.mode = BMP280_MODE_FORCED;
    bmp280_t dev;
    float psum=0;
    float tsum=0;
    float hsum=0;
    esp_err_t res;

   printf("pre semaforo\n");
	    while (i2cdev_init() != ESP_OK)
	    {
	        printf("Could not init I2Cdev library\n");
	        vTaskDelay(250 / portTICK_PERIOD_MS);
	    }

	    while (bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, I2C_NUM_0 , SDA_GPIO, SCL_GPIO) != ESP_OK)
	    {
	        printf("Could not init device descriptor\n");
	        vTaskDelay(250 / portTICK_PERIOD_MS);
	    }

	    while ((res = bmp280_init(&dev, &params)) != ESP_OK)
	    {
	        printf("Could not init BMP280, err: %d\n", res);
	        vTaskDelay(250 / portTICK_PERIOD_MS);
	    }

	    bool bme280p = dev.id == BME280_CHIP_ID;
	    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

	    float pressure, temperature, humidity;
	    int i=0;
	    bool busy;
	    while (i<10) // scaldo il sensore con 10 letture a vuoto
	    {	
	        i++;
	        vTaskDelay(500 / portTICK_PERIOD_MS);
		// force mode
		bmp280_force_measurement(&dev);
		do { bmp280_is_measuring(&dev, &busy); } while(busy);	
		//
	        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
	        {
	            printf("Temperature/pressure reading failed\n");
	            continue;
	        }

	        psum+=pressure;
	        tsum+=temperature;

	        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
	        if (bme280p){
	            printf(", Humidity: %.2f\n", humidity);
	            hsum+=humidity;
	        }
	        else{
	            printf("\n");
	        }
	    }
	    i=0;
	    pressure=0;
            temperature=0;
            humidity=0;
	    psum=0;
            tsum=0;
            hsum=0;
	    while (i<10)
	    {	
		i++;
	        vTaskDelay(500 / portTICK_PERIOD_MS);
		// force mode
		bmp280_force_measurement(&dev);
		do { bmp280_is_measuring(&dev, &busy); } while(busy);	
		//
	        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
	        {
	            printf("Temperature/pressure reading failed\n");
	            continue;
	        }

		psum+=pressure;
		tsum+=temperature;

	        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
	        if (bme280p){
	            printf(", Humidity: %.2f\n", humidity);
		    hsum+=humidity;
		}
	        else{
	            printf("\n");
		}
	    }
	    //int p=(psum/i*10)/100;
	    //int t=(tsum/i*10);
	    if (bme280p){
	    	//int h=(hsum/i*10);
	      p=psum/i/100;
	      t=tsum/i;
	      h=hsum/i;
	    }else{
	      p=psum/i;
	      t=tsum/i;
	      h=hsum/i;
	    }
    sleeppa(300);
}

void bmp280_test(void *pvParamters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.mode = BMP280_MODE_FORCED;
    bmp280_t dev;

    esp_err_t res;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while ((res = bmp280_init(&dev, &params)) != ESP_OK)
    {
        printf("Could not init BMP280, err: %d\n", res);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    float pressure, temperature, humidity;

    while (1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
	// force mode
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        if (bme280p)
            printf(", Humidity: %.2f\n", humidity);
        else
            printf("\n");
    }
}

void app_main()
{
    printf("inizio\n");
    xTaskCreate( &bmp280_status, "bmp280_status", 2048, NULL, 5, NULL );
    //xTaskCreatePinnedToCore(bmp280_test, "bmp280_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


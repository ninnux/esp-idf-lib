#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hmc5883l.h>
#include <math.h>

#define SDA_GPIO 16
#define SCL_GPIO 17

float xvalue[200];
float yvalue[200];
float zvalue[200];
int i,k;
float xmin=0;
float xmax=0;
float ymin=0;
float ymax=0;
float zmin=0;
float zmax=0;

float Xoffset;
float Yoffset;
float Xscale;
float Yscale;
float xmag;
float ymag;
float angle;


void hmc5883l_test(void *pvParameters)
{
    hmc5883l_dev_t dev;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2C bus\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    hmc5883l_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO);
    while (hmc5883l_init(&dev) != ESP_OK)
    {
        printf("HMC5883L not found\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    hmc5883l_set_opmode(&dev, HMC5883L_MODE_CONTINUOUS);
    hmc5883l_set_samples_averaged(&dev, HMC5883L_SAMPLES_8);
    hmc5883l_set_data_rate(&dev, HMC5883L_DATA_RATE_07_50);
    hmc5883l_set_gain(&dev, HMC5883L_GAIN_1090);


   hmc5883l_data_t data;
    for(i=0;i<200;i++){
	if (hmc5883l_get_data(&dev, &data) == ESP_OK){
		xvalue[i]=data.x;
		yvalue[i]=data.y;
		zvalue[i]=data.z;
		if(xmax<xvalue[i]){ xmax=xvalue[i];};
		if(ymax<yvalue[i]){ ymax=yvalue[i];};
		if(zmax<zvalue[i]){ zmax=zvalue[i];};
		if(xmin>xvalue[i]){ xmin=xvalue[i];};
		if(ymin>yvalue[i]){ ymin=yvalue[i];};
		if(zmin>zvalue[i]){ zmin=zvalue[i];};
	}
        vTaskDelay(50 / portTICK_PERIOD_MS);
	
    }
    Xoffset = (xmax + xmin)/2;
    Yoffset = (ymax + ymin)/2;

    Xscale = xmax - xmin;
    Yscale = ymax - ymin;

            printf("Magnetic data min: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", xmin, ymin, zmin);
            printf("Magnetic data max: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", xmax, ymax, zmax);
            printf("Offset X:%.2f, Y:%.2f\n", Xoffset,Yoffset);
            printf("scale X:%.2f, Y:%.2f\n", Xscale,Yscale);

    while (1)
    {
        hmc5883l_data_t data2;
        if (hmc5883l_get_data(&dev, &data2) == ESP_OK){
	xmag=data2.x;
	ymag=data2.y;
        xmag = (xmag - Xoffset) * Xscale;
        ymag = (ymag - Yoffset) * Yscale;
        angle = atan2(xmag, ymag)*(180/3.14);
        printf("Magnetic data2: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", data2.x, data2.y, data2.z);
        printf("results X:%.2f, Y:%.2f , angele:%.2f\n", xmag, ymag, angle);
        }else{
            printf("Could not read HMC5883L data\n");
	}	

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(hmc5883l_test, "hmc5883l_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);
}


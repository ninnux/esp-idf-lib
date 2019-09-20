#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <qmc5883l.h>
#include <math.h>
#include <string.h>

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
float Zoffset;
float scale_x;
float scale_y;
float scale_z;
float xmag;
float ymag;
float zmag;
float angle;
float angle2;
float angle3;
double avg_delta_x;
double avg_delta_y;
double avg_delta_z;
double avg_delta;


void qmc5883l_test(void *pvParameters)
{
    qmc5883l_t dev;

    memset(&dev, 0, sizeof(qmc5883l_t));

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(qmc5883l_init_desc(&dev, 0, QMC5883L_I2C_ADDR_DEF, SDA_GPIO, SCL_GPIO));

    // 50Hz data rate, 128 samples, -2G..+2G range
    ESP_ERROR_CHECK(qmc5883l_set_config(&dev, QMC5883L_DR_50, QMC5883L_OSR_128, QMC5883L_RNG_2));



   qmc5883l_data_t data;
    for(i=0;i<200;i++){
	if (qmc5883l_get_data(&dev, &data) == ESP_OK){
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
    // from https://appelsiini.net/2018/calibrate-magnetometer/
    Xoffset=(xmax+xmin)/2;
    Yoffset=(ymax+ymin)/2;
    Zoffset = (zmax + zmin)/2;
    avg_delta_x=(xmax-xmin)/2;
    avg_delta_y=(ymax-ymin)/2;
    avg_delta_z=(zmax-zmin)/2;
    printf("avg_x:%.2f avg_y:%.2f avg_z:%.2f\n",avg_delta_x,avg_delta_y,avg_delta_z); 
    avg_delta=(avg_delta_x+avg_delta_y+avg_delta_z)/3;
    
    scale_x=avg_delta/avg_delta_x;
    scale_y=avg_delta/avg_delta_y;
    scale_z=avg_delta/avg_delta_z;
    ///

	
    //Xoffset = (xmax + xmin)/2;
    //Yoffset = (ymax + ymin)/2;

    //Xscale = xmax - xmin;
    //Yscale = ymax - ymin;

    printf("Magnetic data min: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", xmin, ymin, zmin);
    printf("Magnetic data max: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", xmax, ymax, zmax);
    //printf("Offset X:%.2f, Y:%.2f\n", Xoffset,Yoffset);
    //printf("scale X:%.2f, Y:%.2f\n", Xscale,Yscale);

    while (1)
    {
        qmc5883l_data_t data2;
        if (qmc5883l_get_data(&dev, &data2) == ESP_OK){
	xmag=data2.x;
	ymag=data2.y;
	zmag=data2.z;
        xmag = (xmag - Xoffset) * scale_x;
        ymag = (ymag - Yoffset) * scale_y;
        zmag = (zmag - Zoffset) * scale_z;
        angle = atan2(xmag, ymag)*(180/3.14)+180;
        angle2 = atan2(xmag, zmag)*(180/3.14)+180;
        angle3 = atan2(ymag, zmag)*(180/3.14)+180;


        printf("Magnetic data min: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", xmin, ymin, zmin);
        printf("Magnetic data max: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", xmax, ymax, zmax);
        printf("Magnetic reading data2: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", data2.x, data2.y, data2.z);
        printf("results X:%.2f,Y:%.2f , angle:%.2f\n", xmag, ymag, angle);
        printf("angle:%.2f\n", 360-angle);
        printf("angle2:%.2f\n", 360-angle2);
        printf("angle3:%.3f\n", 360-angle3);
        }else{
            printf("Could not read qmc5883L data\n");
	}	

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(qmc5883l_test, "qmc5883l_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);
}


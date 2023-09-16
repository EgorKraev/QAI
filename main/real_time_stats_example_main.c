
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "hardconfig.h"




void app_main(void)
{
    //Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(100));
}

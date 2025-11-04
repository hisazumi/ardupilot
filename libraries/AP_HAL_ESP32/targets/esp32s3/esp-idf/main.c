/* Hello World Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stddef.h>
#include "driver/gpio.h"

int main(int argc, char *argv[]);

void app_main()
{
    // M5StampFly ToF sensor XSHUT control
    // GPIO 9: Front ToF XSHUT - disable by pulling LOW
    // GPIO 7: Bottom ToF XSHUT - enable by pulling HIGH

    // Disable Front ToF sensor
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_9, 0);

    // Enable Bottom ToF sensor
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);

    main(0, NULL);
}

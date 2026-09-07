#ifndef DHT22_H
#define DHT22_H

#include "driver/gpio.h"

void dht22_init(gpio_num_t pin);
int dht22_read(float *temperature, float *humidity);

#endif

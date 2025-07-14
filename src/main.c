#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include "GPS.h"
#include "Accelerometer.h"



int main(void)
{
    
    UART_INIT();
    configure_gps_module();
    printk("Waiting for GPS GLL sentences...\n");
    while (1) {
        print_gps_data();
        lsm303ah_config_and_print();
        k_sleep(K_MSEC(1000));
    }
}


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "GPS.h"

static struct gps_data current_gps_data = {0};
static char nmea_buffer[NMEA_BUFFER_SIZE];
static int nmea_buffer_pos = 0;


static uint8_t rx_buffers[NUM_RX_BUFFERS][RX_BUFFER_SIZE];
static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static int current_rx_buffer = 0;

/* Flag to indicate new GPS data is available */
static volatile bool new_gps_data = false;

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static uint8_t tx_buf[] = {"Waiting for GPS coordinates...\r\n"};

/* Function Prototypes */


/* Convert NMEA coordinate format to decimal degrees */
static float nmea_to_decimal(const char *nmea_coord, char direction)
{
    if (strlen(nmea_coord) == 0) {
        return 0.0;
    }
    
    float coord = atof(nmea_coord);
    float degrees = (int)(coord / 100);
    float minutes = coord - (degrees * 100);
    
    float decimal = degrees + (minutes / 60.0);
    
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

/* Parse GPGLL (Geographic Position - Latitude/Longitude) sentence */
static int parse_gpgll(char *sentence)
{
    char *fields[MAX_NMEA_FIELDS];
    char *token;
    int field_count = 0;
    char sentence_copy[NMEA_BUFFER_SIZE];
    
    /* Make a copy since strtok modifies the string */
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    printk("GPGLL: %s\n", sentence);
    
    /* Tokenize the sentence */
    token = strtok(sentence_copy, ",");
    while (token != NULL && field_count < MAX_NMEA_FIELDS) {
        fields[field_count++] = token;
        token = strtok(NULL, ",");
    }
    
    if (field_count < 7) {
        return -1; // Invalid sentence
    }
    
    /* Check if data is valid (field 6 should be 'A' for active) */
    if (field_count > 6 && fields[6][0] != 'A') {
        current_gps_data.valid_fix = false;
        return 0; // Data not valid
    }
    
    current_gps_data.valid_fix = true;
    
    /* Parse latitude (field 1) and direction (field 2) */
    if (strlen(fields[1]) > 0 && strlen(fields[2]) > 0) {
        current_gps_data.latitude = nmea_to_decimal(fields[1], fields[2][0]);
    }
    
    /* Parse longitude (field 3) and direction (field 4) */
    if (strlen(fields[3]) > 0 && strlen(fields[4]) > 0) {
        current_gps_data.longitude = nmea_to_decimal(fields[3], fields[4][0]);
    }
    
    /* Parse time (field 5) - HHMMSS.SSS */
    if (strlen(fields[5]) >= 6) {
        snprintf(current_gps_data.time_str, sizeof(current_gps_data.time_str),
                 "%.2s:%.2s:%.2s", fields[5], fields[5] + 2, fields[5] + 4);
    }
    
    return 0;
}

static void process_nmea_line(char *line)
{
    if (strncmp(line, "$GPGLL", 6) == 0) {
        if (parse_gpgll(line) == 0) {
            new_gps_data = true; // Signal new data is available
        }
    }
}

static void process_uart_data(uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        
        if (c == '\r') {
            continue; // Skip carriage return
        }
        
        if (c == '\n') {
            /* End of NMEA sentence */
            nmea_buffer[nmea_buffer_pos] = '\0';
            
            /* Process the complete NMEA sentence */
            if (nmea_buffer_pos > 0) {
                process_nmea_line(nmea_buffer);
            }
            
            nmea_buffer_pos = 0; // Reset buffer
        } else {
            /* Add character to buffer */
            if (nmea_buffer_pos < (NMEA_BUFFER_SIZE - 1)) {
                nmea_buffer[nmea_buffer_pos++] = c;
            } else {
                nmea_buffer_pos = 0; // Buffer overflow, reset
            }
        }
    }
}

void print_gps_data(void)
{
    printk("\n=== GPS COORDINATES ===\n");
    
    if (current_gps_data.valid_fix) {
        printk("Latitude: %.6f°\n", current_gps_data.latitude);
        printk("Longitude: %.6f°\n", current_gps_data.longitude);
        printk("Time: %s\n", current_gps_data.time_str);
    } else {
        printk("No GPS fix - waiting for satellites...\n");
    }
    
    printk("=======================\n\n");
}

static int send_gps_command(const char *command)
{
    int ret;
    size_t len = strlen(command);
    
    if (len >= UART_TX_BUFFER_SIZE) {
        printk("Command too long\n");
        return -EINVAL;
    }
    
    memcpy(tx_buffer, command, len);
    
    ret = uart_tx(uart, tx_buffer, len, SYS_FOREVER_US);
    if (ret) {
        printk("Failed to send command: %d\n", ret);
        return ret;
    }
    
    printk("Sent GPS command: %s", command);
    return 0;
}

void configure_gps_module(void)
{
    k_sleep(K_SECONDS(2)); // Wait for GPS to fully initialize

    /* Using these commands from RADIONOVA GNSS/GPS MTK NMEA Packet Commands document */
    /* Sets baudrate to 115200 */
    send_gps_command("$PMTK251,115200*1F\r\n");
    k_sleep(K_MSEC(100));
    
    /* Enable EASY */
    send_gps_command("$PMTK353,1,0,0,0,0*2A\r\n");
    k_sleep(K_MSEC(100));
    
    /* Set position update rate to 1Hz (default) */
    send_gps_command("$PMTK220,1000*1F\r\n");
    k_sleep(K_MSEC(100));
    
    /* Enable specific NMEA sentence (GLL only) */
    send_gps_command("$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
    k_sleep(K_MSEC(100));
    
    printk("GPS module configured for GLL sentences\n");
}

/* UART callback function */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        process_uart_data(&evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
        break;
        
    case UART_RX_BUF_REQUEST:
        /* Provide next buffer for receiving */
        current_rx_buffer = (current_rx_buffer + 1) % NUM_RX_BUFFERS;
        uart_rx_buf_rsp(uart, rx_buffers[current_rx_buffer], RX_BUFFER_SIZE);
        break;

    case UART_RX_DISABLED:
        printk("UART RX disabled, restarting...\n");
        uart_rx_enable(uart, rx_buffers[current_rx_buffer], RX_BUFFER_SIZE, 100);
        break;

    case UART_TX_ABORTED:
        printk("UART TX aborted\n");
        break;

    default:
        break;
    }
}

void UART_INIT(void)
{
    int ret;
    
    if (!device_is_ready(uart)) {
        printk("UART device not ready\n");
        return 1;
    }
    
    ret = uart_callback_set(uart, uart_cb, NULL);
    if (ret) {
        printk("Failed to set UART callback\n");
        return 1;
    }
    
    ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);
    if (ret) {
        printk("Failed to send initial message\n");
        return 1;
    }
    
   
    ret = uart_rx_enable(uart, rx_buffers[current_rx_buffer], RX_BUFFER_SIZE, RECEIVE_TIMEOUT);
    if (ret) {
        printk("Failed to enable UART RX\n");
        return 1;
    }
    
    printk("UART initialized, configuring GPS...\n");
}
#define UART_TX_BUFFER_SIZE 128
#define NUM_RX_BUFFERS 2
#define RX_BUFFER_SIZE 128
#define RECEIVE_TIMEOUT 100


#define NMEA_BUFFER_SIZE 256
#define MAX_NMEA_FIELDS 20

struct gps_data {
    float latitude;
    float longitude;
    bool valid_fix;
    char time_str[16];
};

static int parse_gpgll(char *sentence);
static float nmea_to_decimal(const char *nmea_coord, char direction);
static void process_nmea_line(char *line);
static void process_uart_data(uint8_t *data, size_t len);

void print_gps_data(void);
static int send_gps_command(const char *command);
void configure_gps_module(void);
void UART_INIT(void);
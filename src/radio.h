#define PPM_TIME_PULSE 500
#define PPM_TIME_OFF_0 990
#define PPM_TIME_OFF_1 1940
#define PPM_TIME_SYNC 3900

static void send_ppm(uint8_t data[], uint8_t length, uint8_t repeats, uint8_t solight_mode);
static void prologue_send(uint8_t id, uint8_t channel, int16_t temperature10, uint8_t humidity, uint8_t battery_status, uint8_t button_pressed);
static uint8_t rubicson_crc(uint8_t const data[]);
void solight_te44_send(uint8_t id, uint8_t channel, int16_t temperature10);

// static uint16_t ds18b20_readScratchPad();
uint16_t ds18b20_readScratchPad2();
// uint16_t ds18b20_read_single();
uint16_t ds18b20_read_slave();
// uint16_t ds18b20_convert();
uint16_t ds18b20_convert_slave();

// Special return values
static const uint16_t kDS18B20_DeviceNotFound = 0xA800;
static const uint16_t kDS18B20_CrcCheckFailed = 0x5000;

void onewire_search_init();
bool onewire_reset();
void onewire_write_bit(uint8_t bit);
void onewire_write(uint8_t byte);
uint8_t onewire_read_bit();
uint8_t onewire_read();
void onewire_match_rom();
// void onewire_skiprom();
bool _search_next();
bool _search_devices(uint8_t command);
bool onewire_search();
// bool onewire_alarm_search();
bool onewire_check_rom_crc();
uint8_t onewire_get_first_address_byte();
uint16_t onewire_get_address_hash();

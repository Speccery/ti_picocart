// psram.h

void psram_init(void);
void psram_read_spi(uint8_t *dest, uint32_t addr, uint32_t len);
void psram_write_spi(uint32_t addr, uint32_t len, const uint8_t *src);
void psram_read_id(uint8_t *dest2);
void psram_enter_qpi(void);
void psram_exit_qpi(void);
void psram_read_qpi(uint8_t *dest, uint32_t addr, uint32_t len);
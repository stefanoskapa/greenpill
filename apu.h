#include <stdint.h>
void apu_init(void);
void apu_step(int cycles);
void audio_delay(void);
void apu_memw_callback(uint16_t addr, uint8_t b);


#include <stdint.h>
void send_word_to_buffer(uint8_t byte1, uint8_t byte2, int x, bool is_obj, uint8_t flags);
void ppu_init(void);
void ppu_step();
void ppu_steps(int cycles);

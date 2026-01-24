#include <SDL2/SDL.h>
#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// combined 8-bit registers
#define AF  ((F & 0xF0) | (A << 8))
#define BC  (C | (B << 8))
#define DE  (E | (D << 8))
#define HL  (L | (H << 8))
// z - zero flag (7th bit)
#define SETF_Z F = (F | 0b10000000)
#define CLRF_Z F = (F & 0b01111111)
#define READF_Z ((F & 0b10000000) != 0)
// n - subtraction flag (BCD) (6th bit)
#define SETF_N F = (F | 0b01000000)
#define CLRF_N F = (F & 0b10111111)
#define READF_N ((F & 0b01000000) != 0)
// h - half-carry flag (BCD) (5th bit)
#define SETF_H F = (F | 0b00100000)
#define CLRF_H F = (F & 0b11011111)
#define READF_H ((F & 0b00100000) != 0)
// c - carry flag (4th bit)
#define SETF_C F = (F | 0b00010000)
#define CLRF_C F = (F & 0b11101111)
#define READF_C ((F & 0b00010000) != 0)

// Frame timing
#define CYCLES_PER_FRAME 70224  // 154 scanlines × 456 cycles
/*
 * A frame is not exactly one 60th of a second: the Game Boy 
 * runs slightly slower than 60 Hz, as one frame takes ~16.74 ms 
 * instead of ~16.67 (the error is 0.45%).
 */
#define NANOS_PER_FRAME 16742706 
#define CYCLES_PER_SAMPLE (4194304.0f / 44100.0f)

#define SAMPLE_RATE 48000
#define AMPLITUDE   28000

uint64_t timer_diff(struct timespec *start, struct timespec *end);
int cpu_step(void);
void chan2debug(uint8_t nr24);
void ppu_steps(int cycles);
void apu_step(int cycles);
void ppu_step();
void show_registers(void);
void show_cartridge_info();
void mem_write8(uint16_t addr, uint8_t b);
void mem_write16(uint16_t addr, uint16_t b);
void dec_reg16(uint8_t *low, uint8_t *high);
void inc_reg16(uint8_t *low, uint8_t *high);
void timer_step(int cycles);
void handle_interrupts(void);
uint8_t *get_tile(int index);
void show_frame();
void send_word_to_buffer(uint8_t byte1, uint8_t byte2, int x, bool is_obj, uint8_t flags);
void print_bin8(uint8_t v);
void check_joyp();
void toggle_fullscreen(SDL_Window* window);

SDL_Event event;
bool debug = false;
bool sdl_render = true;
uint32_t framebuffer[160 * 144];
SDL_Window *window;
SDL_Renderer *renderer;
SDL_Texture *texture;

uint32_t palette[4] = {
    0xFF9A9E3F,  // Lightest (color 0)
    0xFF496B22,  // Light (color 1)
    0xFF0E450B,  // Dark (color 2)
    0xFF1B2A09   // Darkest (color 3)
};

struct sprite {
    uint8_t y, x, tile, flags;
};
uint8_t mem[2000000];

/*
 * Interrupt Master Enable flag
 * 
 * IME is a flag internal to the CPU that controls
 * whether any interrupt handlers are called,
 * regardless of the contents of IE. IME is only
 * modified by:
 * - ei instruction
 * - di instruction
 * - reti instruction (same as ei followed by ret)
 * - When an interrupt handler is executed.
 *
 * IME is unset when the game start running.
 */
bool IME = false;
bool ime_scheduled = false; 
/*
 * Interrupt Enable
 * - bit0: VBlank
 * - bit1: LCD
 * - bit2: Timer
 * - bit3: Serial
 * - bit4: Joypad
 * - bits 5-7: unused
 */
uint8_t *IE = &mem[0xFFFF];
/*
 * Interrupt Flag
 * - bit0: VBlank
 * - bit1: LCD
 * - bit2: Timer
 * - bit3: Serial
 * - bit4: Joypad
 * - bits 5-7: unused
 */
uint8_t *IF = &mem[0xFF0F];

// CPU registers
uint8_t A = 0;  // A
uint8_t F = 0; // Flags register (can't be accesed directly)
uint8_t B = 0; //BC high
uint8_t C = 0; //BC low
uint8_t D = 0; // DE high
uint8_t E = 0; // DE low
uint8_t H = 0; // HL high
uint8_t L = 0; // HL low
uint16_t SP;
uint16_t PC;

// PPU registers
uint8_t *LCDC = &mem[0xFF40];
uint8_t *LY = &mem[0xFF44];
uint8_t *STAT = &mem[0xFF41];
uint8_t *SCY = &mem[0xFF42];
uint8_t *SCX = &mem[0xFF43];

// APU registers

// FF25 — NR51: 

/*
 * Sound Panning
 * - bit0: CH1 right
 * - bit1: CH2 right
 * - bit2: CH3 right
 * - bit3: CH4 right
 * - bit4: CH1 left
 * - bit5: CH2 left
 * - bit6: CH3 left
 * - bit7: CH4 left
 */
uint8_t *NR51 = &mem[0xFF25];

/*
 * Audio master control
 * - bit0: Ch1 on? (read only)
 * - bit1: Ch2 on? (read only)
 * - bit2: Ch3 on? (read only)
 * - bit3: Ch4 on? (read only)
 * - bit4: 
 * - bit5: 
 * - bit6: 
 * - bit7: Audio On/Off
 **/
uint8_t *NR52 = &mem[0xFF26]; 

/*
 * NR11 - Channel 1 length timer & duty cycle
 * - 0-5: Initial length timer (write only)
 * - 6-7: Wave Duty
 **/
uint8_t *NR11 = &mem[0xFF11];

/*
 * NR12 - Channel 1 volume & envelope
 * - 0-2: Sweep pace
 * - 3  : env dir
 * - 4-7: Initial volume
 **/
uint8_t *NR12 = &mem[0xFF12];

/*
 * NR13 - Channel 1 period low
 **/
uint8_t *NR13 = &mem[0xFF13];

/*
 * NR14 - Channel 1 period high and control
 * - 0-2: period
 * - 6  : length enable
 * - 7  : Trigger
 **/
uint8_t *NR14 = &mem[0xFF14];

/*
 * NR21 - Channel 2 length timer & duty cycle
 * - 0-5: Initial length timer (write only)
 * - 6-7: Wave Duty
 **/
uint8_t *NR21 = &mem[0xFF16];

/*
 * NR22 - Channel 2 volume & envelope
 * - 0-2: Sweep pace
 * - 3  : env dir
 * - 4-7: Initial volume
 **/
uint8_t *NR22 = &mem[0xFF17];

/*
 * NR23 - Channel 2 period low
 **/
uint8_t *NR23 = &mem[0xFF18];

/*
 * NR24 - Channel 2 period high and control
 * - 0-2: period
 * - 6  : length enable
 * - 7  : Trigger
 **/
uint8_t *NR24 = &mem[0xFF19];


//FF00 — P1/JOYP: Joypad
uint8_t *JOYP = &mem[0xFF00];
bool right = false;
bool up = false;
bool down = false;
bool left = false;
bool sel = false;
bool a = false;
bool b = false;
bool start = false;

struct timespec frame_start, frame_end;

SDL_AudioDeviceID audio_device;
SDL_AudioSpec audio_spec;

int dots = 0;

bool channel1_playing = false;
uint8_t channel1_volume = 0;
float channel1_phase = 0;
float channel1_phase_increment = 0;

bool channel2_playing = false;
uint8_t channel2_volume = 0;
float channel2_phase = 0;
float channel2_phase_increment = 0;

float cycles_per_sample_counter = 0;

int main(int argc, char **argv) {
    A = 0x01;
    F = 0xB0;
    B = 0x00;
    C = 0x13;
    D = 0x00;
    E = 0xD8;
    H = 0x01;

    L = 0x4D;
    PC = 0x0100;
    SP = 0x0000;
    mem[0xFF40] = 0x91;  // LCDC - LCD ON!
    mem[0xFF47] = 0xFC;  // BGP palette
    mem[0xFF44] = 0;  // LY starts at 0
    SP = 0xFFFE;
    mem[0xFF00] = 0b11111111; //reset joypad
    if (argc < 2) {
        fprintf(stderr, "No rom file was provided\n");
        exit(1);
    }
    if (argc > 2 && strcmp(argv[2],"-d") == 0) {
        debug = true;
    } 
    if (argc > 3 && strcmp(argv[3],"-s") == 0) {
        sdl_render = false;
    } 
    FILE *rom_file = fopen(argv[1], "rb");
    if (rom_file == NULL) {
        fprintf(stderr, "Could not access file: %s\n", argv[1]);
        exit(1);
    }

    fseek(rom_file, 0, SEEK_END);
    uint32_t size = ftell(rom_file);
    rewind(rom_file);
    fread(mem, 1, size, rom_file);
    fclose(rom_file);

    printf("ROM file is %u bytes\n", size);
    show_cartridge_info();

    uint16_t cycles = 0;

    if (sdl_render) { 
        SDL_Init(SDL_INIT_VIDEO);
        window = SDL_CreateWindow(
                "GB Emulator",
                SDL_WINDOWPOS_CENTERED,
                SDL_WINDOWPOS_CENTERED,
                160 * 5, 144 * 5,
                SDL_WINDOW_SHOWN
                );


        renderer = SDL_CreateRenderer(window, -1, 0);
        texture = SDL_CreateTexture(
                renderer,
                SDL_PIXELFORMAT_ARGB8888,
                SDL_TEXTUREACCESS_STREAMING,
                160, 144
                ); 
    }


    SDL_Init(SDL_INIT_AUDIO);

    // the representation of our audio device in SDL:
    SDL_zero(audio_spec);
    audio_spec.freq = 44100;
    audio_spec.format = AUDIO_S16SYS;
    audio_spec.channels = 1;
    audio_spec.samples = 1024;
    audio_spec.callback = NULL;

    audio_device = SDL_OpenAudioDevice(NULL, 0, &audio_spec, NULL, 0);
    SDL_PauseAudioDevice(audio_device, 0);


    uint64_t frame_cycles = 0;

    while (true) {

        cycles = cpu_step();
        if (debug) show_registers();


        timer_step(cycles);

        if (IME == true) {
            handle_interrupts();
        }
        ppu_steps(cycles);

        apu_step(cycles);

        frame_cycles += cycles;
        if (frame_cycles >= CYCLES_PER_FRAME) {
            frame_cycles -= CYCLES_PER_FRAME;

            check_joyp();
            while (SDL_GetQueuedAudioSize(audio_device) > 4000 ) {
                SDL_Delay(1);
            }

        }
    }

    return EXIT_SUCCESS;
}

uint64_t timer_diff(struct timespec *start, struct timespec *end) {
    uint64_t difference = (uint64_t)(end->tv_sec - start->tv_sec) * 1000000000ULL;
    difference += (uint64_t)(end->tv_nsec - start->tv_nsec);
    return difference;
}
/*
   Game Boy resolution: 160x144
   A "dot" = 222 Hz (≅ 4.194 MHz) time unit. 4 dots / M-cycle
   A frame consists of 154 scanlines; during the first 144, the screen is drawn top to bottom, left to right.

   The cpu_step cycles are given as an argument, in order to know
   how many ticks to advance the PPU, as both units work in sync
   with a system clock.
   */

void ppu_steps(int cycles) {
    for (int i = 0; i < cycles; i++) {
        ppu_step();
    }
}

void ppu_step() {
    dots++; 

    static struct sprite intersecting_sprites[10];
    static int inter_sprite_len = 0;

    int height = (*LCDC & 0b00000100) ? 16 : 8; //LCDC bit 2 = 0 -> 8x8

    if (dots > 0 && dots <= 80) {
        // mode 2 - OAM scan
        // duration: 80 dots
        // description: Searching for OBJs which overlap this line (OAM inaccessible)
        // plan: Scan the 40 OAM entries to find which intersect with the current line
        // Object attributes reside in the object attribute memory (OAM) at $FE00-FE9F.
        // Each of the 40 entries consists of four bytes
        // Object’s vertical position on the screen + 16
        *STAT = (*STAT & 0b11111100) | 0b10;  // Mode 2
        if (dots == 1) { // do all work on dot 1, then wait for the remaining dots
            inter_sprite_len = 0;

            for (int i = 0; i < 40; i++) {

                if (inter_sprite_len == 10) {
                    if (debug) puts("more than 10 objects per line found. ignoring...");
                    break;
                }

                struct sprite object = { 
                    mem[0xFE00 + i * 4],
                    mem[0xFE00 + i * 4 + 1],
                    mem[0xFE00 + i * 4 + 2],
                    mem[0xFE00 + i * 4 + 3]
                };

                // determine intersection with LY
                int screen_y = object.y - 16;
                if (*LY >= screen_y && *LY < screen_y + height) {
                    intersecting_sprites[inter_sprite_len++] = object;
                }
            }
        }

    } else if (dots > 80 && dots <= 456) {
        // mode 3 - Drawing Pixels
        // duration: between 172 and 289
        // description: Sending pixels to the LCD (VRAM inaccessible)
        *STAT = (*STAT & 0b11111100) | 0b11;  // Mode 3

        if (dots == 81 && *LY < 144) {
            uint16_t bg_tilemap_addr;
            if ((*LCDC & 0b00001000) == 0) {
                bg_tilemap_addr = 0x9800;
            } else {
                bg_tilemap_addr = 0x9C00;
            }


            int y = (*SCY + *LY) & 0xFF;

            int tile_row = y / 8;
            int row_in_tile = y % 8; 

            //160 pixels / 8 pixels per tile = 20 tiles.
            for (int i = 0; i < 20; i++) {
                int map_x = (*SCX + (i * 8)) & 0xFF;

                int tile_col = map_x / 8;
                uint8_t tile_index = mem[bg_tilemap_addr + (tile_row * 32) + tile_col];

                uint16_t tile_addr;
                if (*LCDC & 0x10) {
                    tile_addr = 0x8000 + tile_index * 16;
                } else {
                    tile_addr = 0x9000 + ((int8_t)tile_index) * 16;
                }

                uint8_t bgbyte1 = mem[tile_addr + row_in_tile * 2];
                uint8_t bgbyte2 = mem[tile_addr + row_in_tile * 2 + 1];
                send_word_to_buffer(bgbyte1, bgbyte2, i * 8, false, 0);               
            }

            //mix background, window and objects
            //write scanline to SDL array
            // Y = Object’s vertical position on the screen + 16
            // X = Object’s horizontal position on the screen + 8
            //puts("rendering objects");
            //uint8_t pixel_prio_map[144] = {0};
            for (int i = inter_sprite_len - 1; i >=0; i--) {
                struct sprite object = intersecting_sprites[i];
                int row_in_sprite = *LY - (object.y - 16);

                //get pixels from tilemaps
                // 8x8  The index byte specifies the object’s only tile 
                //      index ($00-$FF). This unsigned value selects a 
                //      tile from the memory area at $8000-$8FFF
                // 8x16 The memory area at $8000-$8FFF is still interpreted 
                //      as a series of 8×8 tiles, where every 2 tiles form 
                //      an object. In this mode, this byte specifies 
                //      the index of the first (top) tile of the object.
                //
                //Data format in tile data
                //Each tile occupies 16 bytes, where each line is represented by 2 bytes
                //For each line, the first byte specifies the least significant bit of the 
                //color ID of each pixel, and the second byte specifies the most significant 
                //bit. In both bytes, bit 7 represents the leftmost pixel, and bit 0 the rightmost.
                if (height == 8) { 
                    uint8_t byte1 = mem[0x8000 + object.tile * 16 + row_in_sprite * 2];
                    uint8_t byte2 = mem[0x8000 + object.tile * 16 + row_in_sprite * 2 + 1];
                    send_word_to_buffer(byte1, byte2, object.x - 8, true, object.flags);               
                } else {
                    printf("Tall sprites detected!\n");
                    exit(1);
                }



            }
        }

        //artificial mode change for testing
        if (dots > 80 + 172) {
            // mode 0 - Horizontal Blank
            // duration: 376 - mode 3’s duration
            // description: Waiting until the end of the scanline
            *STAT = (*STAT & 0b11111100) | 0b00;  // Mode 0
        }
    }

    if (dots == 456) {
        dots = 0;
        (*LY)++;

        if (*LY == 144) { // mode 1 Vertical Blank (LY= [144,153])
                          // mode 1 - Vertical Blank
                          // duration: 4560 dots (10 scanlines)
                          // description: Waiting until the next frame
            *STAT = (*STAT & 0b11111100) | 0b01;  // Mode 1
            *IF |= 0b00000001;  // Request VBlank interrupt!
            show_frame();
        } else if (*LY > 153) {
            *LY = 0;
        }
    }
}

void send_word_to_buffer(uint8_t byte1, uint8_t byte2, int x, bool is_obj, uint8_t flags) {

    bool priority = flags & 0b10000000;
    bool y_flip = ((flags & 0b01000000) != 0) && is_obj;
    bool x_flip = ((flags & 0b00100000) != 0) && is_obj;

    for (int j = 0; j < 8; j++) { // 8 pixels from left to right

        int screen_x = x + (x_flip ? (7-j) : j);
        if (screen_x < 0 || screen_x >= 160) continue;
        uint8_t b_mask = 1 << (7 - j); // 00000001 -> 10000000 (7 shifts) 
        uint8_t lo = (byte1 & b_mask) >> (7 - j);
        uint8_t hi = (byte2 & b_mask) >> (7 - j);
        uint8_t color_code = lo | (hi << 1);
        if (is_obj == true && color_code == 0) continue;
        if (is_obj == true && priority == false) { // any bg color except 0 overwrites the object
            if (framebuffer[*LY * 160 + screen_x] != palette[0]){
                continue;
            }
        }
        framebuffer[*LY * 160 + screen_x] = palette[color_code];
    }
}
void show_frame() {
    SDL_UpdateTexture(texture, NULL, framebuffer, 160 * sizeof(uint32_t));
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);
}
/*
 * Tile data is stored in VRAM in the memory area at $8000-$97FF; 
 * with each tile taking 16 bytes, this area defines data for 384 tiles.
 */
uint8_t *get_tile(int index) {
    if (index * 16 + 0x8000 > 0x97F0) {
        puts("Invalid Tile index");
        exit(1);
    }
    static uint8_t tile[16];
    memcpy(&tile, mem + index * 16, 16);
    return tile;
}
bool halted = false;
int cpu_step(void) {
    if (ime_scheduled) {
        IME = true;
        ime_scheduled = false;
    }
    uint8_t n8 = 0;   // 8-bit integer signed or unsigned
    uint16_t n16 = 0; // 16-bit integer signed or unsigned
    int8_t e8 = 0;    // 8-bit signed offset!
    uint8_t a8 = 0;   // unsigned Offset added to 0xFF00
    uint16_t a16 = 0; // absolute offset, always unsigned
    uint8_t op = mem[PC];
    switch (op) {
        case 0x00: // NOP    b1 c4 flags:----
            if (debug) printf("NOP\n");
            PC += 1; 
            return 4;
        case 0x01: // LD BC, <n16>    b3 c12 flags:----
            B = mem[PC + 2];
            C = mem[PC + 1];
            n16 = C | (B << 8);
            if (debug) printf("LD BC, 0x%04X\n", n16);
            PC += 3;
            return 12;
        case 0x02: // LD [BC], A    b1 c8 flags:----
            if (debug) printf("LD [BC], A\n");
            mem_write8(BC, A);
            PC += 1;
            return 8;
        case 0x03: // INC BC    b1 c8 flags:----
            if (debug) printf("INC BC\n");
            inc_reg16(&C, &B);
            PC += 1;
            return 8;
        case 0x04: // INC B    b1 c4 flags:Z0H-
            if (debug) printf("INC B\n");
            if ((B & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            B++;
            if (B == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 1;
            return 4;
        case 0x05:  // DEC B    b1 c4 flags:Z1H-
            if (debug) printf("DEC B\n");
            if ((B & 0x0F) == 0x00) SETF_H; else CLRF_H;
            B--;
            if (B == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            PC += 1;
            return 4;
        case 0x06: // LD B, <n8>   b2 c8 flags:----
            B = mem[PC + 1];
            if (debug) printf("LD B, 0x%02X\n", B);
            PC += 2;
            return 8;
        case 0x07: // RLCA    b1 c4 flags:000C
            if (debug) printf("RLCA\n");
            if ((A & 0b10000000) != 0) SETF_C; else CLRF_C;
            A = (A << 1);
            if (READF_C) A = (A | 0x01);
            CLRF_Z; CLRF_N; CLRF_H;
            PC += 1;
            return 4;
        case 0x08: // LD [a16], SP    b3 c20 flags=----
            a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
            if (debug) printf("LD [0x%04X], SP\n", a16);
            mem_write8(a16, SP & 0xFF);
            mem_write8(a16 + 1, SP >> 8);
            PC += 3;
            return 20;
        case 0x09: // ADD HL, BC    b1 c8 flags:-0HC
            if (debug) printf("ADD HL, BC\n");
            {
                uint16_t hl = HL;
                uint32_t result = hl + BC;
                if (result > 0xFFFF) SETF_C; else CLRF_C;
                if (((hl & 0x0FFF) + (BC & 0x0FFF)) > 0x0FFF) SETF_H; else CLRF_H;
                CLRF_N;
                H = (result >> 8) & 0xFF;
                L = result & 0xFF;
            }
            PC += 1;
            return 8;
        case 0x0A:  // LD A, [BC]    b1 c8 flags:----
            if (debug) printf("LD A, [BC]\n");
            A = mem[C | (B << 8)];
            PC += 1;
            return 8;
        case 0x0B: // DEC BC    b1 c8 flags:----
            if (debug) printf("DEC BC\n");
            dec_reg16(&C, &B);
            PC += 1;
            return 8;
        case 0x0C: // INC C    b1 c4 flags:Z0HC
            if (debug) printf("INC C\n");
            if ((C & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            C++;
            if (C == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 1;
            return 4;
        case 0x0D: // DEC C    b1 c4 flags:Z1H-
            if (debug) printf("DEC C\n");
            if ((C & 0x0F) == 0x00) SETF_H; else CLRF_H;
            C--;
            if (C == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            PC += 1;
            return 4;
        case 0x0E: // LD C, <n8>    b2 c8 flags:----
            C = mem[PC + 1];
            if (debug) printf("LD C, 0x%02X\n", C);
            PC += 2;
            return 8;
        case 0x0F: // RRCA    b1 c4 flags:000C
            if (debug) printf("RRCA\n");
            CLRF_Z; CLRF_N; CLRF_H;
            if ((A & 0b00000001) == 0) CLRF_C; else SETF_C;
            A = (A >> 1) | (A << 7);         
            PC += 1;
            return 4;
        case 0x11: // LD DE, <n16>    b3 c12 flags:----
            n16 = mem[PC + 1] | (mem[PC + 2] << 8);
            if (debug) printf("LD DE, 0x%04X\n", n16);
            D = mem[PC + 2];
            E = mem[PC + 1];
            PC += 3;
            return 12;
        case 0x12: // LD [DE], A    b1 c8 flags:----
            if (debug) printf("LD [DE], A\n");
            mem_write8(DE, A);
            PC += 1;
            return 8;
        case 0x13: // INC DE    b1 c8 flags:---- 
            if (debug) printf("INC DE\n");
            inc_reg16(&E, &D);
            PC += 1;
            return 8;
        case 0x14: // INC D    b1 c4 flags:Z0H-
            if (debug) printf("INC D\n");
            if ((D & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            CLRF_N;
            D++;
            if (D == 0) SETF_Z; else CLRF_Z;
            PC += 1;
            return 4;
        case 0x15:  // DEC D    b1 c4 flags:Z1H-
            if (debug) printf("DEC D\n");
            if ((D & 0x0F) == 0x00) SETF_H; else CLRF_H;
            D--;
            if (D == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            PC += 1;
            return 4;
        case 0x16: // LD D, <n8>    b2 c8 flags:----
            n8 = mem[PC + 1];
            if (debug) printf("LD D, 0x%02X\n", n8);
            D = n8;
            PC += 2;
            return 8;
        case 0x17: // RLA    b1 c4 flags:000C
            {
                if (debug) printf("RLA\n");
                bool oldCarry = READF_C;
                if (A & 0b10000000) SETF_C; else CLRF_C;
                A = (A << 1);
                if (oldCarry) A |= 0x01;  
                CLRF_Z; CLRF_N; CLRF_H;
                PC += 1;
            }
            return 4;
        case 0x18: // JR <e8>    b2 c12 flags:----
            e8 = (int8_t)mem[PC + 1];
            if (debug) printf("JR 0x%04X\n", e8 + PC);
            PC += 2;
            PC += e8;
            return 12;
        case 0x19: // ADD HL, DE    b1 c8 flags:-0HC
            if (debug) printf("ADD HL, DE\n");
            {
                uint16_t hl = HL;
                uint32_t result = hl + DE;
                if (result > 0xFFFF) SETF_C; else CLRF_C;
                if (((hl & 0x0FFF) + (DE & 0x0FFF)) > 0x0FFF) SETF_H; else CLRF_H;
                CLRF_N;
                H = (result >> 8) & 0xFF;
                L = result & 0xFF;
            }
            PC += 1;
            return 8;
        case 0x1A:  // LD A, [DE]    b1 c8 flags:----
            if (debug) printf("LD A, [DE]\n");
            A = mem[E | (D << 8)];
            PC += 1;
            return 8;
        case 0x1B: // DEC DE    b1 c8 flags:----
            if (debug) printf("DEC DE\n");
            dec_reg16(&E, &D);
            PC += 1;
            return 8;
        case 0x1C: // INC E    b1 c4 flags:Z0H-
            if (debug) printf("INC E\n");
            if ((E & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            CLRF_N;
            E++;
            if (E == 0) SETF_Z; else CLRF_Z;
            PC += 1;
            return 4;
        case 0x1D:  // DEC E    b1 c4 flags:Z1H-
            if (debug) printf("DEC E\n");
            if ((E & 0x0F) == 0x00) SETF_H; else CLRF_H;
            E--;
            if (E == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            PC += 1;
            return 4;
        case 0x1E: // LD E, <n8>    b2 c8 flags:----
            n8 = mem[PC + 1];
            if (debug) printf("LD E, 0x%02X\n", n8);
            E = n8;
            PC += 2;
            return 8;
        case 0x1F: // RRA    b1 c4 flags:000C
            if (debug) printf("RRA\n");
            bool oldCarry = READF_C;  // Save old carry FIRST
            if (A & 0x01) SETF_C; else CLRF_C;  // Bit 0 -> new carry
            A = (A >> 1);
            if (oldCarry) A |= 0x80;  // Old carry -> bit 7
            CLRF_Z; CLRF_N; CLRF_H;
            PC += 1;
            return 4;
        case 0x20: // JR NZ, <e8>    b2 c12,8 flags:----
            e8 = (int8_t)mem[PC + 1];
            if (debug) printf("JR NZ, 0x%02X\n", e8);
            PC += 2;
            if (!READF_Z) {
                PC += e8;
                return 12;
            } else {
                return 8;
            }
            return 12; 
        case 0x21: // LD HL, <n16>    b3 c12 flags:----
            H = mem[PC + 2];
            L = mem[PC + 1];
            n16 = L | (H << 8);
            if (debug) printf("LD HL, 0x%04X\n", n16);
            PC += 3;
            return 12;
        case 0x22:  // LD [HL+], A    b1 c8 flags:----
            if (debug) printf("LD [HL+], A\n");
            mem_write8(L | (H << 8), A);
            inc_reg16(&L, &H);
            PC += 1;
            return 8;
        case 0x23: // INC HL    b1 c8 flags:----
            if (debug) printf("INC HL\n");
            inc_reg16(&L, &H);
            PC += 1;
            return 8;
        case 0x24:  // INC H    b1 c4 flags=Z0H- 
            if (debug) printf("INC H\n");
            if ((H & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            H++;
            if (H == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 1;
            return 4;
        case 0x25:  // DEC H    b1 c4 flags:Z1H-
            if (debug) printf("DEC H\n");
            if ((H & 0x0F) == 0x00) SETF_H; else CLRF_H;
            H--;
            if (H == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            PC += 1;
            return 4;
        case 0x26: // LD H, <n8>    b2 c8 flags:----
            H = mem[PC + 1];
            if (debug) printf("LD H, 0x%02X\n", B);
            PC += 2;
            return 8;
        case 0x27: // DAA    b1 c4 flags:Z-0C
            if (debug) printf("DAA\n");
            {
                uint8_t correction = 0;

                if (READF_N) {
                    // After subtraction
                    if (READF_H) correction |= 0x06;
                    if (READF_C) correction |= 0x60;
                    A -= correction;
                } else {
                    // After addition
                    if (READF_H || (A & 0x0F) > 0x09) correction |= 0x06;
                    if (READF_C || A > 0x99) {
                        correction |= 0x60;
                        SETF_C;
                    }
                    A += correction;
                }

                if (A == 0) SETF_Z; else CLRF_Z;
                CLRF_H;
            }
            PC += 1;
            return 4;
        case 0x28: // JR Z, <e8>    b2 c12,8 flags:----
            e8 = (int8_t)mem[PC + 1];
            if (debug) printf("JR Z, 0x%02X\n", e8);
            PC += 2;
            if (READF_Z) {
                PC += e8;
                return 12;
            } else {
                return 8;
            }
            return 12; 
        case 0x29: // ADD HL, HL    b1 c8 flags:-0HC
            if (debug) printf("ADD HL, HL\n");
            {
                uint16_t hl = HL;
                uint32_t result = hl + hl;
                if (result > 0xFFFF) SETF_C; else CLRF_C;
                if (((hl & 0x0FFF) + (hl & 0x0FFF)) > 0x0FFF) SETF_H; else CLRF_H;
                CLRF_N;
                // Z flag is NOT affected
                H = (result >> 8) & 0xFF;
                L = result & 0xFF;
            }
            PC += 1;
            return 8;
        case 0x2A:  // LD A, [HL+]    b1 c8 flags:----
            if (debug) printf("LD A, [HL+]\n");
            A = mem[L | (H << 8)];
            inc_reg16(&L, &H);
            PC += 1;
            return 8;
        case 0x2B: // DEC HL    b1 c8 flags:----
            if (debug) printf("DEC HL\n");
            dec_reg16(&L, &H);
            PC += 1;
            return 8;
        case 0x2C:  // INC L    b1 c4 flags:Z0H- 
            if (debug) printf("INC L\n");
            if ((L & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            L++;
            if (L == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 1;
            return 4;
        case 0x2D:  // DEC L    b1 c4 flags:Z1H-
            if (debug) printf("DEC L\n");
            if ((L & 0x0F) == 0x00) SETF_H; else CLRF_H;
            L--;
            if (L == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            PC += 1;
            return 4;
        case 0x2E: // LD L, <n8>    b2 c8 flags:----
            L = mem[PC + 1];
            if (debug) printf("LD L, 0x%02X\n", mem[PC + 1]);
            PC += 2;
            return 8;
        case 0x2F: // CPL    b1 c4 flags:-11-
            if (debug) printf("CPL\n");
            A = ~A;
            SETF_N; SETF_H;
            PC += 1;
            return 4;
        case 0x30: // JR NC, <e8>    b2 c12,8 flags:----
            e8 = (int8_t)mem[PC + 1];
            if (debug) printf("JR NC, 0x%04X\n", PC + 2 + e8);
            PC += 2;
            if (!READF_C) {
                PC += e8;
                return 12;
            }
            return 8;
        case 0x31: // LD SP, <n16>    b3 c12 flags:----
            n16 = mem[PC + 1] | (mem[PC + 2] << 8);
            if (debug) printf("LD SP, 0x%04X\n", n16);
            SP = n16;
            PC += 3;
            return 12;
        case 0x32: // LD [HL-], A    b1 c8 flags:----
            if (debug) printf("LD [HL-], A\n");
            mem_write8(L | (H << 8), A); 
            dec_reg16(&L, &H);
            PC += 1;
            return 8;
        case 0x33: // INC SP    b1 c8 flags:----
            if (debug) printf("INC SP\n");
            SP++;
            PC += 1;
            return 8;
        case 0x34: // INC [HL]    b1 c12 flags:Z0H-
            if (debug) printf("INC [HL]\n");
            mem_write8(HL, mem[HL] + 1);
            if (mem[HL] == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            if ((mem[HL] & 0x0F) == 0x00) SETF_H; else CLRF_H;
            PC += 1;
            return 12;
        case 0x35: {
                       if (debug) printf("DEC [HL]\n");
                       uint8_t val = mem[HL] - 1;
                       mem_write8(HL, val);
                       if (val == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       if ((val & 0x0F) == 0x0F) SETF_H; else CLRF_H;
                       PC += 1;
                       return 12;
                   }
        case 0x36: // LD [HL], <n8>    b2 c12 flags:----
                   n8 = mem[PC + 1];
                   if (debug) printf("LD [HL], 0x%02X\n", n8);
                   mem_write8(HL, n8);
                   PC += 2;
                   return 12;
        case 0x38: // JR C, <e8>    b2 c12,8 flags:----
                   e8 = (int8_t)mem[PC + 1];
                   if (debug) printf("JR C, 0x%02X\n", e8);
                   PC += 2;
                   if (READF_C) {
                       PC += e8;
                       return 12;
                   } 
                   return 8;
        case 0x39: // ADD HL, SP    b1 c8 flags:-0HC
                   if (debug) printf("ADD HL, SP\n");
                   {
                       uint16_t hl = HL;
                       uint32_t result = hl + SP;
                       if (result > 0xFFFF) SETF_C; else CLRF_C;
                       if (((hl & 0x0FFF) + (SP & 0x0FFF)) > 0x0FFF) SETF_H; else CLRF_H;
                       CLRF_N;
                       H = (result >> 8) & 0xFF;
                       L = result & 0xFF;
                   }
                   PC += 1;
                   return 8;
        case 0x3A:  // LD A, [HL-]    b1 c8 flags:----
                   if (debug) printf("LD A, [HL-]\n");
                   A = mem[L | (H << 8)];
                   dec_reg16(&L, &H);
                   PC += 1;
                   return 8;
        case 0x3B: // DEC SP    b1 c8 flags:----
                   if (debug) printf("DEC SP\n");
                   SP--;
                   PC += 1;
                   return 8;
        case 0x3C: // INC A    b1 c4 flags:Z0H-
                   if (debug) printf("INC A\n");
                   if ((A & 0x0F) == 0x0F) SETF_H; else CLRF_H;
                   A++;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 4;
        case 0x3D:  // DEC A    b1 c4 flags:Z1H-
                   if (debug) printf("DEC A\n");
                   if ((A & 0x0F) == 0x00) SETF_H; else CLRF_H;
                   A--;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   SETF_N;
                   PC += 1;
                   return 4;
        case 0x3E: // LD A, <n8>    b2 c8 flags:----
                   A = mem[PC + 1];
                   if (debug) printf("LD A, 0x%02X\n", mem[PC + 1]);
                   PC += 2;
                   return 8;
        case 0x37: // SCF    b1 c4 flags:-001
                   if (debug) printf("SCF\n");
                   CLRF_N; CLRF_H;
                   SETF_C;
                   PC += 1;
                   return 4;
        case 0x3F: // CCF    b1 c4 flags:-00~C
                   if (debug) printf("CCF\n");
                   if (READF_C) CLRF_C; else SETF_C;
                   CLRF_N; CLRF_H;
                   PC += 1;
                   return 4;
        case 0x40: // LD B, B    b1, c4 flags:----
                   if (debug) printf("LD B, B\n"); //essentially a NOP
                   PC += 1;
                   return 4;
        case 0x41: // LD B, C    b1 c4 flags:----
                   if (debug) printf("LD B, C\n");
                   B = C;
                   PC += 1;
                   return 4;
        case 0x42: // LD B, D    b1 c4 flags:----
                   if (debug) printf("LD B, D\n");
                   B = D;
                   PC += 1;
                   return 4;
        case 0x43: // LD B, E    b1 c4 flags:----
                   if (debug) printf("LD B, E\n");
                   B = E;
                   PC += 1;
                   return 4;
        case 0x44: // LD B, H    b1 c4 flags:----
                   if (debug) printf("LD B, H\n");
                   B = H;
                   PC += 1;
                   return 4;
        case 0x45: // LD B, L    b1 c4 flags:----
                   if (debug) printf("LD B, L\n");
                   B = L;
                   PC += 1;
                   return 4;
        case 0x46: // LD B, [HL]    b1 c8 flags:----
                   if (debug) printf("LD B, [HL]\n");
                   B = mem[HL];
                   PC += 1;
                   return 8;
        case 0x47: // LD B, A    b1 c4 flags:----
                   if (debug) printf("LD B, A\n");
                   B = A;
                   PC += 1;
                   return 4;
        case 0x48: // LD C, B    b1 c4 flags:----
                   if (debug) printf("LD C, B\n");
                   C = B;
                   PC += 1;
                   return 4;
        case 0x49: // LD C, C    b1 c4 flags:----
                   if (debug) printf("LD C, C\n");
                   PC += 1;
                   return 4;
        case 0x4A: // LD C, D    b1 c4 flags:----
                   if (debug) printf("LD C, D\n");
                   C = D;
                   PC += 1;
                   return 4;
        case 0x4B: // LD C, E    b1 c4 flags:----
                   if (debug) printf("LD C, E\n");
                   C = E;
                   PC += 1;
                   return 4;
        case 0x4C: // LD C, H    b1 c4 flags:----
                   if (debug) printf("LD C, H\n");
                   C = H;
                   PC += 1;
                   return 4;
        case 0x4D: // LD C, L    b1 c4 flags:----
                   if (debug) printf("LD C, L\n");
                   C = L;
                   PC += 1;
                   return 4;
        case 0x4E: // LD C, [HL]    b1 c8 flags:----
                   if (debug) printf("LD C, [HL]\n");
                   C = mem[HL];
                   PC += 1;
                   return 8;
        case 0x4F: // LD C, A    b1 c4 flags:----
                   if (debug) printf("LD C, A\n");
                   C = A;
                   PC += 1;
                   return 4;
        case 0x50: // LD D, B    b1 c4 flags:----
                   if (debug) printf("LD D, B\n");
                   D = B;
                   PC += 1;
                   return 4;
        case 0x51: // LD D, C    b1 c4 flags:----
                   if (debug) printf("LD D, C\n");
                   D = C;
                   PC += 1;
                   return 4;
        case 0x52: // LD D, D    b1 c4 flags:----
                   if (debug) printf("LD D, D\n");
                   PC += 1;
                   return 4;
        case 0x53: // LD D, E    b1 c4 flags:----
                   if (debug) printf("LD D, E\n");
                   D = E;
                   PC += 1;
                   return 4;
        case 0x54: // LD D, H    b1 c4 flags:----
                   if (debug) printf("LD D, H\n");
                   D = H;
                   PC += 1;
                   return 4;
        case 0x55: // LD D, L    b1 c4 flags:----
                   if (debug) printf("LD D, L\n");
                   D = L;
                   PC += 1;
                   return 4;
        case 0x56: // LD D, [HL]    b1 c8 flags:----
                   if (debug) printf("LD D, [HL]\n");
                   D = mem[HL];
                   PC += 1;
                   return 8;
        case 0x57: // LD D, A    b1 c4 flags:----
                   if (debug) printf("LD D, A\n");
                   D = A;
                   PC += 1;
                   return 4;
        case 0x58: // LD E, B    b1 c4 flags:----
                   if (debug) printf("LD E, B\n");
                   E = B;
                   PC += 1;
                   return 4;
        case 0x59: // LD E, C    b1 c4 flags:----
                   if (debug) printf("LD E, C\n");
                   E = C;
                   PC += 1;
                   return 4;
        case 0x5A: // LD E, D    b1 c4 flags:----
                   if (debug) printf("LD E, D\n");
                   E = D;
                   PC += 1;
                   return 4;
        case 0x5B: // LD E, E    b1 c4 flags:----
                   if (debug) printf("LD E, E\n");
                   PC += 1;
                   return 4;
        case 0x5C: // LD E, H    b1 c4 flags:----
                   if (debug) printf("LD E, H\n");
                   E = H;
                   PC += 1;
                   return 4;
        case 0x5D: // LD E, L    b1 c4 flags:----
                   if (debug) printf("LD E, L\n");
                   E = L;
                   PC += 1;
                   return 4;
        case 0x5E: // LD E, [HL]    b1 c8 flags:----
                   if (debug) printf("LD E, [HL]\n");
                   E = mem[HL];
                   PC += 1;
                   return 8;
        case 0x5F: // LD E, A b1 c4 flags:----
                   if (debug) printf("LD E, A\n");
                   E = A;
                   PC += 1;
                   return 4;
        case 0x60: // LD H, B    b1 c4 flags:----
                   if (debug) printf("LD H, B\n");
                   H = B;
                   PC += 1;
                   return 4;
        case 0x61: // LD H, C    b1 c4 flags:----
                   if (debug) printf("LD H, C\n");
                   H = C;
                   PC += 1;
                   return 4;
        case 0x62: // LD H, D    b1 c4 flags:----
                   if (debug) printf("LD H, D\n");
                   H = D;
                   PC += 1;
                   return 4;
        case 0x63: // LD H, E    b1 c4 flags:----
                   if (debug) printf("LD H, E\n");
                   H = E;
                   PC += 1;
                   return 4;
        case 0x64: // LD H, H    b1 c4 flags:----
                   if (debug) printf("LD H, H\n");
                   PC += 1;
                   return 4;
        case 0x65: // LD H, L    b1 c4 flags:----
                   if (debug) printf("LD H, L\n");
                   H = L;
                   PC += 1;
                   return 4;
        case 0x66: // LD H, [HL]    b1 c8 flags:----
                   if (debug) printf("LD H, [HL]\n");
                   H = mem[HL];
                   PC += 1;
                   return 8;
        case 0x67: // LD H, A    b1 c4 flags:----
                   if (debug) printf("LD H, A\n");
                   H = A;
                   PC += 1;
                   return 4;
        case 0x68: // LD L, B    b1 c4 flags:----
                   if (debug) printf("LD L, B\n");
                   L = B;
                   PC += 1;
                   return 4;
        case 0x69: // LD L, C    b1 c4 flags:----
                   if (debug) printf("LD L, C\n");
                   L = C;
                   PC += 1;
                   return 4;
        case 0x6A: // LD L, D    b1 c4 flags:----
                   if (debug) printf("LD L, D\n");
                   L = D;
                   PC += 1;
                   return 4;
        case 0x6B: // LD L, E    b1 c4 flags:----
                   if (debug) printf("LD L, E\n");
                   L = E;
                   PC += 1;
                   return 4;
        case 0x6C: // LD L, H    b1 c4 flags:----
                   if (debug) printf("LD L, H\n");
                   L = H;
                   PC += 1;
                   return 4;
        case 0x6D: // LD L, L    b1 c4 flags:----
                   if (debug) printf("LD L, L\n");
                   PC += 1;
                   return 4;
        case 0x6E: // LD L, [HL]    b1 c8 flags:----
                   if (debug) printf("LD L, [HL]\n");
                   L = mem[HL];
                   PC += 1;
                   return 8;
        case 0x6F: // LD L, A  b1 c4 flags:----
                   if (debug) printf("LD L, A\n");
                   L = A;
                   PC += 1;
                   return 4;
        case 0x70: // LD [HL], B    b1 c8 flags:----
                   if (debug) printf("LD [HL], B\n");
                   mem_write8(L | (H << 8), B); 
                   PC += 1;
                   return 8;
        case 0x71: // LD [HL], C    b1 c8 flags:----
                   if (debug) printf("LD [HL], C\n");
                   mem_write8(L | (H << 8), C); 
                   PC += 1;
                   return 8;
        case 0x72: // LD [HL], D    b1 c8 flags:----
                   if (debug) printf("LD [HL], D\n");
                   mem_write8(L | (H << 8), D); 
                   PC += 1;
                   return 8;
        case 0x73: // LD [HL], E   b1 c8 flags:----
                   if (debug) printf("LD [HL], E\n");
                   mem_write8(L | (H << 8), E); 
                   PC += 1;
                   return 8;
        case 0x74: // LD [HL], H   b1 c8 flags:----
                   if (debug) printf("LD [HL], H\n");
                   mem_write8(L | (H << 8), H); 
                   PC += 1;
                   return 8;
        case 0x75: // LD [HL], L   b1 c8 flags:----
                   if (debug) printf("LD [HL], L\n");
                   mem_write8(L | (H << 8), L); 
                   PC += 1;
                   return 8;
        case 0x76: // HALT    b1 c4 flags:----
                   if (debug) printf("HALT\n");
                   if (halted) {
                       if (((*IF & *IE) != 0) || IME == true) {
                           halted = false;
                           PC += 1;
                           return 4;
                       }
                   }
                   halted = true;
                   return 4;
        case 0x77: // LD [HL], A    b1 c8 flags:----
                   if (debug) printf("LD [HL], A\n");
                   mem_write8(L | (H << 8), A); 
                   PC += 1;
                   return 8;
        case 0x78: // LD A, B    b1 c4 flags:----
                   if (debug) printf("LD A, B\n");
                   A = B;
                   PC += 1;
                   return 4;
        case 0x79: // LD A, C    b1 c4 flags:----
                   if (debug) printf("LD A, C\n");
                   A = C;
                   PC += 1;
                   return 4;
        case 0x7A: // LD A, D    b1 c4 flags:----
                   if (debug) printf("LD A, D\n");
                   A = D;
                   PC += 1;
                   return 4;
        case 0x7B: // LD A, E    b1 c4 flags:----
                   if (debug) printf("LD A, E\n");
                   A = E;
                   PC += 1;
                   return 4;
        case 0x7C: // LD A, H    b1 c4 flags:----
                   if (debug) printf("LD A, H\n");
                   A = H;
                   PC += 1;
                   return 4;
        case 0x7D: // LD A, L    b1 c4 flags:----
                   if (debug) printf("LD A, L\n");
                   A = L;
                   PC += 1;
                   return 4;
        case 0x7E: // LD A, [HL]    b1 c8 flags:----
                   if (debug) printf("LD A, [HL]\n");
                   A = mem[HL];
                   PC += 1;
                   return 8;
        case 0x7F: // LD A, A    b1 c4 flags:----
                   if (debug) printf("LD A, A\n");
                   PC += 1;
                   return 4;
        case 0x80: // ADD A, B    b1 c4 flags:Z0HC
                   if (debug) printf("ADD A, B\n");
                   if (A + B > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (B & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += B;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 4;
        case 0x81: // ADD A, C    b1 c4 flags:Z0HC
                   if (debug) printf("ADD A, C\n");
                   if (A + C > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (C & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += C;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 4;
        case 0x82: // ADD A, D    b1 c4 flags:Z0HC
                   if (debug) printf("ADD A, D\n");
                   if (A + D > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (D & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += D;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 4;
        case 0x83: // ADD A, E    b1 c4 flags:Z0HC
                   if (debug) printf("ADD A, E\n");
                   if (A + E > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (E & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += E;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 4;
        case 0x84: // ADD A, H    b1 c4 flags:Z0HC
                   if (debug) printf("ADD A, H\n");
                   if (A + H > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (H & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += H;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 4;
        case 0x85: // ADD A, L    b1 c4 flags:Z0HC
                   if (debug) printf("ADD A, L\n");
                   if (A + L > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (L & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += L;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 4;
        case 0x86: // ADD A, [HL]    b1 c8 flags:Z0HC
                   n8 = mem[HL];
                   if (debug) printf("ADD A, [HL]\n");
                   if (A + n8 > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (n8 & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += n8;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 8;
        case 0x87: // ADD A, A    b1 c4 flags:Z0HC
                   if (debug) printf("ADD A, A\n");
                   if (A + A > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (A & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += A;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 1;
                   return 4;
        case 0x88: { // ADC A, B    b1 c4 flags:Z0HC
                       if (debug) printf("ADC A, B\n");
                       uint8_t carry = READF_C;
                       uint16_t result = A + B + carry;
                       if (((A & 0x0F) + (B & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 1;
                       return 4;
                   }
        case 0x89: { // ADC A, C    b1 c4 flags:Z0HC
                       if (debug) printf("ADC A, C\n");
                       uint8_t carry = READF_C;
                       uint16_t result = A + C + carry;
                       if (((A & 0x0F) + (C & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 1;
                       return 4;
                   }
        case 0x8A: { // ADC A, D    b1 c4 flags:Z0HC
                       if (debug) printf("ADC A, D\n");
                       uint8_t carry = READF_C;
                       uint16_t result = A + D + carry;
                       if (((A & 0x0F) + (D & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 1;
                       return 4;
                   }
        case 0x8B: { // ADC A, E    b1 c4 flags:Z0HC
                       if (debug) printf("ADC A, E\n");
                       uint8_t carry = READF_C;
                       uint16_t result = A + E + carry;
                       if (((A & 0x0F) + (E & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 1;
                       return 4;
                   }
        case 0x8C: { // ADC A, H    b1 c4 flags:Z0HC
                       if (debug) printf("ADC A, H\n");
                       uint8_t carry = READF_C;
                       uint16_t result = A + H + carry;
                       if (((A & 0x0F) + (H & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 1;
                       return 4;
                   }
        case 0x8D: { // ADC A, L    b1 c4 flags:Z0HC
                       if (debug) printf("ADC A, L\n");
                       uint8_t carry = READF_C;
                       uint16_t result = A + L + carry;
                       if (((A & 0x0F) + (L & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 1;
                       return 4;
                   }
        case 0x8E: { // ADC A, [HL]    b1 c8 flags:Z0HC
                       n8 = mem[HL];
                       if (debug) printf("ADC A, [HL]");
                       uint8_t carry = (READF_C) ? 1 : 0;
                       uint16_t result = A + n8 + carry;
                       if (((A & 0x0F) + (n8 & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 1;
                       return 8;
                   }
        case 0x8F: { // ADC A, A    b1 c4 flags:Z0HC
                       if (debug) printf("ADC A, A\n");
                       uint8_t carry = READF_C;
                       uint16_t result = A + A + carry;
                       if (((A & 0x0F) + (A & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 1;
                       return 4;
                   }
        case 0x90: // SUB A, B    b1 c4 flags:Z1HC
                   if (debug) printf("SUB A, B\n");
                   if (B > A) SETF_C; else CLRF_C;
                   if ((B & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   SETF_N;
                   A -= B;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   PC += 1;
                   return 4;
        case 0x91: // SUB A, C    b1 c4 flags:Z1HC
                   if (debug) printf("SUB A, C\n");
                   if (C > A) SETF_C; else CLRF_C;
                   if ((C & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   SETF_N;
                   A -= C;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   PC += 1;
                   return 4;
        case 0x92: // SUB A, D    b1 c4 flags:Z1HC
                   if (debug) printf("SUB A, D\n");
                   if (D > A) SETF_C; else CLRF_C;
                   if ((D & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   SETF_N;
                   A -= D;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   PC += 1;
                   return 4;
        case 0x93: // SUB A, E    b1 c4 flags:Z1HC
                   if (debug) printf("SUB A, E\n");
                   if (E > A) SETF_C; else CLRF_C;
                   if ((E & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   SETF_N;
                   A -= E;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   PC += 1;
                   return 4;
        case 0x94: // SUB A, H    b1 c4 flags:Z1HC
                   if (debug) printf("SUB A, H\n");
                   if (H > A) SETF_C; else CLRF_C;
                   if ((H & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   SETF_N;
                   A -= H;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   PC += 1;
                   return 4;
        case 0x95: // SUB A, L    b1 c4 flags:Z1HC
                   if (debug) printf("SUB A, L\n");
                   if (L > A) SETF_C; else CLRF_C;
                   if ((L & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   SETF_N;
                   A -= L;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   PC += 1;
                   return 4;
        case 0x96: // SUB A, [HL]    b1 c8 flags:Z1HC
                   n8 = mem[HL];
                   if (debug) printf("SUB A, [HL]\n");
                   if (n8 > A) SETF_C; else CLRF_C;
                   if ((n8 & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   SETF_N;
                   A -= n8;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   PC += 1;
                   return 8;
        case 0x97: // SUB A, A    b1 c4 flags:1100
                   if (debug) printf("SUB A, A\n");
                   SETF_Z; SETF_N; CLRF_H; CLRF_C;
                   A -= A;
                   PC += 1;
                   return 4;
        case 0x98: // SBC A, B    b1 c4 flags:Z1HC
                   {
                       if (debug) printf("SBC A, B\n");
                       uint8_t carry = READF_C;
                       if (B + carry > A) SETF_C; else CLRF_C;
                       if ((B & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                       A = A - B - carry;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       PC += 1;
                   }
                   return 4;
        case 0x99: // SBC A, C    b1 c4 flags:Z1HC
                   {
                       if (debug) printf("SBC A, C\n");
                       uint8_t carry = READF_C;
                       if (C + carry > A) SETF_C; else CLRF_C;
                       if ((C & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                       A = A - C - carry;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       PC += 1;
                   }
                   return 4;
        case 0x9A: // SBC A, D    b1 c4 flags:Z1HC
                   {
                       if (debug) printf("SBC A, D\n");
                       uint8_t carry = READF_C;
                       if (D + carry > A) SETF_C; else CLRF_C;
                       if ((D & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                       A = A - D - carry;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       PC += 1;
                   }
                   return 4;
        case 0x9B: // SBC A, E    b1 c4 flags:Z1HC
                   {
                       if (debug) printf("SBC A, E\n");
                       uint8_t carry = READF_C;
                       if (E + carry > A) SETF_C; else CLRF_C;
                       if ((E & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                       A = A - E - carry;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       PC += 1;
                   }
                   return 4;
        case 0x9C: // SBC A, H    b1 c4 flags:Z1HC
                   {
                       if (debug) printf("SBC A, H\n");
                       uint8_t carry = READF_C;
                       if (H + carry > A) SETF_C; else CLRF_C;
                       if ((H & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                       A = A - H - carry;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       PC += 1;
                   }
                   return 4;
        case 0x9D: // SBC A, L    b1 c4 flags:Z1HC
                   {
                       if (debug) printf("SBC A, L\n");
                       uint8_t carry = READF_C;
                       if (L + carry > A) SETF_C; else CLRF_C;
                       if ((L & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                       A = A - L - carry;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       PC += 1;
                   }
                   return 4;
        case 0x9E: // SBC A, [HL]    b1 c8 flags:Z1HC
                   n8 = mem[HL];
                   if (debug) printf("SBC A, [HL]\n");
                   uint8_t carry = READF_C ? 1 : 0;
                   if (n8 + carry > A) SETF_C; else CLRF_C;
                   if ((n8 & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                   A = A - n8 - carry;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   SETF_N;
                   PC += 1;
                   return 8;
        case 0x9F: // SBC A, A    b1 c4 flags:Z1H-
                   {
                       if (debug) printf("SBC A, A\n");
                       uint8_t carry = READF_C;
                       if ((A & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                       A = A - A - carry;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       PC += 1;
                   }
                   return 4;
        case 0xA0: // AND A, B    b1 c4 flags:Z010
                   if (debug) printf("AND A, B\n");
                   A = A & B;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xA1: // AND A, C    b1 c4 flags:Z010
                   if (debug) printf("AND A, C\n");
                   A = A & C;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xA2: // AND A, D    b1 c4 flags:Z010
                   if (debug) printf("AND A, D\n");
                   A = A & D;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xA3: // AND A, E    b1 c4 flags:Z010
                   if (debug) printf("AND A, E\n");
                   A = A & E;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xA4: // AND A, H    b1 c4 flags:Z010
                   if (debug) printf("AND A, H\n");
                   A = A & H;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xA5: // AND A, L    b1 c4 flags:Z010
                   if (debug) printf("AND A, L\n");
                   A = A & L;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xA6: // AND A, [HL]    b1 c8 flags:Z010
                   n8 = mem[HL];
                   if (debug) printf("AND A, [HL]\n");
                   A = A & n8;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 1;
                   return 8;
        case 0xA7: // AND A, A    b1 c4 flags:Z010
                   if (debug) printf("AND A, A\n");
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xA8: // XOR A, B    b1 c4 flags:Z000
                   if (debug) printf("XOR A, B\n");
                   A = A ^ B;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xA9: // XOR A, C    b1 c4 flags:Z000
                   if (debug) printf("XOR A, C\n");
                   A = A ^ C;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xAA: // XOR A, D    b1 c4 flags:Z000
                   if (debug) printf("XOR A, D\n");
                   A = A ^ D;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xAB: // XOR A, E    b1 c4 flags:Z000
                   if (debug) printf("XOR A, E\n");
                   A = A ^ E;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xAC: // XOR A, H    b1 c4 flags:Z000
                   if (debug) printf("XOR A, H\n");
                   A = A ^ H;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xAD: // XOR A, L    b1 c4 flags:Z000
                   if (debug) printf("XOR A, L\n");
                   A = A ^ L;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xAE: // XOR A, [HL]    b1 c8 flags:Z000
                   if (debug) printf("XOR A, [HL]\n");
                   A = A ^ mem[HL];
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 8;
        case 0xAF: // XOR A, A    b1 c4 flags:1000
                   A = 0;
                   SETF_Z;
                   CLRF_N;
                   CLRF_H;
                   CLRF_C;
                   if (debug) printf("XOR A, A\n");
                   PC += 1;
                   return 4;
        case 0xB0: // OR A, B   b1 c4 flags:Z000
                   if (debug) printf("OR A, B\n");
                   A = A | B;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xB1: // OR A, C    b1 c4 flags:Z000
                   if (debug) printf("OR A, C\n");
                   A = A | C;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   CLRF_H;
                   CLRF_C;
                   PC += 1;
                   return 4;
        case 0xB2: // OR A, D   b1 c4 flags:Z000
                   if (debug) printf("OR A, D\n");
                   A = A | D;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xB3: // OR A, E   b1 c4 flags:Z000
                   if (debug) printf("OR A, E\n");
                   A = A | E;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xB4: // OR A, H   b1 c4 flags:Z000
                   if (debug) printf("OR A, H\n");
                   A = A | H;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xB5: // OR A, L   b1 c4 flags:Z000
                   if (debug) printf("OR A, L\n");
                   A = A | L;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xB6: // OR A, [HL]    b1 c8 flags:Z000
                   if (debug) printf("OR A, [HL]\n");
                   A = A | mem[HL];
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 8;
        case 0xB7: // OR A, A    b1 c4 flags:Z000
                   if (debug) printf("OR A, A\n");
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xB8: // CP A, B    b1 c4 flags:Z1HC
                   if (debug) printf("CP A, B\n");
                   if (A == B) SETF_Z; else CLRF_Z;
                   SETF_N;
                   if ((A & 0x0F) < (B & 0x0F)) SETF_H; else CLRF_H;
                   if (A < B) SETF_C; else CLRF_C;
                   PC += 1;
                   return 4;
        case 0xB9: // CP A, C    b1 c4 flags:Z1HC
                   if (debug) printf("CP A, C\n");
                   if (A == C) SETF_Z; else CLRF_Z;
                   SETF_N;
                   if ((A & 0x0F) < (C & 0x0F)) SETF_H; else CLRF_H;
                   if (A < C) SETF_C; else CLRF_C;
                   PC += 1;
                   return 4;
        case 0xBA: // CP A, D    b1 c4 flags:Z1HC
                   if (debug) printf("CP A, D\n");
                   if (A == D) SETF_Z; else CLRF_Z;
                   SETF_N;
                   if ((A & 0x0F) < (D & 0x0F)) SETF_H; else CLRF_H;
                   if (A < D) SETF_C; else CLRF_C;
                   PC += 1;
                   return 4;
        case 0xBB: // CP A, E    b1 c4 flags:Z1HC
                   if (debug) printf("CP A, E\n");
                   if (A == E) SETF_Z; else CLRF_Z;
                   SETF_N;
                   if ((A & 0x0F) < (E & 0x0F)) SETF_H; else CLRF_H;
                   if (A < E) SETF_C; else CLRF_C;
                   PC += 1;
                   return 4;
        case 0xBC: // CP A, H    b1 c4 flags:Z1HC
                   if (debug) printf("CP A, H\n");
                   if (A == H) SETF_Z; else CLRF_Z;
                   SETF_N;
                   if ((A & 0x0F) < (H & 0x0F)) SETF_H; else CLRF_H;
                   if (A < H) SETF_C; else CLRF_C;
                   PC += 1;
                   return 4;
        case 0xBD: // CP A, L    b1 c4 flags:Z1HC
                   if (debug) printf("CP A, L\n");
                   if (A == L) SETF_Z; else CLRF_Z;
                   SETF_N;
                   if ((A & 0x0F) < (L & 0x0F)) SETF_H; else CLRF_H;
                   if (A < L) SETF_C; else CLRF_C;
                   PC += 1;
                   return 4;
        case 0xBE: // CP A, [HL]    b1 c8 flags:Z1HC
                   n8 = mem[HL];
                   if (debug) printf("CP A, [HL]\n");
                   if (n8 == A) SETF_Z; else CLRF_Z;
                   SETF_N;
                   if ((n8 & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   if (n8 > A) SETF_C; else CLRF_C;
                   PC += 1;
                   return 8;
        case 0xBF: // CP A, A    b1 c4 flags:1100
                   if (debug) printf("CP A, A\n");
                   SETF_Z; SETF_N;
                   CLRF_H; CLRF_C;
                   PC += 1;
                   return 4;
        case 0xC0: // RET NZ b1 c20, 8 flags:----
                   if (debug) printf("RET NZ\n");
                   if (!READF_Z) {
                       uint8_t low5 = mem[SP];
                       uint8_t high5 = mem[SP + 1];
                       SP += 2;
                       PC = (high5 << 8) | low5;
                       return 20;
                   }
                   PC += 1;
                   return 8;
        case 0xC1: // POP BC    b1 c12 flags:----
                   if (debug) printf("POP BC\n");
                   C = mem[SP];
                   B = mem[SP + 1];
                   SP += 2;
                   PC += 1;
                   return 12;
        case 0xC2: // JP NZ <a16>    b3 c16,12 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("JP NZ 0x%04X\n", a16);
                   if (!READF_Z) {
                       PC = a16;
                       return 16;
                   }
                   PC += 3;
                   return 12;
        case 0xC3: // JP <a16>    b3 c16 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("JP 0x%04X\n", a16);
                   PC = a16; 
                   return 16;
        case 0xC4: // CALL NZ, <a16>    b3 c24,12 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("CALL NZ, 0x%04X\n", a16);
                   if (!READF_Z) {
                       SP -= 2;
                       mem_write16(SP, PC + 3);
                       PC = a16;
                       return 24;
                   }
                   PC += 3;
                   return 12;
        case 0xC5: // PUSH BC b1 c16 flags:----
                   if (debug) printf("PUSH BC\n");
                   SP -= 2;
                   mem_write16(SP, BC);
                   PC += 1;
                   return 16;
        case 0xC6: // ADD A, <n8>    b2 c8 flags:Z0HC
                   n8 = mem[PC + 1];
                   if (debug) printf("ADD A, 0x%02X\n", n8);
                   if (A + n8 > 0xFF) SETF_C; else CLRF_C;
                   if ( (A & 0x0F) + (n8 & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
                   A += n8;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N;
                   PC += 2;
                   return 8;
        case 0xC7: // RST $00    b1 c16 flags:----
                   if (debug) printf("RST $00\n");
                   SP -= 2;
                   mem_write16(SP, PC + 1); 
                   PC = 0x0000;
                   return 16;
        case 0xC8: // RET Z b1 c20, 8 flags:----
                   if (debug) printf("RET Z\n");
                   if (READF_Z) {
                       uint8_t low5 = mem[SP];
                       uint8_t high5 = mem[SP + 1];
                       SP += 2;
                       PC = (high5 << 8) | low5;
                       return 20;
                   }
                   PC += 1;
                   return 8;
        case 0xC9: // RET b1 c16 flags:----
                   if (debug) printf("RET \n");
                   {}
                   uint8_t low = mem[SP];
                   uint8_t high = mem[SP + 1];
                   SP += 2;
                   PC= (high << 8) | low;
                   return 16;
        case 0xCA: // JP Z <a16>    b3 c16,12 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("JP Z 0x%04X\n", a16);
                   if (READF_Z) {
                       PC = a16;
                       return 16;
                   }
                   PC += 3;
                   return 12;
        case 0xCB: { // PREFIX b1 c4 flags:----
                       uint8_t cb_op = mem[PC + 1];
                       if (debug) printf("CB %02X: ", cb_op);

                       uint8_t *reg;
                       uint8_t hl_val;
                       int cycles = 8;

                       switch (cb_op & 0x07) {
                           case 0: reg = &B; break;
                           case 1: reg = &C; break;
                           case 2: reg = &D; break;
                           case 3: reg = &E; break;
                           case 4: reg = &H; break;
                           case 5: reg = &L; break;
                           case 6: hl_val = mem[HL]; reg = &hl_val; cycles = 16; break;
                           case 7: reg = &A; break;
                       }

                       uint8_t op_type = cb_op >> 3;

                       if (op_type < 8) {
                           // Rotates and shifts (0x00-0x3F)
                           switch (op_type) {
                               case 0: // RLC
                                   if (debug) printf("RLC\n");
                                   if (*reg & 0x80) SETF_C; else CLRF_C;
                                   *reg = (*reg << 1) | (*reg >> 7);
                                   if (*reg == 0) SETF_Z; else CLRF_Z;
                                   CLRF_N; CLRF_H;
                                   break;
                               case 1: // RRC
                                   if (debug) printf("RRC\n");
                                   if (*reg & 0x01) SETF_C; else CLRF_C;
                                   *reg = (*reg >> 1) | (*reg << 7);
                                   if (*reg == 0) SETF_Z; else CLRF_Z;
                                   CLRF_N; CLRF_H;
                                   break;
                               case 2: // RL
                                   if (debug) printf("RL\n");
                                   {
                                       uint8_t old_carry = (READF_C) ? 1 : 0;
                                       if (*reg & 0x80) SETF_C; else CLRF_C;
                                       *reg = (*reg << 1) | old_carry;
                                   }
                                   if (*reg == 0) SETF_Z; else CLRF_Z;
                                   CLRF_N; CLRF_H;
                                   break;
                               case 3: // RR
                                   if (debug) printf("RR\n");
                                   {
                                       uint8_t old_carry = (READF_C) ? 0x80 : 0;
                                       if (*reg & 0x01) SETF_C; else CLRF_C;
                                       *reg = (*reg >> 1) | old_carry;
                                   }
                                   if (*reg == 0) SETF_Z; else CLRF_Z;
                                   CLRF_N; CLRF_H;
                                   break;
                               case 4: // SLA
                                   if (debug) printf("SLA\n");
                                   if (*reg & 0x80) SETF_C; else CLRF_C;
                                   *reg = *reg << 1;
                                   if (*reg == 0) SETF_Z; else CLRF_Z;
                                   CLRF_N; CLRF_H;
                                   break;
                               case 5: // SRA
                                   if (debug) printf("SRA\n");
                                   if (*reg & 0x01) SETF_C; else CLRF_C;
                                   *reg = (*reg >> 1) | (*reg & 0x80); // preserve bit 7
                                   if (*reg == 0) SETF_Z; else CLRF_Z;
                                   CLRF_N; CLRF_H;
                                   break;
                               case 6: // SWAP
                                   if (debug) printf("SWAP\n");
                                   *reg = ((*reg & 0x0F) << 4) | ((*reg & 0xF0) >> 4);
                                   if (*reg == 0) SETF_Z; else CLRF_Z;
                                   CLRF_N; CLRF_H; CLRF_C;
                                   break;
                               case 7: // SRL
                                   if (debug) printf("SRL\n");
                                   if (*reg & 0x01) SETF_C; else CLRF_C;
                                   *reg = *reg >> 1;
                                   if (*reg == 0) SETF_Z; else CLRF_Z;
                                   CLRF_N; CLRF_H;
                                   break;
                           }
                       } else if (op_type < 16) {
                           // BIT (0x40-0x7F)
                           uint8_t bit = op_type - 8;
                           if (debug) printf("BIT %d\n", bit);
                           if ((*reg & (1 << bit)) == 0) SETF_Z; else CLRF_Z;
                           CLRF_N;
                           SETF_H;
                           if ((cb_op & 0x07) == 6) cycles = 12; // BIT b,[HL] is 12 cycles
                       } else if (op_type < 24) {
                           // RES (0x80-0xBF)
                           uint8_t bit = op_type - 16;
                           if (debug) printf("RES %d\n", bit);
                           *reg &= ~(1 << bit);
                       } else {
                           // SET (0xC0-0xFF)
                           uint8_t bit = op_type - 24;
                           if (debug) printf("SET %d\n", bit);
                           *reg |= (1 << bit);
                       }

                       // Write back to [HL] if needed
                       if ((cb_op & 0x07) == 6 && op_type != 8 && op_type != 9 && op_type != 10 &&
                               op_type != 11 && op_type != 12 && op_type != 13 && op_type != 14 && op_type != 15) {
                           // Not a BIT instruction, write back
                           mem_write8(HL, hl_val);
                       }

                       PC += 2;
                       return cycles;
                   }
        case 0xCC: // CALL Z, <a16>    b3 c24,12 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("CALL Z, 0x%04X\n", a16);
                   if (READF_Z) {
                       SP -= 2;
                       mem_write16(SP, PC + 3);
                       PC = a16;
                       return 24;
                   }
                   PC += 3;
                   return 12;
        case 0xCD: // CALL <a16>    b3 c24 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("CALL 0x%04X\n", a16);
                   SP -= 2;
                   mem_write16(SP, PC + 3);
                   PC = a16;
                   return 24;
        case 0xCE: { // ADC A, <n8>    b2 c8 flags:Z0HC
                       n8 = mem[PC + 1];
                       if (debug) printf("ADC A, 0x%02X\n", n8);
                       uint8_t carry = (READF_C) ? 1 : 0;
                       uint16_t result = A + n8 + carry;
                       if (((A & 0x0F) + (n8 & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
                       if (result > 0xFF) SETF_C; else CLRF_C;
                       A = (uint8_t)result;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       CLRF_N;
                       PC += 2;
                       return 8;
                   }
        case 0xCF: // RST $08    b1 c16 flags:----
                   if (debug) printf("RST $08\n");
                   SP -= 2;
                   mem_write16(SP, PC + 1); 
                   PC = 0x0008;
                   return 16;
        case 0xD0: // RET NC    b1 c20, 8 flags:----
                   if (debug) printf("RET NC\n");
                   if (!READF_C) {
                       uint8_t low4 = mem[SP];
                       uint8_t high4 = mem[SP + 1];
                       SP += 2;
                       PC = (high4 << 8) | low4;
                       return 20;
                   }
                   PC += 1;
                   return 8;
        case 0xD1: // POP DE    b1 c12 flags:----
                   if (debug) printf("POP DE\n");
                   E = mem[SP];
                   D = mem[SP + 1];
                   SP += 2;
                   PC += 1;
                   return 12;
        case 0xD2: // JP NC <a16>    b3 c16,12 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("JP NC 0x%04X\n", a16);
                   if (!READF_C) {
                       PC = a16;
                       return 16;
                   }
                   PC += 3;
                   return 12;
        case 0xD4: // CALL NC, <a16>    b3 c24,12 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("CALL NC, 0x%04X\n", a16);
                   if (!READF_C) {
                       SP -= 2;
                       mem_write16(SP, PC + 3);
                       PC = a16;
                       return 24;
                   }
                   PC += 3;
                   return 12;
        case 0xD5: // PUSH DE    b1 c16 flags:----
                   if (debug) printf("PUSH DE\n");
                   SP -= 2;
                   mem_write16(SP, DE);
                   PC += 1;
                   return 16;
        case 0xD6: // SUB A, <n8>    b2 c8 flags:Z1HC
                   n8 = mem[PC + 1];
                   if (debug) printf("SUB A, 0x%02X\n", n8);
                   if (n8 > A) SETF_C; else CLRF_C;
                   if ((n8 & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   SETF_N;
                   A -= n8;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   PC += 2;
                   return 8;
        case 0xD7: // RST $10    b1 c16 flags:----
                   if (debug) printf("RST $10\n");
                   SP -= 2;
                   mem_write16(SP, PC + 1); 
                   PC = 0x0010;
                   return 16;
        case 0xD8: // RET C    b1 c20, 8 flags:----
                   if (debug) printf("RET C\n");
                   if (READF_C) {
                       uint8_t low5 = mem[SP];
                       uint8_t high5 = mem[SP + 1];
                       SP += 2;
                       PC = (high5 << 8) | low5;
                       return 20;
                   }
                   PC += 1;
                   return 8;
        case 0xD9: // RETI    b1 c16 flags:----
                   if (debug) printf("RETI \n");
                   {
                       uint8_t low = mem[SP];
                       uint8_t high = mem[SP + 1];
                       ime_scheduled = true;
                       SP += 2;
                       PC = (high << 8) | low;
                   }
                   return 16;
        case 0xDA: // JP C <a16>    b3 c16,12 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("JP C 0x%04X\n", a16);
                   if (READF_C) {
                       PC = a16;
                       return 16;
                   }
                   PC += 3;
                   return 12;
        case 0xDC: // CALL C, <a16>    b3 c24,12 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8); 
                   if (debug) printf("CALL C, 0x%04X\n", a16);
                   if (READF_C) {
                       SP -= 2;
                       mem_write16(SP, PC + 3);
                       PC = a16;
                       return 24;
                   }
                   PC += 3;
                   return 12;
        case 0xDE: // SBC A, <n8>    b2 c8 flags:Z1HC
                   n8 = mem[PC + 1];
                   {
                       if (debug) printf("SBC A, 0x%02X\n", n8);
                       uint8_t carry = READF_C ? 1 : 0;
                       if (n8 + carry > A) SETF_C; else CLRF_C;
                       if ((n8 & 0x0F) + carry > (A & 0x0F)) SETF_H; else CLRF_H;
                       A = A - n8 - carry;
                       if (A == 0) SETF_Z; else CLRF_Z;
                       SETF_N;
                       PC += 2;
                   }
                   return 8;
        case 0xDF: // RST $18    b1 c16 flags:----
                   if (debug) printf("RST $18\n");
                   SP -= 2;
                   mem_write16(SP, PC + 1); 
                   PC = 0x0018;
                   return 16;
        case 0xE0: // LDH <a8>, A    b2 c12 flags:----
                   a8 = mem[PC + 1];
                   uint16_t addr = 0xFF00 + a8;
                   if (debug) printf("LDH 0x%04X, A\n", addr);
                   mem_write8(addr, A);
                   PC += 2;
                   return 12;
        case 0xE1: // POP HL    b1 c12 flags:----
                   {}
                   if (debug) printf("POP HL\n");
                   uint8_t low1 = mem[SP];
                   uint8_t high1 = mem[SP + 1];
                   SP += 2;
                   L = low1;
                   H = high1;
                   PC += 1;
                   return 12;
        case 0xE2: // LDH [C], A  b1 c8 flags:----
                   if (debug) printf("LDH [C], A\n");
                   mem_write8(0xFF00 + C, A);
                   PC += 1;
                   return 8;
        case 0xE5: // PUSH HL    b1 c16 flags:----
                   if (debug) printf("PUSH HL\n");
                   SP -= 2;
                   mem_write16(SP, HL);
                   PC += 1;
                   return 16;
        case 0xE6: // AND A, <n8>    b2 c8 flags:Z010
                   n8 = mem[PC + 1];
                   if (debug) printf("AND A, 0x%02X\n", n8);
                   A = A & n8;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; SETF_H; CLRF_C;
                   PC += 2;
                   return 8;
        case 0xE7: // RST $20    b1 c16 flags:----
                   if (debug) printf("RST $20\n");
                   SP -= 2;
                   mem_write16(SP, PC + 1); 
                   PC = 0x0020;
                   return 16;
        case 0xE8: // ADD SP, <e8>    b2 c16 flags=00HC
                   e8 = (int8_t)mem[PC + 1];
                   if (debug) printf("ADD SP, 0x%02X\n", e8);
                   {
                       uint16_t result = SP + e8;
                       uint8_t unsigned_e8 = (uint8_t)mem[PC + 1];  // Use unsigned for flag calc

                       if (((SP & 0x0F) + (unsigned_e8 & 0x0F)) > 0x0F) SETF_H; else CLRF_H;
                       if (((SP & 0xFF) + unsigned_e8) > 0xFF) SETF_C; else CLRF_C;

                       CLRF_Z;
                       CLRF_N;

                       SP = result;
                   }
                   PC += 2;
                   return 16;
        case 0xE9: // JP HL    b1 c4 flags:----
                   if (debug) printf("JP HL\n");
                   PC = HL;
                   return 4;
        case 0xEA: // LD [a16], A    b3 c16 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8);
                   if (debug) printf("LD 0x%04X, A\n", a16);
                   mem_write8(a16, A);
                   PC += 3;
                   return 16;
        case 0xEE: // XOR A, <n8>    b2 c8 flags:Z000
                   n8 = mem[PC + 1];
                   if (debug) printf("XOR A, 0x%02X\n", n8);
                   A = A ^ n8;
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 2;
                   return 8;
        case 0xEF: // RST $28    b1 c16 flags:----
                   if (debug) printf("RST $28\n");
                   SP -= 2;
                   mem_write16(SP, PC + 1); 
                   PC = 0x0028;
                   return 16;

        case 0xF0: // LDH A, <a8>
                   a8 = mem[PC + 1];

                   //if (a8 == 0x00) {
                   //Joypad register handling
                   //} else {
                   A = mem[0xFF00 + a8];
                   //}

                   if (debug) printf("LDH A, 0x%02X\n", a8);

                   PC += 2;
                   return 12;
        case 0xF1: // POP AF    b1 c12 flags:ZNHC
                   if (debug) printf("POP AF\n");
                   {
                       uint8_t low2 = mem[SP];
                       uint8_t high2 = mem[SP + 1];
                       SP += 2;
                       F = low2 & 0xF0;
                       A = high2;
                   }
                   PC += 1;
                   return 12;
        case 0xF2: // LDH A, [0xFF00 + C]    b1 c8 flags:----
                   if (debug) printf("LDH A, [0xFF00 + C]\n");
                   A = mem[0xFF00 + C];
                   PC += 1;
                   return 8;
        case 0xF3: // DI    b1 c4 flags:----
                   if (debug) printf("DI\n");
                   IME = false;
                   ime_scheduled = false;  // Cancel any pending EI
                   PC += 1;
                   return 4;
        case 0xF5: // PUSH AF    b1 c16 flags:----
                   if (debug) printf("PUSH AF\n");
                   SP -= 2;
                   mem_write16(SP, AF);
                   PC += 1;
                   return 16;
        case 0xF6: // OR A, <n8>    b2 c8 flags:Z000
                   n8 = mem[PC + 1];
                   if (debug) printf("OR A, 0x%02X\n", n8);
                   A = (A | n8);
                   if (A == 0) SETF_Z; else CLRF_Z;
                   CLRF_N; CLRF_H; CLRF_C;
                   PC += 2;
                   return 8;
        case 0xF7: // RST $30    b1 c16 flags:----
                   if (debug) printf("RST $30\n");
                   SP -= 2;
                   mem_write16(SP, PC + 1); 
                   PC = 0x0030;
                   return 16;
        case 0xF8: // LD HL, SP+e8    b2 c12 flags=00HC
                   e8 = (int8_t)mem[PC + 1];
                   if (debug) printf("LD HL, SP+0x%02X\n", e8);
                   {
                       uint16_t result = SP + e8;
                       uint8_t unsigned_e8 = (uint8_t)mem[PC + 1];  // Use unsigned for flag calc

                       if (((SP & 0x0F) + (unsigned_e8 & 0x0F)) > 0x0F) SETF_H; else CLRF_H;
                       if (((SP & 0xFF) + unsigned_e8) > 0xFF) SETF_C; else CLRF_C;

                       CLRF_Z;
                       CLRF_N;

                       H = (result >> 8) & 0xFF;
                       L = result & 0xFF;
                   }
                   PC += 2;
                   return 12;
        case 0xF9: // LD SP, HL    b1 c8 flags:----
                   if (debug) printf("LD SP, HL\n");
                   SP = HL;
                   PC += 1;
                   return 8;
        case 0xFA:  //LD A, [a16]    b3 c16 flags:----
                   a16 = mem[PC + 1] | (mem[PC + 2] << 8);
                   if (debug) printf("LD A, [0x%04X]\n", a16);
                   A = mem[a16];
                   PC += 3;
                   return 16;
        case 0xFB: // EI    b1 c4 flags:----
                   if (debug) printf("EI\n");
                   ime_scheduled = true;
                   PC += 1;
                   return 4;
        case 0xFE: // CP A, <n8>    b2 c8 flags:Z1HC
                   n8 = mem[PC + 1];
                   if (n8 == 0x94) {
                   }
                   if (debug) printf("CP A, 0x%02X\n", n8);
                   if (n8 == A) SETF_Z; else CLRF_Z;
                   SETF_N;
                   if ((n8 & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
                   if (n8 > A) SETF_C; else CLRF_C;
                   PC += 2;
                   return 8;
        case 0xFF: // RST $38    b1 c16 flags:----
                   if (debug) printf("RST $38\n");
                   SP -= 2;
                   mem_write16(SP, PC + 1); 
                   PC = 0x0038;
                   return 16;
        default:
                   printf("<UNKNOWN 0x%02X at PC=0x%04X>\n", op, PC);
                   exit(1);

    }
}

void inc_reg16(uint8_t *low, uint8_t *high) {
    uint16_t reg = (*low) | ((*high) << 8);
    reg++;
    *low = (uint8_t)reg;
    *high = reg >> 8;
}
void dec_reg16(uint8_t *low, uint8_t *high) {
    uint16_t reg = (*low) | ((*high) << 8);
    reg--;
    *low = (uint8_t)reg;
    *high = reg >> 8;
}
uint8_t val_char = 0;
void print_bin8(uint8_t v) {
    for (int i = 7; i >= 0; --i)
        putchar( (v & (1u << i)) ? '1' : '0' );
}
void mem_write8(uint16_t addr, uint8_t b) {
    if (addr == 0xFF46) {
        uint16_t src = b << 8;
        for (int i = 0; i < 160; i++) {
            mem[0xFE00 + i] = mem[src + i];
        }
    }
    if (addr == 0xFF00) {
        uint8_t select = b & 0x30;   // 0b0011_0000
        *JOYP = (*JOYP & ~0x30) | select;
        if ((*JOYP & 0b00110000) == 0b00110000) {
            *JOYP |= 0b00001111;
        } else if ((*JOYP & 0b00010000) == 0){ //d-pad
            if (right) *JOYP &= 0b11111110; else *JOYP |= 0b00000001; 
            if (left) *JOYP &= 0b11111101; else *JOYP |= 0b00000010; 
            if (up) *JOYP &= 0b11111011; else *JOYP |= 0b00000100; 
            if (down) *JOYP &= 0b11110111; else *JOYP |= 0b00001000; 
        } else if ((*JOYP & 0b00100000) == 0) { 
            if (a) *JOYP &= 0b11111110; else *JOYP |= 0b00000001; 
            if (b) *JOYP &= 0b11111101; else *JOYP |= 0b00000010; 
            if (sel) *JOYP &= 0b11111011; else *JOYP |= 0b00000100; 
            if (start) *JOYP &= 0b11110111; else *JOYP |= 0b00001000; 
        }
        return;
    }

    // Channel 2 square wave
    if (addr == 0xFF19) { // channel 2 period high & control register (NR24) 
                          //chan2debug(b);
        if ((*NR52 & 0b10000000) == 0) {
            channel2_playing = false;
        } else if ((*NR22 & 0xF8) == 0) { //DAC is turned off
            channel2_playing = false;
        } else if ((b & 0b10000000) != 0) {
            uint8_t initial_volume = (*NR22 & 0b11110000) >> 4;
            channel2_volume = initial_volume;
            channel2_playing = true;
            channel2_phase = 0;
            uint16_t period = *NR23; 

            uint16_t ph = b & 0b00000111;
            ph = ph << 8;
            period |= ph;
            double frequency = (double)131072 / (2048 - period);
            channel2_phase_increment = frequency / 44100.0f;
        }
    }

    if (addr == 0xFF14) { // channel 1 period high & control register (NR24) 
        if ((*NR52 & 0b10000000) == 0) {
            channel1_playing = false;
        } else if ((*NR12 & 0xF8) == 0) { //DAC is turned off
            channel1_playing = false;
        } else if ((b & 0b10000000) != 0) {
            uint8_t initial_volume = (*NR12 & 0b11110000) >> 4;
            channel1_volume = initial_volume;
            channel1_playing = true;
            channel1_phase = 0;
            uint16_t period = *NR13; 

            uint16_t ph = b & 0b00000111;
            ph = ph << 8;
            period |= ph;
            double frequency = (double)131072 / (2048 - period);
            channel1_phase_increment = frequency / 44100.0f;
        }
    }

    if (debug) printf("Writing to ");
    if (addr < 0x4000) {
        if (debug)   printf("ROM bank 0 (ignored)\n");
    } else if (addr < 0x8000) {
        if (debug) printf("ROM bank 1 (ignored)\n");
    } else if (addr < 0xA000) {
        if (debug) printf("8 KiB Video RAM (VRAM)\n");
        mem[addr] = b;
    } else if (addr < 0xC000) {
        if (debug) printf("8 KiB External RAM\n");
        mem[addr] = b;
    } else if (addr < 0xE000) {
        if (debug) if (debug) printf("4 KiB Work RAM (WRAM)\n");
        mem[addr] = b;
    } else if (addr < 0xFE00) {
        if (debug) printf("Echo RAM -> WRAM mirror\n");
        mem[addr - 0x2000] = b;  // Mirror to WRAM (0xE000 -> 0xC000)
    } else if (addr < 0xFEA0) {
        if (debug) printf("Object attribute memory (OAM)\n");
        mem[addr] = b;
    } else if (addr < 0xFF00) {
        if (debug) printf("Not Usable (ignored)\n");
    } else if (addr < 0xFF80) {
        if( addr == 0xFF01) {
            val_char = b;

        } else if(addr == 0xFF02 && b == 0x81) {
            putchar(val_char);
            fflush(stdout);
        }
        if (debug) printf("I/O Registers\n");
        mem[addr] = b;
    } else if (addr < 0xFFFF) {
        if (debug) printf("High RAM (HRAM)\n");
        mem[addr] = b;
    } else {
        if (debug) printf("Interrupt Enable register (IE)\n");
        mem[addr] = b;
    }
}
void mem_write16(uint16_t addr, uint16_t b) {
    if (debug) printf("Writing to ");
    if (addr < 0x4000) {
        if (debug) printf("ROM bank 0 (ignored)\n");
    } else if (addr < 0x8000) {
        if (debug) printf("ROM bank 1 (ignored)\n");
    } else if (addr < 0xA000) {
        if (debug)  printf("8 KiB Video RAM (VRAM)\n");

        mem[addr] = b & 0xFF;
        mem[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xC000) {
        if (debug) printf("8 KiB External RAM\n");
        mem[addr] = b & 0xFF;
        mem[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xE000) {
        if (debug) printf("4 KiB Work RAM (WRAM)\n");
        mem[addr] = b & 0xFF;
        mem[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xFE00) {
        if (debug) printf("Echo RAM (ignored)\n");
        mem[addr - 0x2000] = b & 0xFF;  // Mirror to WRAM (0xE000 -> 0xC000)
        mem[addr - 0x2000 + 1] = (b >> 8) & 0xFF;  // Mirror to WRAM (0xE000 -> 0xC000)
    } else if (addr < 0xFEA0) {
        if (debug) printf("Object attribute memory (OAM)\n");
        mem[addr] = b & 0xFF;
        mem[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xFF00) {
        if (debug)  printf("Not Usable (ignored)\n");
    } else if (addr < 0xFF80) {
        if (debug) printf("I/O Registers\n");
        mem[addr] = b & 0xFF;
        mem[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xFFFF) {
        if (debug) printf("High RAM (HRAM)\n");
        mem[addr] = b & 0xFF;
        mem[addr + 1] = (b >> 8) & 0xFF;
    } else {
        if (debug)  printf("Interrupt Enable register (IE)\n");
        mem[addr] = b & 0xFF;
        mem[addr + 1] = (b >> 8) & 0xFF;
    }
}
void show_registers() {
    printf("\tA=0x%02X\n", A); 
    printf("\tBC=0x%04X\n", BC); 
    printf("\tDE=0x%04X\n", DE); 
    printf("\tHL=0x%04X\n", HL); 
    printf("\tSP=0x%04X\n", SP); 
    printf("\tPC=0x%04X\n", PC); 
    printf("\t");
    if ((F & 0b10000000) != 0) printf("Z:1 "); else printf("Z:0 ");
    if ((F & 0b01000000) != 0) printf("N:1 "); else printf("N:0 ");
    if ((F & 0b00100000) != 0) printf("H:1 "); else printf("H:0 ");
    if ((F & 0b00010000) != 0) printf("C:1 "); else printf("C:0 ");
    printf("\n\n");
}

// cartridge header 0x0100 - 0x014F
// 0x0100 - 0x0103: Entry point
// 0x0100 - 0x0133: Nintendo logo
// 0x0134 - 0x0143: Title
//  0x013F - 0x0142: Manufacturer code
//  0x0143: CGB flag
//  0x0144 - 0x0145: new license code
//  
uint8_t logo[] = {
    0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D,
    0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99,
    0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E
};
const char *old_licensee_codes[256] = {
    [0x00] = "None",
    [0x01] = "Nintendo",
    [0x08] = "Capcom",
    [0x09] = "HOT-B",
    [0x0A] = "Jaleco",
    [0x0B] = "Coconuts Japan",
    [0x0C] = "Elite Systems",
    [0x13] = "EA (Electronic Arts)",
    [0x18] = "Hudson Soft",
    [0x19] = "ITC Entertainment",
    [0x1A] = "Yanoman",
    [0x1D] = "Japan Clary",
    [0x1F] = "Virgin Games Ltd.",
    [0x24] = "PCM Complete",
    [0x25] = "San-X",
    [0x28] = "Kemco",
    [0x29] = "SETA Corporation",
    [0x30] = "Infogrames",
    [0x31] = "Nintendo",
    [0x32] = "Bandai",
    [0x33] = NULL, // Use new licensee code
    [0x34] = "Konami",
    [0x35] = "HectorSoft",
    [0x38] = "Capcom",
    [0x39] = "Banpresto",
    [0x3C] = "Entertainment Interactive",
    [0x3E] = "Gremlin",
    [0x41] = "Ubi Soft",
    [0x42] = "Atlus",
    [0x44] = "Malibu Interactive",
    [0x46] = "Angel",
    [0x47] = "Spectrum HoloByte",
    [0x49] = "Irem",
    [0x4A] = "Virgin Games Ltd.",
    [0x4D] = "Malibu Interactive",
    [0x4F] = "U.S. Gold",
    [0x50] = "Absolute",
    [0x51] = "Acclaim Entertainment",
    [0x52] = "Activision",
    [0x53] = "Sammy USA Corporation",
    [0x54] = "GameTek",
    [0x55] = "Park Place",
    [0x56] = "LJN",
    [0x57] = "Matchbox",
    [0x59] = "Milton Bradley Company",
    [0x5A] = "Mindscape",
    [0x5B] = "Romstar",
    [0x5C] = "Naxat Soft",
    [0x5D] = "Tradewest",
    [0x60] = "Titus Interactive",
    [0x61] = "Virgin Games Ltd.",
    [0x67] = "Ocean Software",
    [0x69] = "EA (Electronic Arts)",
    [0x6E] = "Elite Systems",
    [0x6F] = "Electro Brain",
    [0x70] = "Infogrames",
    [0x71] = "Interplay Entertainment",
    [0x72] = "Broderbund",
    [0x73] = "Sculptured Software",
    [0x75] = "The Sales Curve Limited",
    [0x78] = "THQ",
    [0x79] = "Accolade",
    [0x7A] = "Triffix Entertainment",
    [0x7C] = "MicroProse",
    [0x7F] = "Kemco",
    [0x80] = "Misawa Entertainment",
    [0x83] = "LOZC G.",
    [0x86] = "Tokuma Shoten",
    [0x8B] = "Bullet-Proof Software",
    [0x8C] = "Vic Tokai Corp.",
    [0x8E] = "Ape Inc.",
    [0x8F] = "I'Max",
    [0x91] = "Chunsoft Co.",
    [0x92] = "Video System",
    [0x93] = "Tsubaraya Productions",
    [0x95] = "Varie",
    [0x96] = "Yonezawa/S'Pal",
    [0x97] = "Kemco",
    [0x99] = "Arc",
    [0x9A] = "Nihon Bussan",
    [0x9B] = "Tecmo",
    [0x9C] = "Imagineer",
    [0x9D] = "Banpresto",
    [0x9F] = "Nova",
    [0xA1] = "Hori Electric",
    [0xA2] = "Bandai",
    [0xA4] = "Konami",
    [0xA6] = "Kawada",
    [0xA7] = "Takara",
    [0xA9] = "Technos Japan",
    [0xAA] = "Broderbund",
    [0xAC] = "Toei Animation",
    [0xAD] = "Toho",
    [0xAF] = "Namco",
    [0xB0] = "Acclaim Entertainment",
    [0xB1] = "ASCII Corporation or Nexsoft",
    [0xB2] = "Bandai",
    [0xB4] = "Square Enix",
    [0xB6] = "HAL Laboratory",
    [0xB7] = "SNK",
    [0xB9] = "Pony Canyon",
    [0xBA] = "Culture Brain",
    [0xBB] = "Sunsoft",
    [0xBD] = "Sony Imagesoft",
    [0xBF] = "Sammy Corporation",
    [0xC0] = "Taito",
    [0xC2] = "Kemco",
    [0xC3] = "Square",
    [0xC4] = "Tokuma Shoten",
    [0xC5] = "Data East",
    [0xC6] = "Tonkin House",
    [0xC8] = "Koei",
    [0xC9] = "UFL",
    [0xCA] = "Ultra Games",
    [0xCB] = "VAP, Inc.",
    [0xCC] = "Use Corporation",
    [0xCD] = "Meldac",
    [0xCE] = "Pony Canyon",
    [0xCF] = "Angel",
    [0xD0] = "Taito",
    [0xD1] = "SOFEL (Software Engineering Lab)",
    [0xD2] = "Quest",
    [0xD3] = "Sigma Enterprises",
    [0xD4] = "ASK Kodansha Co.",
    [0xD6] = "Naxat Soft",
    [0xD7] = "Copya System",
    [0xD9] = "Banpresto",
    [0xDA] = "Tomy",
    [0xDB] = "LJN",
    [0xDD] = "Nippon Computer Systems",
    [0xDE] = "Human Ent.",
    [0xDF] = "Altron",
    [0xE0] = "Jaleco",
    [0xE1] = "Towa Chiki",
    [0xE2] = "Yutaka",
    [0xE3] = "Varie",
    [0xE5] = "Epoch",
    [0xE7] = "Athena",
    [0xE8] = "Asmik Ace Entertainment",
    [0xE9] = "Natsume",
    [0xEA] = "King Records",
    [0xEB] = "Atlus",
    [0xEC] = "Epic/Sony Records",
    [0xEE] = "IGS",
    [0xF0] = "A Wave",
    [0xF3] = "Extreme Entertainment",
    [0xFF] = "LJN",
};
const char *new_licensee_codes[256] = {
    [0x00] = "None",
    [0x01] = "Nintendo Research & Development 1",
    [0x08] = "Capcom",
    [0x13] = "EA (Electronic Arts)",
    [0x18] = "Hudson Soft",
    [0x19] = "B-AI",
    [0x20] = "KSS",
    [0x22] = "Planning Office WADA",
    [0x24] = "PCM Complete",
    [0x25] = "San-X",
    [0x28] = "Kemco",
    [0x29] = "SETA Corporation",
    [0x30] = "Viacom",
    [0x31] = "Nintendo",
    [0x32] = "Bandai",
    [0x33] = "Ocean Software/Acclaim Entertainment",
    [0x34] = "Konami",
    [0x35] = "HectorSoft",
    [0x37] = "Taito",
    [0x38] = "Hudson Soft",
    [0x39] = "Banpresto",
    [0x41] = "Ubi Soft",
    [0x42] = "Atlus",
    [0x44] = "Malibu Interactive",
    [0x46] = "Angel",
    [0x47] = "Bullet-Proof Software",
    [0x49] = "Irem",
    [0x50] = "Absolute",
    [0x51] = "Acclaim Entertainment",
    [0x52] = "Activision",
    [0x53] = "Sammy USA Corporation",
    [0x54] = "Konami",
    [0x55] = "Hi Tech Expressions",
    [0x56] = "LJN",
    [0x57] = "Matchbox",
    [0x58] = "Mattel",
    [0x59] = "Milton Bradley Company",
    [0x60] = "Titus Interactive",
    [0x61] = "Virgin Games Ltd.",
    [0x64] = "Lucasfilm Games",
    [0x67] = "Ocean Software",
    [0x69] = "EA (Electronic Arts)",
    [0x70] = "Infogrames",
    [0x71] = "Interplay Entertainment",
    [0x72] = "Broderbund",
    [0x73] = "Sculptured Software",
    [0x75] = "The Sales Curve Limited",
    [0x78] = "THQ",
    [0x79] = "Accolade",
    [0x80] = "Misawa Entertainment",
    [0x83] = "LOZC G.",
    [0x86] = "Tokuma Shoten",
    [0x87] = "Tsukuda Original",
    [0x91] = "Chunsoft Co.",
    [0x92] = "Video System",
    [0x93] = "Ocean Software/Acclaim Entertainment",
    [0x95] = "Varie",
    [0x96] = "Yonezawa/S'Pal",
    [0x97] = "Kaneko",
    [0x99] = "Pack-In-Video",
};
char *cart_type[] = {
    [0x00] = "ROM ONLY",
    [0x01] = "MBC1",
    [0x02] = "MBC1+RAM",
    [0x03] = "MBC1+RAM+BATTERY",
    [0x05] = "MBC2",
    [0x06] = "MBC2+BATTERY",
    [0x08] = "ROM+RAM",
    [0x09] = "ROM+RAM+BATTERY",
    [0x0B] = "MMM01",
    [0x0C] = "MMM01+RAM",
    [0x0D] = "MMM01+RAM+BATTERY",
    [0x0F] = "MBC3+TIMER+BATTERY",
    [0x10] = "MBC3+TIMER+RAM+BATTERY",
    [0x11] = "MBC3",
    [0x12] = "MBC3+RAM",
    [0x13] = "MBC3+RAM+BATTERY",
    [0x19] = "MBC5",
    [0x1A] = "MBC5+RAM",
    [0x1B] = "MBC5+RAM+BATTERY",
    [0x1C] = "MBC5+RUMBLE",
    [0x1D] = "MBC5+RUMBLE+RAM",
    [0x1E] = "MBC5+RUMBLE+RAM+BATTERY",
    [0x20] = "MBC6",
    [0x22] = "MBC7+SENSOR+RUMBLE+RAM+BATTERY",
    [0xFC] = "POCKET CAMERA",
    [0xFD] = "BANDAI TAMA5",
    [0xFE] = "HuC3",
    [0xFF] = "HuC1+RAM+BATTERY"
};
void show_cartridge_info() {

    uint8_t cgb_flag = mem[0x0143];
    char *cart_comp;

    if (cgb_flag == 0x80) {
        cart_comp = "CGB-compatible";
    } else if (cgb_flag == 0xC0) {
        cart_comp = "CGB-only";
    } else {
        cart_comp = "DMG";
    }
    bool is_old_licensee = mem[0x014B] != 0x33;
    bool logo_found = memcmp(logo, mem + 0x104, 48) == 0;
    printf("Genuine Nintendo logo found: %s\n", logo_found ? "YES" : "NO");
    printf("Title: %s\n", mem + 0x134);
    printf("Cartridge compatibility: %s\n", cart_comp);
    printf("Cartridge type: %s\n", cart_type[mem[0x0147]]);
    printf("SGB support: %s\n", mem[0x0146] == 0x03 ? "YES" : "NO");
    if (is_old_licensee) {
        printf("Publisher: %s\n", old_licensee_codes[mem[0x014b]]);
    }
    uint8_t rom_size = mem[0x0148];
    if (rom_size == 0x00) {
        printf("ROM size: 32 KiB\n");
        printf("ROM banks: 2 (no banking)\n");
    } else if (rom_size == 0x01) {
        printf("ROM size: 64 KiB\n");
        printf("ROM banks: 4\n");
    } else if (rom_size == 0x02) {
        printf("ROM size: 128 KiB\n");
        printf("ROM banks: 8\n");
    } else if (rom_size == 0x03) {
        printf("ROM size: 256 KiB\n");
        printf("ROM banks: 16\n");
    } else if (rom_size == 0x04) {
        printf("ROM size: 512 KiB\n");
        printf("ROM banks: 32\n");
    } else if (rom_size == 0x05) {
        printf("ROM size: 1 MiB\n");
        printf("ROM banks: 64\n");
    } else if (rom_size == 0x06) {
        printf("ROM size: 2 MiB\n");
        printf("ROM banks: 128\n");
    } else if (rom_size == 0x07) {
        printf("ROM size: 4 MiB\n");
        printf("ROM banks: 256\n");
    } else if (rom_size == 0x08) {
        printf("ROM size: 8 MiB\n");
        printf("ROM banks: 512\n");
    } else if (rom_size == 0x52) {
        printf("ROM size: 1.1 MiB\n");
        printf("ROM banks: 72\n");
    } else if (rom_size == 0x53) {
        printf("ROM size: 1.2 MiB\n");
        printf("ROM banks: 80\n");
    } else if (rom_size == 0x54) {
        printf("ROM size: 1.5 MiB\n");
        printf("ROM banks: 96\n");
    }
    uint8_t ram_size = mem[0x0149];
    if (ram_size == 0x00) {
        printf("cartridge RAM: none\n");
    } else if (ram_size == 0x02) {
        printf("cartridge RAM: 8 KiB\n");
    } else if (ram_size == 0x03) {
        printf("cartridge RAM: 32 KiB\n");
    } else if (ram_size == 0x04) {
        printf("cartridge RAM: 128 KiB\n");
    } else if (ram_size == 0x05) {
        printf("cartridge RAM: 64 KiB\n");
    }
    printf("\n");

}
// Global variables
int timer_counter = 0;
int div_counter = 0;

void timer_step(int cycles) {
    // DIV register increments every 256 cycles
    div_counter += cycles;
    while (div_counter >= 256) {
        div_counter -= 256;
        mem[0xFF04]++;  // DIV register

        //
        if (mem[0xFF04] % 512 == 0) { // every 64 Hz
                                      //envelope sweep  channel 1
            uint8_t ch1pace = *NR12 & 0b00000111;
            static int pace_counter1 = 0;
            if (ch1pace != 0) { //channel 1 envelope active
                if (pace_counter1 == ch1pace) {
                    if ((*NR12 & 0b00001000) != 0 ) { //increasing
                        channel1_volume++;
                    } else {
                        channel1_volume--;
                    }
                    pace_counter1 = 0;
                } else {
                    pace_counter1 ++;
                }

            }
            //envelope sweep  channel 2
            uint8_t ch2pace = *NR22 & 0b00000111;
            static int pace_counter2 = 0;
            if (ch2pace != 0) { //channel 1 envelope active
                if (pace_counter2 == ch2pace) {
                    if ((*NR22 & 0b00001000) != 0 ) { //increasing
                        channel2_volume++;
                    } else {
                        channel2_volume--;
                    }
                    pace_counter2 = 0;
                } else {
                    pace_counter2 ++;
                }

            }
        } 
        if (mem[0xFF04] % 64 == 0) { // every 256 Hz
                                     // sound length
        } 
        if (mem[0xFF04] % 128 == 0) { // every 128 Hz
                                      // Ch1 freq sweep
        }
    }

    uint8_t TAC = mem[0xFF07];
    if (!(TAC & 0x04)) return;  // Timer disabled

    // Frequency selection
    int threshold;
    switch (TAC & 0x03) {
        case 0: threshold = 1024; break;  // 4096 Hz
        case 1: threshold = 16; break;    // 262144 Hz
        case 2: threshold = 64; break;    // 65536 Hz
        case 3: threshold = 256; break;   // 16384 Hz
    }

    timer_counter += cycles;
    while (timer_counter >= threshold) {
        timer_counter -= threshold;
        mem[0xFF05]++;  // TIMA register

        if (mem[0xFF05] == 0) {  // Overflow
            mem[0xFF05] = mem[0xFF06];  // Reset to TMA
            mem[0xFF0F] |= 0x04;  // Request timer interrupt
        }
    }
}

void handle_interrupts(void) {
    if ((*IE & *IF & 0b00000001) == 0b00000001) { // VBlank
        IME = false;
        *IF = (*IF & 0b11111110);
        SP -= 2;
        mem_write16(SP, PC);
        PC = 0x40;
        if (debug) printf("<VBlank Interrupt>\n");
    } else if ((*IE & *IF & 0b00000010) == 0b00000010) { // LCD
        IME = false;
        *IF = (*IF & 0b11111101);
        SP -= 2;
        mem_write16(SP, PC);
        PC = 0x48;
        if (debug) printf("<STAT Interrupt>\n");
    } else if ((*IE & *IF & 0b00000100) == 0b00000100) { // Timer
        IME = false;
        *IF =  (*IF & 0b11111011);
        SP -= 2;
        mem_write16(SP, PC);
        PC = 0x50;
        if (debug) printf("<Timer Interrupt>\n");
    } else if ((*IE & *IF & 0b00001000) == 0b00001000) { // serial
        IME = false;
        *IF = (*IF & 0b11110111);
        SP -= 2;
        mem_write16(SP, PC);
        PC = 0x58;
        if (debug) printf("<Serial Interrupt>\n");
    } else if ((*IE & *IF & 0b00010000) == 0b00010000) { // Joypad
        IME = false;
        *IF = (*IF & 0b11101111);
        SP -= 2;
        mem_write16(SP, PC);
        PC = 0x60;
        if (debug) printf("<Joypad Interrupt>\n");
    }
}




void apu_step(int cycles) {
    cycles_per_sample_counter += cycles;
    if (cycles_per_sample_counter >= CYCLES_PER_SAMPLE) {
        cycles_per_sample_counter -= CYCLES_PER_SAMPLE;
        int16_t vol_steps = 1000;
        int16_t sample = 0;
        if (channel1_playing) {
            sample += (channel1_phase < 0.5f) ? channel1_volume * vol_steps : channel1_volume * (-vol_steps);
            channel1_phase += channel1_phase_increment;
            if (channel1_phase >= 1.0f) channel1_phase -= 1.0f;
        }

        if (channel2_playing) {
            sample += (channel2_phase < 0.5f) ? channel2_volume * vol_steps : channel2_volume * (-vol_steps);
            channel2_phase += channel2_phase_increment;
            if (channel2_phase >= 1.0f) channel2_phase -= 1.0f;
        }

        SDL_QueueAudio(audio_device, &sample, sizeof(sample));
    }
}


void chan2debug(uint8_t nr24) {

    return;
    puts("---Channel 2 debug---");
    if ((*NR52 & 0b10000000) == 0) {
        puts("APU is OFF");
    } else {
        puts("APU is ON");
    }

    if ((*NR22 & 0xF8) == 0) {
        puts("Channel 2 DAC is OFF");
    } else {
        puts("Channel 2 DAC is ON");
    }

    if (nr24 &0b1000000) {
        puts("Channel 2 ON");
    }

    //NR21
    uint8_t initial_length = *NR21 & 0b00111111;
    uint8_t wave_duty = (*NR21 & 0b11000000) >> 6;
    printf("NR21: wave_duty=%d, initial_length=%d\n",wave_duty, initial_length); 

    //NR22
    uint8_t initial_volume = (*NR22 & 0b11110000) >> 4;
    uint8_t env_dir = (*NR22 & 0b00001000) >> 3;
    uint8_t sweep_pace = (*NR22 & 0b00000111);
    printf("NR22: initial_volume=%d, envelope_direction=%d, sweep_pace=%d\n", initial_volume, env_dir, sweep_pace);

    //NR24
    uint8_t trigger = (nr24 & 0b10000000) >> 7;
    uint8_t length_enable = (nr24 & 0b01000000) >> 6;
    printf("NR24: trigger=%d, length_enable=%d\n", trigger, length_enable);
}

void check_joyp() {
    if (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            SDL_Quit();
            exit(1);
        } else if (event.type == SDL_KEYDOWN) {
            SDL_Keycode k = event.key.keysym.sym;
            switch (k) {
                case SDLK_UP:    up = true; break;
                case SDLK_DOWN:  down = true; break;
                case SDLK_LEFT:  left = true; break;
                case SDLK_RIGHT: right = true; break;
                case SDLK_s:     start = true; break;
                case SDLK_d:     sel = true; break;
                case SDLK_a:     a = true; break;
                case SDLK_b:     b = true; break;
                case SDLK_F11:  
                                 toggle_fullscreen(window);
                                 return;
            }
            *IF |= 0b00010000;  // Request joypad interrupt
        } else if (event.type == SDL_KEYUP) {
            SDL_Keycode k = event.key.keysym.sym;
            switch (k) {
                case SDLK_UP:    up = false; break;
                case SDLK_DOWN:  down = false; break;
                case SDLK_LEFT:  left = false; break;
                case SDLK_RIGHT: right = false; break;
                case SDLK_s:     start = false; break;
                case SDLK_d:     sel = false;   break;
                case SDLK_a:     a = false;  break;
                case SDLK_b:     b = false;  break;
            }
        }
    }
}

void toggle_fullscreen(SDL_Window* window) {
    bool isFullscreen = SDL_GetWindowFlags(window) & SDL_WINDOW_FULLSCREEN;
    /*
       SDL_DisplayMode dm;
       SDL_GetCurrentDisplayMode(0, &dm);
       int width = dm.w;
       int height = dm.h;
       */
    if (isFullscreen) {
        SDL_SetWindowFullscreen(window, 0);
    } else {
        SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN);
    }
}

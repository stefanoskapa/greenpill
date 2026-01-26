#include <SDL2/SDL.h>
#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "cpu.h"
#include "apu.h"
#include "ppu.h"

#define CYCLES_PER_FRAME 70224  // 154 scanlines × 456 cycles

uint8_t mem_read8(uint16_t addr);
void show_cartridge_info();
void mem_write8(uint16_t addr, uint8_t b);
void mem_write16(uint16_t addr, uint16_t b);
void timer_step(int cycles);
uint8_t *get_tile(int index);
void check_joyp();
static void load_rom(char *filename);

uint8_t logo[] = {
    0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D,
    0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99,
    0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E
};

char *cart_type[] = {
    [0x00] = "ROM ONLY", [0x01] = "MBC1", [0x02] = "MBC1+RAM", [0x03] = "MBC1+RAM+BATTERY",
    [0x05] = "MBC2", [0x06] = "MBC2+BATTERY", [0x08] = "ROM+RAM", [0x09] = "ROM+RAM+BATTERY",
    [0x0B] = "MMM01", [0x0C] = "MMM01+RAM", [0x0D] = "MMM01+RAM+BATTERY", [0x0F] = "MBC3+TIMER+BATTERY",
    [0x10] = "MBC3+TIMER+RAM+BATTERY", [0x11] = "MBC3", [0x12] = "MBC3+RAM", [0x13] = "MBC3+RAM+BATTERY",
    [0x19] = "MBC5", [0x1A] = "MBC5+RAM", [0x1B] = "MBC5+RAM+BATTERY", [0x1C] = "MBC5+RUMBLE",
    [0x1D] = "MBC5+RUMBLE+RAM", [0x1E] = "MBC5+RUMBLE+RAM+BATTERY", [0x20] = "MBC6",
    [0x22] = "MBC7+SENSOR+RUMBLE+RAM+BATTERY", [0xFC] = "POCKET CAMERA", [0xFD] = "BANDAI TAMA5",
    [0xFE] = "HuC3", [0xFF] = "HuC1+RAM+BATTERY"
};

static char *rom_sizes[] = {
    [0x00] = "32 KiB", [0x01] = "64 KiB", [0x02] = "128 KiB", [0x03] = "256 KiB",
    [0x04] = "512 KiB", [0x05] = "1 MiB", [0x06] = "2 MiB", [0x07] = "4 MiB",
    [0x08] = "8 MiB", [0x52] = "1.1 MiB", [0x53] = "1.2 MiB", [0x54] = "1.5 MiB"
};

static int rom_banks[] = {
    [0x00] = 2, [0x01] = 4, [0x02] = 8, [0x03] = 16, [0x04] = 32, [0x05] = 64,
    [0x06] = 128, [0x07] = 256, [0x08] = 512, [0x52] = 72, [0x53] = 80, [0x54] = 96
};

SDL_Event event;
bool debug = false;
bool apu_debug = false;
bool cpu_debug = false;
uint8_t mem[2000000];
/*
 * Serial transfer data
 */
uint8_t *SB = &mem[0xFF01];

/*
 * Serial transfer control
 *
 * - bit 0: clock select
 * - bit1: clock speed
 * - bit7: transfer enable
 */
uint8_t *SC = &mem[0xFF02];
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

//FF00 — P1/JOYP: Joypad
uint8_t *JOYP = &mem[0xFF00];
bool b_right = false;
bool b_up = false;
bool b_down = false;
bool b_left = false;
bool b_sel = false;
bool b_a = false;
bool b_b = false;
bool b_start = false;

int timer_counter = 0;
int div_counter = 0;

struct timespec frame_start, frame_end;


int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "No rom file was provided\n");
        exit(1);
    }
    if (argc > 2 && strcmp(argv[2],"-d") == 0) {
        debug = true;
    } 

    load_rom(argv[1]);

    show_cartridge_info();

    uint16_t cycles = 0;

    uint64_t frame_cycles = 0;

    cpu_init();
    apu_init();
    ppu_init();

    mem[0xFF00] = 0b11111111; //reset joypad
    *SC = 0b00000000;
    *SB = 0xFF;

    while (true) {

        cycles = cpu_step();
        if (debug) show_registers();

        timer_step(cycles);

        handle_interrupts();

        ppu_steps(cycles);

        apu_step(cycles);

        frame_cycles += cycles;

        if (frame_cycles >= CYCLES_PER_FRAME) {
            frame_cycles -= CYCLES_PER_FRAME;
            audio_delay();
            check_joyp();
        }
    }

    return EXIT_SUCCESS;
}

uint8_t mem_read8(uint16_t addr) {
    return mem[addr];
}

void mem_write8(uint16_t addr, uint8_t b) {
    if (addr == 0xFF02) { //SC
        if((b & 0b10000001) == 0b10000001) {
            *SB = 0b11111111; //unplugged
            *SC &= 0b01111111; // transfer done
            *IF |= 0b00001000; //request serial interrupt
            return;
        }
    }
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
            if (b_right) *JOYP &= 0b11111110; else *JOYP |= 0b00000001; 
            if (b_left) *JOYP &= 0b11111101; else *JOYP |= 0b00000010; 
            if (b_up) *JOYP &= 0b11111011; else *JOYP |= 0b00000100; 
            if (b_down) *JOYP &= 0b11110111; else *JOYP |= 0b00001000; 
        } else if ((*JOYP & 0b00100000) == 0) { 
            if (b_a) *JOYP &= 0b11111110; else *JOYP |= 0b00000001; 
            if (b_b) *JOYP &= 0b11111101; else *JOYP |= 0b00000010; 
            if (b_sel) *JOYP &= 0b11111011; else *JOYP |= 0b00000100; 
            if (b_start) *JOYP &= 0b11110111; else *JOYP |= 0b00001000; 
        }
        return;
    }

    apu_memw_callback(addr, b);

    if (addr < 0x8000 || (addr >= 0xFEA0 && addr < 0xFF00)) {
        if (debug) printf("Write to 0x%04X ignored\n", addr);
    } else if (addr >= 0xE000 && addr < 0xFE00) {
        if (debug) puts("Echo RAM -> WRAM mirror");
        mem[addr - 0x2000] = b;  // Mirror to WRAM (0xE000 -> 0xC000)
    } else {
        mem[addr] = b;
    }
}

void mem_write16(uint16_t addr, uint16_t b) {
    mem_write8(addr, (b & 0xFF));
    mem_write8(addr + 1, (b >> 8) & 0xFF);
}

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
    bool logo_found = memcmp(logo, mem + 0x104, 48) == 0;
    printf("Genuine Nintendo logo found: %s\n", logo_found ? "YES" : "NO");
    printf("Title: %s\n", mem + 0x134);
    printf("Cartridge compatibility: %s\n", cart_comp);
    printf("Cartridge type: %s\n", cart_type[mem[0x0147]]);
    uint8_t ram_size = mem[0x0149];
    printf("Cartridge RAM: ");
    if (ram_size == 0x00) printf("none\n");
    else if (ram_size == 0x02) printf("8 KiB\n");
    else if (ram_size == 0x03) printf("32 KiB\n");
    else if (ram_size == 0x04) printf("128 KiB\n");
    else if (ram_size == 0x05) printf("64 KiB\n");
    printf("SGB support: %s\n", mem[0x0146] == 0x03 ? "YES" : "NO");
    uint8_t rom_size = mem[0x0148];
    printf("ROM size: %s\n", rom_sizes[rom_size]);
    printf("ROM banks: %d\n", rom_banks[rom_size]);
    puts(""); 
}

void timer_step(int cycles) {
    // DIV register increments every 256 cycles
    div_counter += cycles;
    if (div_counter >= 256) {
        div_counter -= 256;
        mem[0xFF04]++;  // DIV register

        //
        if (mem[0xFF04] % 512 == 0) { // every 64 Hz

        } 

        if (mem[0xFF04] % 64 == 0) {

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

void check_joyp() {
    if (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            SDL_Quit();
            exit(1);
        } else if (event.type == SDL_KEYDOWN) {
            SDL_Keycode k = event.key.keysym.sym;
            switch (k) {
                case SDLK_UP:    b_up = true; break;
                case SDLK_DOWN:  b_down = true; break;
                case SDLK_LEFT:  b_left = true; break;
                case SDLK_RIGHT: b_right = true; break;
                case SDLK_s:     b_start = true; break;
                case SDLK_d:     b_sel = true; break;
                case SDLK_a:     b_a = true; break;
                case SDLK_b:     b_b = true; break;
            }
            *IF |= 0b00010000;  // Request joypad interrupt
        } else if (event.type == SDL_KEYUP) {
            SDL_Keycode k = event.key.keysym.sym;
            switch (k) {
                case SDLK_UP:    b_up = false; break;
                case SDLK_DOWN:  b_down = false; break;
                case SDLK_LEFT:  b_left = false; break;
                case SDLK_RIGHT: b_right = false; break;
                case SDLK_s:     b_start = false; break;
                case SDLK_d:     b_sel = false;   break;
                case SDLK_a:     b_a = false;  break;
                case SDLK_b:     b_b = false;  break;
            }
        }
    }
}

static void load_rom(char *filename) {
    FILE *rom_file = fopen(filename, "rb");
    if (rom_file == NULL) {
        fprintf(stderr, "Could not access file: %s\n", filename);
        exit(1);
    }

    fseek(rom_file, 0, SEEK_END);
    uint32_t size = ftell(rom_file);
    rewind(rom_file);
    fread(mem, 1, size, rom_file);
    fclose(rom_file);
}

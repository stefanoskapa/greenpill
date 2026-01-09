#include <SDL2/SDL.h>
#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

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

int cpu_step(void);
int ppu_step(int cycles);
void show_registers(void);
void show_cartridge_info();
void mem_write8(uint16_t addr, uint8_t b);
void mem_write16(uint16_t addr, uint16_t b);
void dec_reg16(uint8_t *low, uint8_t *high);
void inc_reg16(uint8_t *low, uint8_t *high);
void timer_step(int cycles);
void handle_interrupts(void);

bool debug = false;
uint8_t rom[2000000];

bool IME = false;           // Interrupt Master Enable flag (write only) IME is unset (interrupts are disabled) when the game starts running.
bool ime_scheduled = false; 

uint8_t A = 0;  // A
uint8_t F = 0; // Flags register (can't be accesed directly)
uint8_t B = 0; //BC high
uint8_t C = 0; //BC low
uint8_t D = 0; // DE high
uint8_t E = 0; // DE low
uint8_t H = 0; // HL high
uint8_t L = 0; // HL low

uint16_t SP = 0;
uint16_t PC = 0x0100;

int ppu_dots = 0;
uint8_t *ppu_ly = &rom[0xFF44]; //0xFF44h 

int main(int argc, char **argv) {

    if (argc < 2) {
        fprintf(stderr, "No rom file was provided\n");
        exit(1);
    }

    FILE *rom_file = fopen(argv[1], "rb");
    if (rom_file == NULL) {
        fprintf(stderr, "Could not access file: %s\n", argv[1]);
        exit(1);
    }

    fseek(rom_file, 0, SEEK_END);
    uint32_t size = ftell(rom_file);
    rewind(rom_file);
    fread(rom, 1, size, rom_file);
    fclose(rom_file);

    printf("ROM file is %u bytes\n", size);
    show_cartridge_info();

    uint16_t cycles = 0;

    /*
       SDL_Init(SDL_INIT_VIDEO);
       SDL_Window *window = SDL_CreateWindow(
       "GB Emulator",
       SDL_WINDOWPOS_CENTERED,
       SDL_WINDOWPOS_CENTERED,
       160 * 3, 144 * 3,
       SDL_WINDOW_SHOWN
       f);

       SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);
       */

    while (true) {

        cycles = cpu_step();

        timer_step(cycles);

        if (IME == true) {
            handle_interrupts();
        }

        ppu_step(cycles);
    }

    return EXIT_SUCCESS;
}

// 160x144
// GB has three layers: the Background, the Window and the Objects
//
// TILE: pixels grouped in 8x8 squares. The base unit in GB graphics
//       a tile assigns a color index to each of its pixels from 0 to 3 (2 bits)
//       Tile data is stored in VRAM 0x8000 - 0x97FF so we can store up to 384 tiles.:
//              (6143 bytes / (64 bits * 2 / 8) = 384
//
// OBJECT: 1 or 2 stacked tiles (8x8 or 8x16) that can be displayed anywhere on the screen
//
// Mode order: 2 -> 3 -> 0 (loop until frame complete) -> 1 
int ppu_step(int cycles) {
    ppu_dots += cycles;

    if (ppu_dots >= 456) {
        ppu_dots -= 456;
        if (*ppu_ly < 144) {
            //render scanline (accumulate into a buffer)
        }
        (*ppu_ly)++;

        if (*ppu_ly == 144) {
            //show frame
        }
        if (*ppu_ly > 153) {
            *ppu_ly = 0;
        }
    }
    if (*ppu_ly >= 144) {
        //VBlank mode (mode 1)
    } else if (ppu_dots <= 80) {
        // OAM scan (mode 2)
    } else if (ppu_dots <= 80 + 172) {
        // drawing (mode 3) 
    } else {
        // HBlank (mode 0)
    }

    return 0;
}

int cpu_step(void) {

    uint8_t n8 = 0;   // 8-bit integer signed or unsigned
    uint16_t n16 = 0; // 16-bit integer signed or unsigned
    int8_t e8 = 0;    // 8-bit signed offset!
    uint8_t a8 = 0;   // unsigned Offset added to 0xFF00
    uint16_t a16 = 0; // absolute offset, always unsigned
    if (ime_scheduled) {
        IME = true;
        ime_scheduled = false;
    }
    uint8_t op = rom[PC];
    if (debug) printf("0x%04X: ", PC);

    switch (op) {
        case 0x00: // NOP    b1 c4 flags:----
            if (debug) printf("NOP\n");
            PC += 1; 
            return 4;
        case 0x01: // LD BC, <n16>    b3 c12 flags:----
            B = rom[PC + 2];
            C = rom[PC + 1];
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
            B = rom[PC + 1];
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
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
            if (debug) printf("LD [0x%04X], SP\n", a16);
            rom[a16] = (SP & 0xFF);
            rom[a16 + 1] = (SP >> 8);
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
            A = rom[C | (B << 8)];
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
            C = rom[PC + 1];
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
            n16 = rom[PC + 1] | (rom[PC + 2] << 8);
            if (debug) printf("LD DE, 0x%04X\n", n16);
            D = rom[PC + 2];
            E = rom[PC + 1];
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
            n8 = rom[PC + 1];
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
            e8 = (int8_t)rom[PC + 1];
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
            A = rom[E | (D << 8)];
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
            n8 = rom[PC + 1];
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
            e8 = (int8_t)rom[PC + 1];
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
            H = rom[PC + 2];
            L = rom[PC + 1];
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
            H = rom[PC + 1];
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
            e8 = (int8_t)rom[PC + 1];
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
            A = rom[L | (H << 8)];
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
            L = rom[PC + 1];
            if (debug) printf("LD L, 0x%02X\n", rom[PC + 1]);
            PC += 2;
            return 8;
        case 0x2F: // CPL    b1 c4 flags:-11-
            if (debug) printf("CPL\n");
            A = ~A;
            SETF_N; SETF_H;
            PC += 1;
            return 4;
        case 0x30: // JR NC, <e8>    b2 c12,8 flags:----
            e8 = (int8_t)rom[PC + 1];
            if (debug) printf("JR NC, 0x%04X\n", PC + 2 + e8);
            PC += 2;
            if (!READF_C) {
                PC += e8;
                return 12;
            }
            return 8;
        case 0x31: // LD SP, <n16>    b3 c12 flags:----
            n16 = rom[PC + 1] | (rom[PC + 2] << 8);
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
            rom[HL] = rom[HL] + 1;
            if (rom[HL] == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            if ((rom[HL] & 0x0F) == 0x00) SETF_H; else CLRF_H;
            PC += 1;
            return 12;
        case 0x35: // DEC [HL]    b1 c12 flags:Z1H-
            if (debug) printf("DEC [HL]\n");
            rom[HL] = rom[HL] - 1;
            if (rom[HL] == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            if ((rom[HL] & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            PC += 1;
            return 12;
        case 0x36: // LD [HL], <n8>    b2 c12 flags:----
            n8 = rom[PC + 1];
            if (debug) printf("LD [HL], 0x%02X\n", n8);
            mem_write8(HL, n8);
            PC += 2;
            return 12;
        case 0x38: // JR C, <e8>    b2 c12,8 flags:----
            e8 = (int8_t)rom[PC + 1];
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
            A = rom[L | (H << 8)];
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
            A = rom[PC + 1];
            if (debug) printf("LD A, 0x%02X\n", rom[PC + 1]);
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
            B = rom[HL];
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
            C = rom[HL];
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
            D = rom[HL];
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
            E = rom[HL];
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
            H = rom[HL];
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
            L = rom[HL];
            PC += 1;
            return 8;
        case 0x6F: // LD L, A  b1 c4 flags:----
            if (debug) printf("LD L, A\n");
            L = A;
            PC += 1;
            return 8;
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
            {
                uint8_t IE = rom[0xFFFF];
                uint8_t IF = rom[0xFF0F];

                if (IME) {
                    // IME set: wait for interrupt, handler will be called
                    while ((IE & IF & 0x1F) == 0) {
                        timer_step(4);
                        ppu_step(4);
                        IE = rom[0xFFFF];
                        IF = rom[0xFF0F];
                    }
                    PC += 1;
                } else if ((IE & IF & 0x1F) == 0) {
                    // IME not set, no interrupt pending: wait, but handler won't be called
                    while ((IE & IF & 0x1F) == 0) {
                        timer_step(4);
                        ppu_step(4);
                        IE = rom[0xFFFF];
                        IF = rom[0xFF0F];
                    }
                    PC += 1;
                } else {
                    // IME not set, interrupt pending: HALT bug - don't increment PC
                    // Next byte will be read twice
                    PC += 0;  // Don't increment - this is the HALT bug
                }
            }
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
            A = rom[HL];
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
            PC += 2;
            return 8;
        case 0x81: // ADD A, C    b1 c4 flags:Z0HC
            if (debug) printf("ADD A, C\n");
            if (A + C > 0xFF) SETF_C; else CLRF_C;
            if ( (A & 0x0F) + (C & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
            A += C;
            if (A == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 2;
            return 8;
        case 0x82: // ADD A, D    b1 c4 flags:Z0HC
            if (debug) printf("ADD A, D\n");
            if (A + D > 0xFF) SETF_C; else CLRF_C;
            if ( (A & 0x0F) + (D & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
            A += D;
            if (A == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 2;
            return 8;
        case 0x83: // ADD A, E    b1 c4 flags:Z0HC
            if (debug) printf("ADD A, E\n");
            if (A + E > 0xFF) SETF_C; else CLRF_C;
            if ( (A & 0x0F) + (E & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
            A += E;
            if (A == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 2;
            return 8;
        case 0x84: // ADD A, H    b1 c4 flags:Z0HC
            if (debug) printf("ADD A, H\n");
            if (A + H > 0xFF) SETF_C; else CLRF_C;
            if ( (A & 0x0F) + (H & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
            A += H;
            if (A == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 2;
            return 8;
        case 0x85: // ADD A, L    b1 c4 flags:Z0HC
            if (debug) printf("ADD A, L\n");
            if (A + L > 0xFF) SETF_C; else CLRF_C;
            if ( (A & 0x0F) + (L & 0x0F) > 0x0F ) SETF_H; else CLRF_H;
            A += L;
            if (A == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 2;
            return 8;
        case 0x86: // ADD A, [HL]    b1 c8 flags:Z0HC
            n8 = rom[HL];
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
            PC += 2;
            return 8;
        case 0x88: { // ADC A, B    b1 c4 flags:Z0HC
            if (debug) printf("ADC A, B\n");
            uint8_t carry = READF_C;
            uint16_t result = A + B + carry;
            if (((A & 0x0F) + (B & 0x0F) + carry) > 0x0F) SETF_H; else CLRF_H;
            if (result > 0xFF) SETF_C; else CLRF_C;
            A = (uint8_t)result;
            if (A == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 2;
            return 8;
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
            PC += 2;
            return 8;
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
            PC += 2;
            return 8;
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
            PC += 2;
            return 8;
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
            PC += 2;
            return 8;
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
            PC += 2;
            return 8;
            }
        case 0x8E: { // ADC A, [HL]    b1 c8 flags:Z0HC
                       n8 = rom[HL];
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
            PC += 2;
            return 8;
            }
        case 0x90: // SUB A, B    b1 c4 flags:Z1HC
           if (debug) printf("SUB A, B\n");
           if (B > A) SETF_C; else CLRF_C;
           if ((B & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
           SETF_N;
           A -= B;
           if (A == 0) SETF_Z; else CLRF_Z;
           PC += 2;
           return 4;
        case 0x91: // SUB A, C    b1 c4 flags:Z1HC
           if (debug) printf("SUB A, C\n");
           if (C > A) SETF_C; else CLRF_C;
           if ((C & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
           SETF_N;
           A -= C;
           if (A == 0) SETF_Z; else CLRF_Z;
           PC += 2;
           return 4;
        case 0x92: // SUB A, D    b1 c4 flags:Z1HC
           if (debug) printf("SUB A, D\n");
           if (D > A) SETF_C; else CLRF_C;
           if ((D & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
           SETF_N;
           A -= D;
           if (A == 0) SETF_Z; else CLRF_Z;
           PC += 2;
           return 4;
        case 0x93: // SUB A, E    b1 c4 flags:Z1HC
           if (debug) printf("SUB A, E\n");
           if (E > A) SETF_C; else CLRF_C;
           if ((E & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
           SETF_N;
           A -= E;
           if (A == 0) SETF_Z; else CLRF_Z;
           PC += 2;
           return 4;
        case 0x94: // SUB A, H    b1 c4 flags:Z1HC
           if (debug) printf("SUB A, H\n");
           if (H > A) SETF_C; else CLRF_C;
           if ((H & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
           SETF_N;
           A -= H;
           if (A == 0) SETF_Z; else CLRF_Z;
           PC += 2;
           return 4;
        case 0x95: // SUB A, L    b1 c4 flags:Z1HC
           if (debug) printf("SUB A, L\n");
           if (L > A) SETF_C; else CLRF_C;
           if ((L & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
           SETF_N;
           A -= L;
           if (A == 0) SETF_Z; else CLRF_Z;
           PC += 2;
           return 4;
        case 0x96: // SUB A, [HL]    b1 c8 flags:Z1HC
                   n8 = rom[HL];
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
           PC += 2;
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
                   n8 = rom[HL];
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
                   n8 = rom[HL];
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
            A = A ^ rom[HL];
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
            A = A | rom[HL];
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
                   n8 = rom[HL];
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
                uint8_t low5 = rom[SP];
                uint8_t high5 = rom[SP + 1];
                SP += 2;
                PC = (high5 << 8) | low5;
                return 20;
            }
            PC += 1;
            return 8;
        case 0xC1: // POP BC    b1 c12 flags:----
            C = rom[SP];
            B = rom[SP + 1];
            SP += 2;
            PC += 1;
            return 12;
        case 0xC2: // JP NZ <a16>    b3 c16,12 flags:----
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
            if (debug) printf("JP NZ 0x%04X", a16);
            if (!READF_Z) {
                PC = a16;
                return 16;
            }
            PC += 3;
            return 12;
        case 0xC3: // JP <a16>    b3 c16 flags:----
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
            if (debug) printf("JP 0x%04X\n", a16);
            PC = a16; 
            return 16;
        case 0xC4: // CALL NZ, <a16>    b3 c24,12 flags:----
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
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
            n8 = rom[PC + 1];
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
                uint8_t low5 = rom[SP];
                uint8_t high5 = rom[SP + 1];
                SP += 2;
                PC = (high5 << 8) | low5;
                return 20;
            }
            PC += 1;
            return 8;
        case 0xC9: // RET b1 c16 flags:----
            if (debug) printf("RET \n");
            {}
            uint8_t low = rom[SP];
            uint8_t high = rom[SP + 1];
            SP += 2;
            PC= (high << 8) | low;
            return 16;
        case 0xCA: // JP Z <a16>    b3 c16,12 flags:----
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
            if (debug) printf("JP Z 0x%04X", a16);
            if (READF_Z) {
                PC = a16;
                return 16;
            }
            PC += 3;
            return 12;
        case 0xCB: { // PREFIX b1 c4 flags:----
                       uint8_t cb_op = rom[PC + 1];
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
                           case 6: hl_val = rom[HL]; reg = &hl_val; cycles = 16; break;
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
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
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
                   a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
                   if (debug) printf("CALL 0x%04X\n", a16);
                   SP -= 2;
                   mem_write16(SP, PC + 3);
                   PC = a16;
                   return 24;
        case 0xCE: { // ADC A, <n8>    b2 c8 flags:Z0HC
                       n8 = rom[PC + 1];
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
                       uint8_t low4 = rom[SP];
                       uint8_t high4 = rom[SP + 1];
                       SP += 2;
                       PC = (high4 << 8) | low4;
                       return 20;
                   }
                   PC += 1;
                   return 8;
        case 0xD1: // POP DE    b1 c12 flags:----
                   if (debug) printf("POP DE\n");
                   E = rom[SP];
                   D = rom[SP + 1];
                   SP += 2;
                   PC += 1;
                   return 12;
        case 0xD2: // JP NC <a16>    b3 c16,12 flags:----
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
            if (debug) printf("JP NC 0x%04X", a16);
            if (!READF_C) {
                PC = a16;
                return 16;
            }
            PC += 3;
            return 12;
        case 0xD4: // CALL NC, <a16>    b3 c24,12 flags:----
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
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
                   n8 = rom[PC + 1];
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
                       uint8_t low5 = rom[SP];
                       uint8_t high5 = rom[SP + 1];
                       SP += 2;
                       PC = (high5 << 8) | low5;
                       return 20;
                   }
                   PC += 1;
                   return 8;
        case 0xD9: // RETI    b1 c16 flags:----
            if (debug) printf("RET \n");
            {
            uint8_t low = rom[SP];
            uint8_t high = rom[SP + 1];
            ime_scheduled = true;
            SP += 2;
            PC = (high << 8) | low;
            }
            return 16;
        case 0xDA: // JP C <a16>    b3 c16,12 flags:----
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
            if (debug) printf("JP C 0x%04X", a16);
            if (READF_C) {
                PC = a16;
                return 16;
            }
            PC += 3;
            return 12;
        case 0xDC: // CALL C, <a16>    b3 c24,12 flags:----
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
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
                   n8 = rom[PC + 1];
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
                   a8 = rom[PC + 1];
                   uint16_t addr = 0xFF00 + a8;
                   if (debug) printf("LDH 0x%04X, A\n", addr);
                   mem_write8(addr, A);
                   PC += 2;
                   return 12;
        case 0xE1: // POP HL    b1 c12 flags:----
                   {}
                   uint8_t low1 = rom[SP];
                   uint8_t high1 = rom[SP + 1];
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
                   n8 = rom[PC + 1];
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
                   e8 = (int8_t)rom[PC + 1];
                   if (debug) printf("ADD SP, 0x%02X\n", e8);
                   {
                       uint16_t result = SP + e8;
                       uint8_t unsigned_e8 = (uint8_t)rom[PC + 1];  // Use unsigned for flag calc

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
                   a16 = rom[PC + 1] | (rom[PC + 2] << 8);
                   if (debug) printf("LD 0x%04X, A\n", a16);
                   mem_write8(a16, A);
                   PC += 3;
                   return 16;
        case 0xEE: // XOR A, <n8>    b2 c8 flags:Z000
                   n8 = rom[PC + 1];
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
        case 0xF0: // LDH A, <a8>    b2 c12 flags:----
                   a8 = rom[PC + 1];
                   A = rom[0xFF00 + a8];
                   if (debug) printf("LDH A, 0x%04X\n", (0xFF00 + a8));
                   PC += 2;
                   return 12;
        case 0xF1: // POP AF    b1 c12 flags:ZNHC
                   if (debug) printf("POP AF\n");
                   {
                       uint8_t low2 = rom[SP];
                       uint8_t high2 = rom[SP + 1];
                       SP += 2;
                       F = low2 & 0xF0;
                       A = high2;
                   }
                   PC += 1;
                   return 12;
        case 0xF2: // LDH A, [0xFF00 + C]    b1 c8 flags:----
                   if (debug) printf("LDH A, [0xFF00 + C]\n");
                   A = rom[0xFF00 + C];
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
                   n8 = rom[PC + 1];
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
                   e8 = (int8_t)rom[PC + 1];
                   if (debug) printf("LD HL, SP+0x%02X\n", e8);
                   {
                       uint16_t result = SP + e8;
                       uint8_t unsigned_e8 = (uint8_t)rom[PC + 1];  // Use unsigned for flag calc

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
                   a16 = rom[PC + 1] | (rom[PC + 2] << 8);
                   if (debug) printf("LD A, [0x%04X]\n", a16);
                   A = rom[a16];
                   PC += 3;
                   return 16;
        case 0xFB: // EI    b1 c4 flags:----
                   if (debug) printf("EI\n");
                   ime_scheduled = true;
                   PC += 1;
                   return 4;
        case 0xFE: // CP A, <n8>    b2 c8 flags:Z1HC
                   n8 = rom[PC + 1];
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
    if (debug) show_registers();
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
void mem_write8(uint16_t addr, uint8_t b) {
    if (debug) printf("Writing to ");
    if (addr < 0x4000) {
        if (debug)   printf("ROM bank 0 (ignored)\n");
    } else if (addr < 0x8000) {
        if (debug) printf("ROM bank 1 (ignored)\n");
    } else if (addr < 0xA000) {
        if (debug) printf("8 KiB Video RAM (VRAM)\n");
        rom[addr] = b;
    } else if (addr < 0xC000) {
        if (debug) printf("8 KiB External RAM\n");
        rom[addr] = b;
    } else if (addr < 0xE000) {
        if (debug) if (debug) printf("4 KiB Work RAM (WRAM)\n");
        rom[addr] = b;
    } else if (addr < 0xFE00) {
        if (debug) printf("Echo RAM (ignored)\n");
    } else if (addr < 0xFEA0) {
        if (debug) printf("Object attribute memory (OAM)\n");
        rom[addr] = b;
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
        rom[addr] = b;
    } else if (addr < 0xFFFF) {
        if (debug) printf("High RAM (HRAM)\n");
        rom[addr] = b;
    } else {
        if (debug) printf("Interrupt Enable register (IE)\n");
        rom[addr] = b;
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
        rom[addr] = b & 0xFF;
        rom[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xC000) {
        if (debug) printf("8 KiB External RAM\n");
        rom[addr] = b & 0xFF;
        rom[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xE000) {
        if (debug) printf("4 KiB Work RAM (WRAM)\n");
        rom[addr] = b & 0xFF;
        rom[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xFE00) {
        if (debug) printf("Echo RAM (ignored)\n");
    } else if (addr < 0xFEA0) {
        if (debug) printf("Object attribute memory (OAM)\n");
        rom[addr] = b & 0xFF;
        rom[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xFF00) {
        if (debug)  printf("Not Usable (ignored)\n");
    } else if (addr < 0xFF80) {
        if (debug) printf("I/O Registers\n");
        rom[addr] = b & 0xFF;
        rom[addr + 1] = (b >> 8) & 0xFF;
    } else if (addr < 0xFFFF) {
        if (debug) printf("High RAM (HRAM)\n");
        rom[addr] = b & 0xFF;
        rom[addr + 1] = (b >> 8) & 0xFF;
    } else {
        if (debug)  printf("Interrupt Enable register (IE)\n");
        rom[addr] = b & 0xFF;
        rom[addr + 1] = (b >> 8) & 0xFF;
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

    uint8_t cgb_flag = rom[0x0143];
    char *cart_comp;
    bool is_old_cart = true;
    if (cgb_flag == 0x80) {
        cart_comp = "CGB-compatible";
    } else if (cgb_flag == 0xC0) {
        cart_comp = "CGB-only";
    } else {
        cart_comp = "DMG";
        is_old_cart = false;
    }
    bool is_old_licensee = rom[0x014B] != 0x33;
    bool logo_found = memcmp(logo, rom + 0x104, 48) == 0;
    printf("Genuine Nintendo logo found: %s\n", logo_found ? "YES" : "NO");
    printf("Title: %s\n", rom + 0x134);
    printf("Cartridge compatibility: %s\n", cart_comp);
    printf("Cartridge type: %s\n", cart_type[rom[0x0147]]);
    printf("SGB support: %s\n", rom[0x0146] == 0x03 ? "YES" : "NO");
    if (is_old_licensee) {
        printf("Publisher: %s\n", old_licensee_codes[rom[0x014b]]);
    }
    uint8_t rom_size = rom[0x0148];
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
    uint8_t ram_size = rom[0x0149];
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
        rom[0xFF04]++;  // DIV register
    }

    uint8_t TAC = rom[0xFF07];
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
        rom[0xFF05]++;  // TIMA register

        if (rom[0xFF05] == 0) {  // Overflow
            rom[0xFF05] = rom[0xFF06];  // Reset to TMA
            rom[0xFF0F] |= 0x04;  // Request timer interrupt
        }
    }
}

void handle_interrupts(void) {

            uint8_t IE = rom[0xFFFF]; //interrupt enable flag
            uint8_t IF = rom[0xFF0F]; //interrupt flag
            if ((IE & IF & 0b00000001) == 0b00000001) {
                // VBlank
                IME = false;
                rom[0xFF0F] = (IF & 0b11111110);
                SP -= 2;
                mem_write16(SP, PC);
                PC = 0x40;
                if (debug) printf("<VBlank Interrupt>\n");
            } else if ((IE & IF & 0b00000010) == 0b00000010) {
                // LCD
                IME = false;
                rom[0xFF0F] = (IF & 0b11111101);
                SP -= 2;
                mem_write16(SP, PC);
                PC = 0x48;
                if (debug) printf("<STAT Interrupt>\n");
            } else if ((IE & IF & 0b00000100) == 0b00000100) {
                // Timer
                IME = false;
                rom[0xFF0F] = (IF & 0b11111011);
                SP -= 2;
                mem_write16(SP, PC);
                PC = 0x50;
                if (debug) printf("<Timer Interrupt>\n");
            } else if ((IE & IF & 0b00001000) == 0b00001000) {
                // serial
                IME = false;
                rom[0xFF0F] = (IF & 0b11110111);
                SP -= 2;
                mem_write16(SP, PC);
                PC = 0x58;
                if (debug) printf("<Serial Interrupt>\n");
            } else if ((IE & IF & 0b00010000) == 0b00010000) {
                //Joypad
                IME = false;
                rom[0xFF0F] = (IF & 0b11101111);
                SP -= 2;
                mem_write16(SP, PC);
                PC = 0x60;
                if (debug) printf("<Joypad Interrupt>\n");
            }
}

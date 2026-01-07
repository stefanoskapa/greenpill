#include <SDL2/SDL.h>
#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define AF  F | (A << 8)
#define BC  C | (B << 8)
#define DE  E | (D << 8)
#define HL  L | (H << 8)
// z - zero flag (7th bit)
#define SETF_Z F = (F | 0b10000000)
#define CLRF_Z F = (F & 0b01111111)
#define READF_Z (F & 0b10000000)
// n - subtraction flag (BCD) (6th bit)
#define SETF_N F = (F | 0b01000000)
#define CLRF_N F = (F & 0b10111111)
#define READF_N (F & 0b01000000)
// h - half-carry flag (BCD) (5th bit)
#define SETF_H F = (F | 0b00100000)
#define CLRF_H F = (F & 0b11011111)
#define READF_H (F & 0b00100000)
// c - carry flag (4th bit)
#define SETF_C F = (F | 0b00010000)
#define CLRF_C F = (F & 0b11101111)
#define READF_C (F & 0b00010000)

int cpu_step(void);
int ppu_step(int cycles);
void show_registers(void);
void show_cartridge_info();
void mem_write8(uint16_t addr, uint8_t b);
void mem_write16(uint16_t addr, uint16_t b);
void dec_reg16(uint8_t *low, uint8_t *high);
void inc_reg16(uint8_t *low, uint8_t *high);

uint8_t rom[2000000];

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

bool debug = true;
int ppu_dots = 0;
uint8_t *ppu_ly = &rom[0xFF44]; //0xFF44h 
//int ppu_scanline = 0;
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
    );

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);
*/
    while (true) {
       cycles =  cpu_step();
       if (debug == true) {
            show_registers();
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
    
    uint8_t n8 = 0;
    int8_t e8 = 0;
    uint8_t a8 = 0;
    
    uint16_t a16 = 0;
    uint16_t n16 = 0;

    uint8_t op = rom[PC];
    if (debug) printf("0x%04X: ", PC);

    switch (op) {
        case 0x00: // NOP   c=4 b=1 flags=none
            if (debug) printf("NOP\n");
            PC += 1; 
            return 4;
        case 0x01: // LD BC, <n16>      c=12, b=3, flags=none
            B = rom[PC + 2];
            C = rom[PC + 1];
            n16 = C | (B << 8);
            if (debug) printf("LD BC, 0x%04X\n", n16);
            PC += 3;
            return 12;
        case 0x05:  // DEC B    c=4, b=1, flags= Z=Z N=1 H=H C=-
            if (debug) printf("DEC B\n");
            if ((B & 0x0F) == 0x00) SETF_H; else CLRF_H;
            B--;
            if (B == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            PC += 1;
            return 4;
        case 0x06: // LD B, <n8>        c=8, b=2, flags=none
            B = rom[PC + 1];
            if (debug) printf("LD B, 0x%02X\n", B);
            PC += 2;
            return 8;
        case 0x0B: // DEC BC  c=8, b=1, flags=none
            if (debug) printf("DEC BC\n");
            dec_reg16(&C, &B);
            PC += 1;
            return 8;
        case 0x0C: // INC C  c=4 b=1 flags: Z=Z N=0 H=H C=-
            if (debug) printf("INC C\n");
            if ((C & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            C++;
            if (C == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            PC += 1;
            return 4;

        case 0x0D: // DEC C c=4 b=1 flags: Z=Z, N=1, H=H, C=-
            if (debug) printf("DEC C\n");
            if ((C & 0x0F) == 0x00) SETF_H; else CLRF_H;
            C--;
            if (C == 0) SETF_Z; else CLRF_Z;
            SETF_N;
            PC += 1;
            return 4;
        case 0x0E: // LD C, <n8>        c=8, b=2, flags=none
            C = rom[PC + 1];
            if (debug) printf("LD C, 0x%02X\n", C);
            PC += 2;
            return 8;
        case 0x0F: // RRCA  c=4 b=1, flags= Z=0, N=0, H=0, C=C
            if (debug) printf("RRCA\n");
            CLRF_Z; CLRF_N; CLRF_H;
            if ((A & 0x00000001) == 0) CLRF_C; else SETF_C;
            A = (A >> 1) | (A << 7);         
            PC += 1;
            return 4;
        case 0x11: // LD DE, <n16>      b=3, c=12 flags=none
            n16 = rom[PC + 1] | (rom[PC + 2] << 8);
            if (debug) printf("LD DE, 0x%04X\n", n16);
            D = rom[PC + 2];
            E = rom[PC + 1];
            PC += 3;
            return 12;
        case 0x12: // LD [DE], A c=8, b=1 flags=none
            if (debug) printf("LD [DE], A\n");
            mem_write8(DE, A);
            PC += 1;
            return 8;
        case 0x14: //INC D b=1 c=4
            if (debug) printf("INC D\n");
            if ((D & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            CLRF_N;
            D++;
            if (D == 0) SETF_Z; else CLRF_Z;
            PC += 1;
            return 4;
        case 0x18: //JR <e8> b=2 c=12
            e8 = rom[PC + 1];
            if (debug) printf("JR 0x%04X\n", e8 + PC);
            PC += 2;
            PC += e8;
            return 12;
            /*
        case 0x14: // RL H  b=2 c=8 flags= Z=Z, N=0, H=0, C=C
            if (debug) printf("RL H\n");
            uint8_t temp = H & 0b10000000; 
            H = H << 1;
            if (READF_C != 0) H = H | 0b00000001; 
            if (temp == 0b10000000) SETF_C; else CLRF_C;
            if (H == 0) SETF_Z; else CLRF_Z;
            PC += 2;
            return 8;
            */
        case 0x1C: // INC E  c=4 b=1 flags Z=Z, N=0, H=H, C=-
            if (debug) printf("INC E\n");
            if ((E & 0x0F) == 0x0F) SETF_H; else CLRF_H;
            CLRF_N;
            E++;
            if (E == 0) SETF_Z; else CLRF_Z;
            PC += 1;
            return 4;
        case 0x20: //JR NZ, <e8> c=12,8 b=2 flags=none (signed relative offset 8 bit)
            e8 = rom[PC + 1];
            if (debug) printf("JR NZ, 0x%02X\n", e8);
            PC += 2;
            if (READF_Z == 0) {
                PC += e8;
                return 12;
            } else {
                return 8;
            }
            return 12; 
        case 0x21: // LD HL, <n16>      c=12, b=3, flags=none
            H = rom[PC + 2];
            L = rom[PC + 1];
            n16 = L | (H << 8);
            if (debug) printf("LD HL, 0x%04X\n", n16);
            PC += 3;
            return 12;
        case 0x23: //INC HL   b=1 c=8 flags = none
            if (debug) printf("INC HL\n");
            inc_reg16(&L, &H);
            PC += 1;
            return 8;
        case 0x2A:  //
            if (debug) printf("LD A, [HL+]\n");
            A = rom[L | (H << 8)];
            inc_reg16(&L, &H);
            PC += 1;
            return 8;
        case 0x31: //LD SP, <n16>   c=12, b=3, flags=none
           n16 = rom[PC + 1] | (rom[PC + 2] << 8);
           if (debug) printf("LD SP, 0x%04X\n", n16);
           SP = n16;
           PC += 3;
           return 12;
        case 0x32: // LD (HL-), A   c=8, b=1 flags=none
           if (debug) printf("LD [HL-], A\n");
           mem_write8(L | (H << 8), A); 
           dec_reg16(&L, &H);
           PC += 1;
           return 8;
        case 0x36: // LD [HL], <n8>  c=12, b=2 flags=none
           n8 = rom[PC + 1];
           if (debug) printf("LD [HL], 0x%02X\n", n8);
           mem_write8(HL, n8);
           PC += 2;
           return 12;
        case 0x3E: // LD A, <n8> c=8, b=2, flags= none 
            A = rom[PC + 1];
            if (debug) printf("LD A, 0x%02X\n", C);
            PC += 2;
            return 8;
        case 0x47: //LD B, A  c=4 b=1 flags=none
            if (debug) printf("LD B, A\n");
            B = A;
            PC += 1;
            return 8;
        case 0x78: //LD A, B  b=1, c=4 flags=none
            if (debug) printf("LD A, B\n");
            A = B;
            PC += 1;
            return 4;
        case 0x7C: // LD A, H b = 1, c=4, flags=none
            if (debug) printf("LD A, H\n");
            A = H;
            PC += 1;
            return 4;
        case 0x7D: // LD A, L b = 1, c=4, flags=none
            if (debug) printf("LD A, L\n");
            A = L;
            PC += 1;
            return 4;
        case 0xAF: // XOR A, A   c=4, b=1, flags=z
            A = 0;
            SETF_Z;
            CLRF_N;
            CLRF_H;
            CLRF_C;
            if (debug) printf("XOR A, A\n");
            PC += 1;
            return 4;
        case 0xB1: // OR A, C
            if (debug) printf("OR A, C\n");
            A = A | C;
            if (A == 0) SETF_Z; else CLRF_Z;
            CLRF_N;
            CLRF_H;
            CLRF_C;
            PC += 1;
            return 4;
        case 0xC3: // JP <a16>      c=16, b=3
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
            if (debug) printf("JP 0x%04X\n", a16);
            PC = a16; 
            return 16;
        case 0xC9: //RET b=1 c=16
            {}
            uint8_t low = rom[SP];
            uint8_t high = rom[SP + 1];
            SP += 2;
            PC= (high << 8) | low;
            return 16;
        case 0xCD: // CALL <a16> c=24, b=3 flags=none
            a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
            if (debug) printf("CALL 0x%04X\n", a16);
            SP -= 2;
            mem_write16(SP, PC + 3);
            PC = a16;
            return 24;
        case 0xE0: // LDH <a8>, A c=12, b = 2 flags=none
            a8 = rom[PC + 1];
            uint16_t addr = 0xFF00 + a8;
            if (debug) printf("LDH 0x%04X, A\n", addr);
            mem_write8(addr, A);
            PC += 2;
            return 12;
        case 0xE1: // POP HL b- 1 c=12 flags none
            {}
            uint8_t low1 = rom[SP];
            uint8_t high1 = rom[SP + 1];
            SP += 2;
            L = low1;
            H = high1;
            PC += 1;
            return 12;
        case 0xE2: //LDH [C], A  c=8, b=1
            if (debug) printf("LDH [C], A\n");
            mem_write8(0xFF00 + C, A);
            PC += 1;
            return 8;
        case 0xE5: // PUSH HL  b=1 c=16 flags=none
            if (debug) printf("PUSH HL\n");
            SP -= 2;
            mem_write16(SP, HL);
            PC += 1;
            return 16;
        case 0xEA: //LD [a16], A  c=16 b=3 flags=none
            a16 = rom[PC + 1] | (rom[PC + 2] << 8);
            if (debug) printf("LD 0x%04X, A\n", a16);
            mem_write8(a16, A);
            PC += 3;
            return 16;
        case 0xF0: // LDH A, <a8>  c=12 b=2 flags=none
            a8 = rom[PC + 1];
            A = rom[0xFF00 + a8];
            if (debug) printf("LDH A, 0x%04X\n", (0xFF00 + a8));
            PC += 2;
            return 12;
        case 0xF1: // POP AF b- 1 c=12 flags none
            {}
            uint8_t low2 = rom[SP];
            uint8_t high2 = rom[SP + 1];
            SP += 2;
            F = low2;
            A = high2;
            PC += 1;
            return 12;
        case 0xF3: // DI  b=1 c=4 flags=none 
            if (debug) printf("DI\n");
            //TODO: disable interrupts by clearing IME flag (?)
            PC += 1;
            return 4;
        case 0xF5: // PUSH HL  b=1 c=16 flags=none
            if (debug) printf("PUSH AF\n");
            SP -= 2;
            mem_write16(SP, AF);
            PC += 1;
            return 16;
        case 0xFE: // CP A, <n8>  c=8, b=2 flags=Z=Z, N=1, H=H, C=C
            n8 = rom[PC + 1];
            if (n8 == A) SETF_Z; else CLRF_Z;
            SETF_N;
            if ((n8 & 0x0F) > (A & 0x0F)) SETF_H; else CLRF_H;
            if (n8 > A) SETF_C; else CLRF_C;
            PC += 2;
            return 8;
        case 0xFF: // RST $38  c=16 b=1 flags=none
            if (debug) printf("RST $38\n");
            SP -= 2;
            mem_write16(SP, PC + 1); 
            PC = 0x0038;
            return 16;
        default:
            if (debug) printf("<UNKNOWN 0x%02X>\n", op);
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
void mem_write8(uint16_t addr, uint8_t b) {
    if (debug) printf("Writing to ");
   if (addr < 0x4000) {
       printf("ROM bank 0 (ignored)\n");
   } else if (addr < 0x8000) {
       printf("ROM bank 1 (ignored)\n");
   } else if (addr < 0xA000) {
       printf("8 KiB Video RAM (VRAM)\n");
       rom[addr] = b;
   } else if (addr < 0xC000) {
       printf("8 KiB External RAM\n");
       rom[addr] = b;
   } else if (addr < 0xE000) {
       if (debug) printf("4 KiB Work RAM (WRAM)\n");
       rom[addr] = b;
   } else if (addr < 0xFE00) {
       printf("Echo RAM (ignored)\n");
   } else if (addr < 0xFEA0) {
       printf("Object attribute memory (OAM)\n");
       rom[addr] = b;
   } else if (addr < 0xFF00) {
       printf("Not Usable (ignored)\n");
   } else if (addr < 0xFF80) {
       if( addr == 0xFF01) {
          val_char = b; 
       } else if(addr == 0xFF02 && b == 0x81) {
           putchar(val_char);
           fflush(stdout);
       }
       printf("I/O Registers\n");
       rom[addr] = b;
   } else if (addr < 0xFFFF) {
       printf("High RAM (HRAM)\n");
       rom[addr] = b;
   } else {
       printf("Interrupt Enable register (IE)\n");
       rom[addr] = b;
   }
}
void mem_write16(uint16_t addr, uint16_t b) {
    printf("Writing to ");
   if (addr < 0x4000) {
       printf("ROM bank 0 (ignored)\n");
   } else if (addr < 0x8000) {
       printf("ROM bank 1 (ignored)\n");
   } else if (addr < 0xA000) {
       printf("8 KiB Video RAM (VRAM)\n");
       rom[addr] = b & 0xFF;
       rom[addr + 1] = (b >> 8) & 0xFF;
   } else if (addr < 0xC000) {
       printf("8 KiB External RAM\n");
       rom[addr] = b & 0xFF;
       rom[addr + 1] = (b >> 8) & 0xFF;
   } else if (addr < 0xE000) {
       printf("4 KiB Work RAM (WRAM)\n");
       rom[addr] = b & 0xFF;
       rom[addr + 1] = (b >> 8) & 0xFF;
   } else if (addr < 0xFE00) {
       printf("Echo RAM (ignored)\n");
   } else if (addr < 0xFEA0) {
       printf("Object attribute memory (OAM)\n");
       rom[addr] = b & 0xFF;
       rom[addr + 1] = (b >> 8) & 0xFF;
   } else if (addr < 0xFF00) {
       printf("Not Usable (ignored)\n");
   } else if (addr < 0xFF80) {
       printf("I/O Registers\n");
       rom[addr] = b & 0xFF;
       rom[addr + 1] = (b >> 8) & 0xFF;
   } else if (addr < 0xFFFF) {
       printf("High RAM (HRAM)\n");
       rom[addr] = b & 0xFF;
       rom[addr + 1] = (b >> 8) & 0xFF;
   } else {
       printf("Interrupt Enable register (IE)\n");
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
   if ((F & 0b01000000) != 0) printf("Z:1 "); else printf("Z:0 ");
   if ((F & 0b00100000) != 0) printf("N:1 "); else printf("N:0 ");
   if ((F & 0b00010000) != 0) printf("H:1 "); else printf("H:0 ");
   if ((F & 0b00001000) != 0) printf("C:1 "); else printf("C:0 ");
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

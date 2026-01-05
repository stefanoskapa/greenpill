#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define N16 n16 = rom[i + 1] | (rom[i + 2] << 8); i += 2
#define N8  n8 = rom[i + 1]; i++
#define E8  e8 = (int8_t)rom[i + 1]; i++

void disasm_cb(uint8_t op) {
    const char *regs[] = {"B", "C", "D", "E", "H", "L", "(HL)", "A"};
    uint8_t r = op & 0x07;
    uint8_t bit = (op >> 3) & 0x07;
    
    if (op < 0x08)      printf("RLC\t%s\n", regs[r]);
    else if (op < 0x10) printf("RRC\t%s\n", regs[r]);
    else if (op < 0x18) printf("RL\t%s\n", regs[r]);
    else if (op < 0x20) printf("RR\t%s\n", regs[r]);
    else if (op < 0x28) printf("SLA\t%s\n", regs[r]);
    else if (op < 0x30) printf("SRA\t%s\n", regs[r]);
    else if (op < 0x38) printf("SWAP\t%s\n", regs[r]);
    else if (op < 0x40) printf("SRL\t%s\n", regs[r]);
    else if (op < 0x80) printf("BIT\t%d, %s\n", bit, regs[r]);
    else if (op < 0xC0) printf("RES\t%d, %s\n", bit, regs[r]);
    else                printf("SET\t%d, %s\n", bit, regs[r]);
}

int main(int argc, char **argv) {

    if (argc < 2) {
        fprintf(stderr, "No rom file was provided\n");
        exit(1);
    }

    uint8_t rom[2000000];

    uint8_t logo[] = {
        0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D,
        0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99,
        0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E
    };

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
    bool logo_found = memcmp(logo, rom + 0x103, 48);
    printf("Nintendo logo found: %s\n", logo_found ? "YES" : "NO");

    uint16_t n16 = 0;
    uint8_t n8 = 0;
    int8_t e8 = 0;
    
    uint8_t regB = 0;
    uint8_t regC = 0;
    
    uint8_t regD = 0;
    uint8_t regE = 0;

    uint8_t regH = 0;
    uint8_t regL = 0;

    uint16_t regSP = 0;
    uint16_t PC = 0x0100;

    uint16_t a16 = 0;
    while (true) {
        uint8_t op = rom[PC];
        printf("0x%04X: ", PC);

        switch (op) {
            // 0x00 - 0x0F
            
            // NOP
            // cycles: 4
            // bytes: 1
            case 0x00: 
                printf("NOP\n");
                PC += 1; 
                break;
            // JP <a16>
            // cycles: 16
            // bytes: 3
            case 0xC3:
                a16 = rom[PC + 1] | (rom[PC + 2] << 8); 
                printf("JP 0x%04X\n", a16);
                PC = a16; 
                break; 
            default:
                printf("<UNKNOWN 0x%02X>\n", op);
                exit(1);
                break;
        }
    }

    return EXIT_SUCCESS;
}

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

    for (int i = 0x0150; i < size; i++) {
        uint8_t op = rom[i];
        printf("0x%04X: ", i);

        switch (op) {
            // 0x00 - 0x0F
            case 0x00: printf("NOP\n"); break;
            case 0x01: N16; printf("LD\tBC, 0x%04X\n", n16); break;
            case 0x02: printf("LD\t(BC), A\n"); break;
            case 0x03: printf("INC\tBC\n"); break;
            case 0x04: printf("INC\tB\n"); break;
            case 0x05: printf("DEC\tB\n"); break;
            case 0x06: N8; printf("LD\tB, 0x%02X\n", n8); break;
            case 0x07: printf("RLCA\n"); break;
            case 0x08: N16; printf("LD\t(0x%04X), SP\n", n16); break;
            case 0x09: printf("ADD\tHL, BC\n"); break;
            case 0x0A: printf("LD\tA, (BC)\n"); break;
            case 0x0B: printf("DEC\tBC\n"); break;
            case 0x0C: printf("INC\tC\n"); break;
            case 0x0D: printf("DEC\tC\n"); break;
            case 0x0E: N8; printf("LD\tC, 0x%02X\n", n8); break;
            case 0x0F: printf("RRCA\n"); break;

            // 0x10 - 0x1F
            case 0x10: N8; printf("STOP\n"); break;
            case 0x11: N16; printf("LD\tDE, 0x%04X\n", n16); break;
            case 0x12: printf("LD\t(DE), A\n"); break;
            case 0x13: printf("INC\tDE\n"); break;
            case 0x14: printf("INC\tD\n"); break;
            case 0x15: printf("DEC\tD\n"); break;
            case 0x16: N8; printf("LD\tD, 0x%02X\n", n8); break;
            case 0x17: printf("RLA\n"); break;
            case 0x18: E8; printf("JR\t%+d\n", e8); break;
            case 0x19: printf("ADD\tHL, DE\n"); break;
            case 0x1A: printf("LD\tA, (DE)\n"); break;
            case 0x1B: printf("DEC\tDE\n"); break;
            case 0x1C: printf("INC\tE\n"); break;
            case 0x1D: printf("DEC\tE\n"); break;
            case 0x1E: N8; printf("LD\tE, 0x%02X\n", n8); break;
            case 0x1F: printf("RRA\n"); break;

            // 0x20 - 0x2F
            case 0x20: E8; printf("JR\tNZ, %+d\n", e8); break;
            case 0x21: N16; printf("LD\tHL, 0x%04X\n", n16); break;
            case 0x22: printf("LD\t(HL+), A\n"); break;
            case 0x23: printf("INC\tHL\n"); break;
            case 0x24: printf("INC\tH\n"); break;
            case 0x25: printf("DEC\tH\n"); break;
            case 0x26: N8; printf("LD\tH, 0x%02X\n", n8); break;
            case 0x27: printf("DAA\n"); break;
            case 0x28: E8; printf("JR\tZ, %+d\n", e8); break;
            case 0x29: printf("ADD\tHL, HL\n"); break;
            case 0x2A: printf("LD\tA, (HL+)\n"); break;
            case 0x2B: printf("DEC\tHL\n"); break;
            case 0x2C: printf("INC\tL\n"); break;
            case 0x2D: printf("DEC\tL\n"); break;
            case 0x2E: N8; printf("LD\tL, 0x%02X\n", n8); break;
            case 0x2F: printf("CPL\n"); break;

            // 0x30 - 0x3F
            case 0x30: E8; printf("JR\tNC, %+d\n", e8); break;
            case 0x31: N16; printf("LD\tSP, 0x%04X\n", n16); break;
            case 0x32: printf("LD\t(HL-), A\n"); break;
            case 0x33: printf("INC\tSP\n"); break;
            case 0x34: printf("INC\t(HL)\n"); break;
            case 0x35: printf("DEC\t(HL)\n"); break;
            case 0x36: N8; printf("LD\t(HL), 0x%02X\n", n8); break;
            case 0x37: printf("SCF\n"); break;
            case 0x38: E8; printf("JR\tC, %+d\n", e8); break;
            case 0x39: printf("ADD\tHL, SP\n"); break;
            case 0x3A: printf("LD\tA, (HL-)\n"); break;
            case 0x3B: printf("DEC\tSP\n"); break;
            case 0x3C: printf("INC\tA\n"); break;
            case 0x3D: printf("DEC\tA\n"); break;
            case 0x3E: N8; printf("LD\tA, 0x%02X\n", n8); break;
            case 0x3F: printf("CCF\n"); break;

            // 0x40 - 0x4F: LD B/C, r
            case 0x40: printf("LD\tB, B\n"); break;
            case 0x41: printf("LD\tB, C\n"); break;
            case 0x42: printf("LD\tB, D\n"); break;
            case 0x43: printf("LD\tB, E\n"); break;
            case 0x44: printf("LD\tB, H\n"); break;
            case 0x45: printf("LD\tB, L\n"); break;
            case 0x46: printf("LD\tB, (HL)\n"); break;
            case 0x47: printf("LD\tB, A\n"); break;
            case 0x48: printf("LD\tC, B\n"); break;
            case 0x49: printf("LD\tC, C\n"); break;
            case 0x4A: printf("LD\tC, D\n"); break;
            case 0x4B: printf("LD\tC, E\n"); break;
            case 0x4C: printf("LD\tC, H\n"); break;
            case 0x4D: printf("LD\tC, L\n"); break;
            case 0x4E: printf("LD\tC, (HL)\n"); break;
            case 0x4F: printf("LD\tC, A\n"); break;

            // 0x50 - 0x5F: LD D/E, r
            case 0x50: printf("LD\tD, B\n"); break;
            case 0x51: printf("LD\tD, C\n"); break;
            case 0x52: printf("LD\tD, D\n"); break;
            case 0x53: printf("LD\tD, E\n"); break;
            case 0x54: printf("LD\tD, H\n"); break;
            case 0x55: printf("LD\tD, L\n"); break;
            case 0x56: printf("LD\tD, (HL)\n"); break;
            case 0x57: printf("LD\tD, A\n"); break;
            case 0x58: printf("LD\tE, B\n"); break;
            case 0x59: printf("LD\tE, C\n"); break;
            case 0x5A: printf("LD\tE, D\n"); break;
            case 0x5B: printf("LD\tE, E\n"); break;
            case 0x5C: printf("LD\tE, H\n"); break;
            case 0x5D: printf("LD\tE, L\n"); break;
            case 0x5E: printf("LD\tE, (HL)\n"); break;
            case 0x5F: printf("LD\tE, A\n"); break;

            // 0x60 - 0x6F: LD H/L, r
            case 0x60: printf("LD\tH, B\n"); break;
            case 0x61: printf("LD\tH, C\n"); break;
            case 0x62: printf("LD\tH, D\n"); break;
            case 0x63: printf("LD\tH, E\n"); break;
            case 0x64: printf("LD\tH, H\n"); break;
            case 0x65: printf("LD\tH, L\n"); break;
            case 0x66: printf("LD\tH, (HL)\n"); break;
            case 0x67: printf("LD\tH, A\n"); break;
            case 0x68: printf("LD\tL, B\n"); break;
            case 0x69: printf("LD\tL, C\n"); break;
            case 0x6A: printf("LD\tL, D\n"); break;
            case 0x6B: printf("LD\tL, E\n"); break;
            case 0x6C: printf("LD\tL, H\n"); break;
            case 0x6D: printf("LD\tL, L\n"); break;
            case 0x6E: printf("LD\tL, (HL)\n"); break;
            case 0x6F: printf("LD\tL, A\n"); break;

            // 0x70 - 0x7F: LD (HL)/A, r
            case 0x70: printf("LD\t(HL), B\n"); break;
            case 0x71: printf("LD\t(HL), C\n"); break;
            case 0x72: printf("LD\t(HL), D\n"); break;
            case 0x73: printf("LD\t(HL), E\n"); break;
            case 0x74: printf("LD\t(HL), H\n"); break;
            case 0x75: printf("LD\t(HL), L\n"); break;
            case 0x76: printf("HALT\n"); break;
            case 0x77: printf("LD\t(HL), A\n"); break;
            case 0x78: printf("LD\tA, B\n"); break;
            case 0x79: printf("LD\tA, C\n"); break;
            case 0x7A: printf("LD\tA, D\n"); break;
            case 0x7B: printf("LD\tA, E\n"); break;
            case 0x7C: printf("LD\tA, H\n"); break;
            case 0x7D: printf("LD\tA, L\n"); break;
            case 0x7E: printf("LD\tA, (HL)\n"); break;
            case 0x7F: printf("LD\tA, A\n"); break;

            // 0x80 - 0x8F: ADD/ADC A, r
            case 0x80: printf("ADD\tA, B\n"); break;
            case 0x81: printf("ADD\tA, C\n"); break;
            case 0x82: printf("ADD\tA, D\n"); break;
            case 0x83: printf("ADD\tA, E\n"); break;
            case 0x84: printf("ADD\tA, H\n"); break;
            case 0x85: printf("ADD\tA, L\n"); break;
            case 0x86: printf("ADD\tA, (HL)\n"); break;
            case 0x87: printf("ADD\tA, A\n"); break;
            case 0x88: printf("ADC\tA, B\n"); break;
            case 0x89: printf("ADC\tA, C\n"); break;
            case 0x8A: printf("ADC\tA, D\n"); break;
            case 0x8B: printf("ADC\tA, E\n"); break;
            case 0x8C: printf("ADC\tA, H\n"); break;
            case 0x8D: printf("ADC\tA, L\n"); break;
            case 0x8E: printf("ADC\tA, (HL)\n"); break;
            case 0x8F: printf("ADC\tA, A\n"); break;

            // 0x90 - 0x9F: SUB/SBC A, r
            case 0x90: printf("SUB\tA, B\n"); break;
            case 0x91: printf("SUB\tA, C\n"); break;
            case 0x92: printf("SUB\tA, D\n"); break;
            case 0x93: printf("SUB\tA, E\n"); break;
            case 0x94: printf("SUB\tA, H\n"); break;
            case 0x95: printf("SUB\tA, L\n"); break;
            case 0x96: printf("SUB\tA, (HL)\n"); break;
            case 0x97: printf("SUB\tA, A\n"); break;
            case 0x98: printf("SBC\tA, B\n"); break;
            case 0x99: printf("SBC\tA, C\n"); break;
            case 0x9A: printf("SBC\tA, D\n"); break;
            case 0x9B: printf("SBC\tA, E\n"); break;
            case 0x9C: printf("SBC\tA, H\n"); break;
            case 0x9D: printf("SBC\tA, L\n"); break;
            case 0x9E: printf("SBC\tA, (HL)\n"); break;
            case 0x9F: printf("SBC\tA, A\n"); break;

            // 0xA0 - 0xAF: AND/XOR A, r
            case 0xA0: printf("AND\tA, B\n"); break;
            case 0xA1: printf("AND\tA, C\n"); break;
            case 0xA2: printf("AND\tA, D\n"); break;
            case 0xA3: printf("AND\tA, E\n"); break;
            case 0xA4: printf("AND\tA, H\n"); break;
            case 0xA5: printf("AND\tA, L\n"); break;
            case 0xA6: printf("AND\tA, (HL)\n"); break;
            case 0xA7: printf("AND\tA, A\n"); break;
            case 0xA8: printf("XOR\tA, B\n"); break;
            case 0xA9: printf("XOR\tA, C\n"); break;
            case 0xAA: printf("XOR\tA, D\n"); break;
            case 0xAB: printf("XOR\tA, E\n"); break;
            case 0xAC: printf("XOR\tA, H\n"); break;
            case 0xAD: printf("XOR\tA, L\n"); break;
            case 0xAE: printf("XOR\tA, (HL)\n"); break;
            case 0xAF: printf("XOR\tA, A\n"); break;

            // 0xB0 - 0xBF: OR/CP A, r
            case 0xB0: printf("OR\tA, B\n"); break;
            case 0xB1: printf("OR\tA, C\n"); break;
            case 0xB2: printf("OR\tA, D\n"); break;
            case 0xB3: printf("OR\tA, E\n"); break;
            case 0xB4: printf("OR\tA, H\n"); break;
            case 0xB5: printf("OR\tA, L\n"); break;
            case 0xB6: printf("OR\tA, (HL)\n"); break;
            case 0xB7: printf("OR\tA, A\n"); break;
            case 0xB8: printf("CP\tA, B\n"); break;
            case 0xB9: printf("CP\tA, C\n"); break;
            case 0xBA: printf("CP\tA, D\n"); break;
            case 0xBB: printf("CP\tA, E\n"); break;
            case 0xBC: printf("CP\tA, H\n"); break;
            case 0xBD: printf("CP\tA, L\n"); break;
            case 0xBE: printf("CP\tA, (HL)\n"); break;
            case 0xBF: printf("CP\tA, A\n"); break;

            // 0xC0 - 0xCF
            case 0xC0: printf("RET\tNZ\n"); break;
            case 0xC1: printf("POP\tBC\n"); break;
            case 0xC2: N16; printf("JP\tNZ, 0x%04X\n", n16); break;
            case 0xC3: N16; printf("JP\t0x%04X\n", n16); break;
            case 0xC4: N16; printf("CALL\tNZ, 0x%04X\n", n16); break;
            case 0xC5: printf("PUSH\tBC\n"); break;
            case 0xC6: N8; printf("ADD\tA, 0x%02X\n", n8); break;
            case 0xC7: printf("RST\t0x00\n"); break;
            case 0xC8: printf("RET\tZ\n"); break;
            case 0xC9: printf("RET\n"); break;
            case 0xCA: N16; printf("JP\tZ, 0x%04X\n", n16); break;
            case 0xCB: i++; printf("CB "); disasm_cb(rom[i]); break;
            case 0xCC: N16; printf("CALL\tZ, 0x%04X\n", n16); break;
            case 0xCD: N16; printf("CALL\t0x%04X\n", n16); break;
            case 0xCE: N8; printf("ADC\tA, 0x%02X\n", n8); break;
            case 0xCF: printf("RST\t0x08\n"); break;

            // 0xD0 - 0xDF
            case 0xD0: printf("RET\tNC\n"); break;
            case 0xD1: printf("POP\tDE\n"); break;
            case 0xD2: N16; printf("JP\tNC, 0x%04X\n", n16); break;
            case 0xD3: printf("ILLEGAL\t0xD3\n"); break;
            case 0xD4: N16; printf("CALL\tNC, 0x%04X\n", n16); break;
            case 0xD5: printf("PUSH\tDE\n"); break;
            case 0xD6: N8; printf("SUB\tA, 0x%02X\n", n8); break;
            case 0xD7: printf("RST\t0x10\n"); break;
            case 0xD8: printf("RET\tC\n"); break;
            case 0xD9: printf("RETI\n"); break;
            case 0xDA: N16; printf("JP\tC, 0x%04X\n", n16); break;
            case 0xDB: printf("ILLEGAL\t0xDB\n"); break;
            case 0xDC: N16; printf("CALL\tC, 0x%04X\n", n16); break;
            case 0xDD: printf("ILLEGAL\t0xDD\n"); break;
            case 0xDE: N8; printf("SBC\tA, 0x%02X\n", n8); break;
            case 0xDF: printf("RST\t0x18\n"); break;

            // 0xE0 - 0xEF
            case 0xE0: N8; printf("LDH\t(0xFF%02X), A\n", n8); break;
            case 0xE1: printf("POP\tHL\n"); break;
            case 0xE2: printf("LD\t(0xFF00+C), A\n"); break;
            case 0xE3: printf("ILLEGAL\t0xE3\n"); break;
            case 0xE4: printf("ILLEGAL\t0xE4\n"); break;
            case 0xE5: printf("PUSH\tHL\n"); break;
            case 0xE6: N8; printf("AND\tA, 0x%02X\n", n8); break;
            case 0xE7: printf("RST\t0x20\n"); break;
            case 0xE8: E8; printf("ADD\tSP, %+d\n", e8); break;
            case 0xE9: printf("JP\tHL\n"); break;
            case 0xEA: N16; printf("LD\t(0x%04X), A\n", n16); break;
            case 0xEB: printf("ILLEGAL\t0xEB\n"); break;
            case 0xEC: printf("ILLEGAL\t0xEC\n"); break;
            case 0xED: printf("ILLEGAL\t0xED\n"); break;
            case 0xEE: N8; printf("XOR\tA, 0x%02X\n", n8); break;
            case 0xEF: printf("RST\t0x28\n"); break;

            // 0xF0 - 0xFF
            case 0xF0: N8; printf("LDH\tA, (0xFF%02X)\n", n8); break;
            case 0xF1: printf("POP\tAF\n"); break;
            case 0xF2: printf("LD\tA, (0xFF00+C)\n"); break;
            case 0xF3: printf("DI\n"); break;
            case 0xF4: printf("ILLEGAL\t0xF4\n"); break;
            case 0xF5: printf("PUSH\tAF\n"); break;
            case 0xF6: N8; printf("OR\tA, 0x%02X\n", n8); break;
            case 0xF7: printf("RST\t0x30\n"); break;
            case 0xF8: E8; printf("LD\tHL, SP%+d\n", e8); break;
            case 0xF9: printf("LD\tSP, HL\n"); break;
            case 0xFA: N16; printf("LD\tA, (0x%04X)\n", n16); break;
            case 0xFB: printf("EI\n"); break;
            case 0xFC: printf("ILLEGAL\t0xFC\n"); break;
            case 0xFD: printf("ILLEGAL\t0xFD\n"); break;
            case 0xFE: N8; printf("CP\tA, 0x%02X\n", n8); break;
            case 0xFF: printf("RST\t0x38\n"); break;

            default:
                printf("<UNKNOWN 0x%02X>\n", op);
                break;
        }
    }

    return EXIT_SUCCESS;
}

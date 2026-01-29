#include <SDL2/SDL.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "ppu.h"

extern uint8_t mem[];
extern bool debug;
extern uint8_t *IF;
struct sprite {
    uint8_t y, x, tile, flags;
};

/*
 * LCDC: LCD control (0xFF40)
 * - 0: BG & Window enable / priority
 * - 1: OBJ enable
 * - 2: OBJ size
 * - 3: BG tile map
 * - 4: BG & Window tiles
 * - 5: Window enable
 * - 6: Window tile map
 * - 7: LCD & PPU enable
 */
uint8_t *LCDC = &mem[0xFF40];
/*
 * LY: LCD Y coordinate [read-only] (0xFF44)
 */
uint8_t *LY = &mem[0xFF44];
/*
 * STAT: LCD status (0xFF41)
 *
 * - 0-1: PPU mode
 * - 2: LYC == LY
 * - 3: Mode 0 int select
 * - 4: Mode 1 int select
 * - 5: Mode 2 int select
 * - 6: LYC int select
 * - 7: 
 */
uint8_t *STAT = &mem[0xFF41];
/*
 * LYC: LY compare (0xFF45)
 */
uint8_t *LYC = &mem[0xFF45];
/*
 * SCY: Background viewport Y position (0xFF42)
 */
uint8_t *SCY = &mem[0xFF42];
/*
 * SCX: Background viewport X position (0xFF43)
 */
uint8_t *SCX = &mem[0xFF43];


static int dots = 0;
bool sdl_render = true;
uint32_t framebuffer[160 * 144];
uint16_t bg_tilemap_addr;

SDL_Window *window;
SDL_Renderer *renderer;
SDL_Texture *texture;

uint32_t palette[4] = {
    0xFF9A9E3F,  // Lightest (color 0)
    0xFF496B22,  // Light (color 1)
    0xFF0E450B,  // Dark (color 2)
    0xFF1B2A09   // Darkest (color 3)
};

static void show_frame();
static void render_tile_row(uint8_t byte1, uint8_t byte2, int x, bool is_obj, uint8_t flags);
static void ppu_step();

void ppu_init(void) {

    mem[0xFF40] = 0x91;  // LCDC - LCD ON!
    mem[0xFF47] = 0xFC;  // BGP palette
    mem[0xFF44] = 0;  // LY starts at 0
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

    if ((*LCDC & 0b00001000) == 0) {
        bg_tilemap_addr = 0x9800;
    } else {
        bg_tilemap_addr = 0x9C00;
    }

}

void ppu_step() {

    if (*LYC == *LY) {
        *STAT |= 0b00000100; // set LYC=LY flag
        if ((*STAT & 0b01000000) != 0) {
            *IF |= 0b00000010;  // Request STAT interrupt
        }
    }

    dots++; 

    static struct sprite intersecting_sprites[10];
    static int sprites_per_line = 0;
    int height = (*LCDC & 0b00000100) ? 16 : 8; //LCDC bit 2 = 0 -> 8x8

    if (dots > 0 && dots <= 80) { // mode 2 - OAM scan, duration: 80 dots
        *STAT = (*STAT & 0b11111100) | 0b10; // report Mode 2 to STAT
        if (dots == 1) { // do all work on dot 1, then wait for the remaining dots
            sprites_per_line = 0;

            for (int i = 0; i < 40; i++) {
                if (sprites_per_line == 10) {
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
                //int screen_y = object.y - height * 2;
                int screen_y = object.y - 16;
                if (*LY >= screen_y && *LY < screen_y + height) {
                    intersecting_sprites[sprites_per_line++] = object;
                }
            }
        }
    } else if (dots > 80 && dots <= 456) { // mode 3 - Drawing Pixels duration: between 172 and 289 dots
        *STAT = (*STAT & 0b11111100) | 0b11;  // report Mode 3 to STAT
        if (dots == 81 && *LY < 144) {

            int y = (*SCY + *LY) & 0xFF;

            int tile_row = y / 8;
            int row_in_tile = y % 8; 

            int fine_x = *SCX & 7;  // pixels to skip in first tile

            for (int i = 0; i < 21; i++) {  // Need 21 tiles to cover screen when scrolling
                int map_x = (*SCX + i * 8) & 0xFF;  

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

                render_tile_row(bgbyte1, bgbyte2, i * 8 - fine_x, false, 0);
            }

            // render sprites
            for (int i = sprites_per_line - 1; i >=0; i--) {
                struct sprite object = intersecting_sprites[i];
                bool y_flip = ((object.flags & 0b01000000) != 0);
                int row_in_sprite = *LY - (object.y - 16);
                if (y_flip) {
                    row_in_sprite = 7 - row_in_sprite;
                }
                if (height == 8) { 
                    uint8_t byte1 = mem[0x8000 + object.tile * 16 + row_in_sprite * 2];
                    uint8_t byte2 = mem[0x8000 + object.tile * 16 + row_in_sprite * 2 + 1];
                    render_tile_row(byte1, byte2, object.x - 8, true, object.flags);               
                } else {
                    printf("Tall sprites detected!\n");
                    exit(1);
                }
            }
        }

        if (dots > 80 + 172) { //mode 0 - Horizontal Blank, duration: 376 - mode 3’s duration
            *STAT = (*STAT & 0b11111100) | 0b00;  // report Mode 0 to STAT
        }
    }

    if (dots == 456) {
        dots = 0;
        (*LY)++;

        if (*LY == 144) { // mode 1 Vertical Blank (LY= [144,153]) duration: 4560 dots (10 scanlines)
            *STAT = (*STAT & 0b11111100) | 0b01;  // report Mode 1 to STAT
            *IF |= 0b00000001;  // Request VBlank interrupt
            show_frame();
        } else if (*LY > 153) {
            *LY = 0;
        }
    }
}

void ppu_steps(int cycles) {
    for (int i = 0; i < cycles; i++) {
        ppu_step();
    }
}
void render_tile_row(uint8_t byte1, uint8_t byte2, int x, bool is_obj, uint8_t flags) {

    bool priority = flags & 0b10000000;
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
static void show_frame() {
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


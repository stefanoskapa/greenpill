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

// PPU registers
uint8_t *LCDC = &mem[0xFF40];
uint8_t *LY = &mem[0xFF44];
uint8_t *STAT = &mem[0xFF41];
uint8_t *SCY = &mem[0xFF42];
uint8_t *SCX = &mem[0xFF43];
static int dots = 0;

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

static void show_frame();

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
void ppu_steps(int cycles) {
    for (int i = 0; i < cycles; i++) {
        ppu_step();
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


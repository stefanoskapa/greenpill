#include <stdint.h>
#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdio.h>

#define CYCLES_PER_SAMPLE (4194304.0f / 44100.0f)

#define SAMPLE_RATE 48000
#define AMPLITUDE   28000

extern uint8_t mem[];

extern bool apu_debug;

/*
 * NR51 (0xFF25) Sound Panning
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
 * NR52 (0xFF26) Audio master control
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

static bool channel1_playing = false;
static uint8_t channel1_volume = 0;
static float channel1_phase = 0;
static float channel1_phase_increment = 0;
static uint8_t channel1_length = 0;

static bool channel2_playing = false;
static uint8_t channel2_volume = 0;
static float channel2_phase = 0;
static float channel2_phase_increment = 0;
static uint8_t channel2_length = 0;

SDL_AudioDeviceID audio_device;
SDL_AudioSpec audio_spec;

float cycles_per_sample_counter = 0;

void apu_init(void) {
    SDL_Init(SDL_INIT_AUDIO);

    SDL_zero(audio_spec);
    audio_spec.freq = 44100;
    audio_spec.format = AUDIO_S16SYS;
    audio_spec.channels = 1;
    audio_spec.samples = 1024;
    audio_spec.callback = NULL;

    audio_device = SDL_OpenAudioDevice(NULL, 0, &audio_spec, NULL, 0);
    SDL_PauseAudioDevice(audio_device, 0);
}

void apu_step(int cycles) {
    cycles_per_sample_counter += cycles;
    if (cycles_per_sample_counter >= CYCLES_PER_SAMPLE) {
        cycles_per_sample_counter -= CYCLES_PER_SAMPLE;
        int16_t vol_steps = 100;
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

void chan1debug(uint8_t nr14) {

    puts("---Channel 1 debug---");
    if ((*NR52 & 0b10000000) == 0) {
        puts("APU is OFF");
    } else {
        puts("APU is ON");
    }

    if ((*NR12 & 0xF8) == 0) {
        puts("Channel 1 DAC is OFF");
    } else {
        puts("Channel 1 DAC is ON");
    }

    if (nr14 &0b1000000) {
        puts("Channel 1 ON");
    }

    //NR11
    uint8_t initial_length = *NR11 & 0b00111111;
    uint8_t wave_duty = (*NR11 & 0b11000000) >> 6;
    printf("NR11: wave_duty=%d, initial_length=%d\n",wave_duty, initial_length); 

    //NR12
    uint8_t initial_volume = (*NR12 & 0b11110000) >> 4;
    uint8_t env_dir = (*NR12 & 0b00001000) >> 3;
    uint8_t sweep_pace = (*NR12 & 0b00000111);
    printf("NR12: initial_volume=%d, envelope_direction=%d, sweep_pace=%d\n", initial_volume, env_dir, sweep_pace);

    //NR24
    uint8_t trigger = (nr14 & 0b10000000) >> 7;
    uint8_t length_enable = (nr14 & 0b01000000) >> 6;
    printf("NR14: trigger=%d, length_enable=%d\n", trigger, length_enable);
}

void chan2debug(uint8_t nr24) {

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

void audio_delay(void){
    while (SDL_GetQueuedAudioSize(audio_device) > 4000 ) {
        SDL_Delay(1);
    }
}

void apu_memw_callback(uint16_t addr, uint8_t b) {


    switch (addr) {
        case 0xFF26: //NR52
            if ((b & 0b10000000) == 0) {
                channel1_playing = false;
                channel2_playing = false;
                if (apu_debug) puts("AUDIO OFF");
            } else {
                if (apu_debug) puts("AUDIO ON");
            }
            break;
        case 0xFF12: //NR12
            if ((b & 0b11111000) == 0) { // DAC is turned off
               channel1_playing = false;
               if (apu_debug) puts("Ch1: DAC turned OFF");
            } else {
                uint8_t init_vol = (b & 0b11110000) >> 4;
                uint8_t sw_pace = b & 0b00000111;
                char *env_dir = (b & 0b00001000) == 0 ? "down" : "up";
                if (apu_debug) printf("Ch1: init_vol=%u, env_dir=%s, sw_pace=%u }\n", init_vol, env_dir, sw_pace);
                channel1_volume = init_vol;
                //TODO sweep pace, envelope direction
            }
            break;
        case 0xFF14: // NR14
             if ((b & 0b10000000) != 0) {
                channel1_volume = (*NR12 & 0b11110000) >> 4;
                channel1_playing = true;
                uint16_t period = *NR13; 
                uint16_t period_high = b & 0b00000111;
                period_high = period_high << 8;
                period |= period_high;
                double frequency = (double)131072 / (2048 - period);
                channel1_phase_increment = frequency / 44100.0f;
                if (apu_debug) {
                    puts("Ch1: triggered");
                    printf("Ch1: initial volume = %u\n", channel1_volume);
                    printf("Ch1: period = %u (%f Hz)\n", period, frequency);
                }
             }
             break;
        case 0xFF17: // NR22
            if ((b & 0xF8) == 0) { 
               channel2_playing = false;
               if (apu_debug) puts("Ch2: DAC turned OFF");
            } else {
                uint8_t init_vol = (b & 0b11110000) >> 4;
                uint8_t sw_pace = b & 0b00000111;
                char *env_dir = (b & 0b00001000) == 0 ? "down" : "up";
                if (apu_debug) printf("Ch2: init_vol=%u, env_dir=%s, sw_pace=%u }\n", init_vol, env_dir, sw_pace);
                channel2_volume = init_vol;
                //TODO sweep pace, envelope direction
            }
            break;
        case 0xFF19: // NR24
             if ((b & 0b10000000) != 0) {
                channel2_volume = (*NR22 & 0b11110000) >> 4;
                channel2_playing = true;
                uint16_t period = *NR23; 
                uint16_t period_high = b & 0b00000111;
                period_high = period_high << 8;
                period |= period_high;
                double frequency = (double)131072 / (2048 - period);
                channel2_phase_increment = frequency / 44100.0f;
                if (apu_debug) {
                    puts("Ch2: triggered");
                    printf("Ch2: initial volume = %u\n", channel2_volume);
                    printf("Ch2: period = %u (%f Hz)\n", period, frequency);
                }
             }
             break;
    }

}

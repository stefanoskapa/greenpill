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

uint8_t *NR10 = &mem[0xFF10];
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
static bool channel1_length_enabled = false;
static uint8_t channel1_sweep_pace = 0;
static uint8_t channel1_envelope_pace = 0;
static uint8_t channel1_envelope_counter = 0;
static uint8_t channel1_envelope_dir = 0;

static bool channel2_playing = false;
static uint8_t channel2_volume = 0;
static float channel2_phase = 0;
static float channel2_phase_increment = 0;
static uint8_t channel2_length = 0;
static bool channel2_length_enabled = false;
static uint8_t channel2_envelope_pace = 0;
static uint8_t channel2_envelope_counter = 0;
static uint8_t channel2_envelope_dir = 0;

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
        if (channel1_playing && !(channel1_length_enabled && channel1_length == 64)) {
            sample += (channel1_phase < 0.5f) ? channel1_volume * vol_steps : channel1_volume * (-vol_steps);
            channel1_phase += channel1_phase_increment;
            if (channel1_phase >= 1.0f) channel1_phase -= 1.0f;
        }

        //if (channel2_playing) {
        if (channel2_playing && !(channel2_length_enabled && channel2_length == 64)) {
            sample += (channel2_phase < 0.5f) ? channel2_volume * vol_steps : channel2_volume * (-vol_steps);
            channel2_phase += channel2_phase_increment;
            if (channel2_phase >= 1.0f) channel2_phase -= 1.0f;
        }

        SDL_QueueAudio(audio_device, &sample, sizeof(sample));
    }
}

void ch1_env_call() {
    if (channel1_envelope_pace == 0) return;
    channel1_envelope_counter++;
    if (channel1_envelope_counter == channel1_envelope_pace) {
       channel1_volume += (channel1_envelope_dir == 0 ? -1 : 1);
       if (channel1_volume < 0 || channel1_volume > 15) {
           channel1_volume = 0;
       }
       channel1_envelope_counter = 0;
    }
}
void ch2_env_call() {
    if (channel2_envelope_pace == 0) return;
    channel2_envelope_counter++;
    if (channel2_envelope_counter == channel2_envelope_pace) {
       channel2_volume += (channel2_envelope_dir == 0 ? -1 : 1);
       if (channel2_volume < 0 || channel2_volume > 15) {
           channel2_volume = 0;
       }
       channel2_envelope_counter = 0;
    }
}

void audio_delay(void){
    while (SDL_GetQueuedAudioSize(audio_device) > 4000 ) {
        SDL_Delay(1);
    }
}

//When the length timer reaches 64 (CH1, CH2, and CH4) or 256 (CH3), the channel is turned off.
void inc_ch2_len() {
    if (!channel2_length_enabled) return;
    if (channel2_length == 64) {
        channel2_playing = false;
    } else {
        channel2_length++;
    }
}

void inc_ch1_len() {
    if (!channel1_length_enabled) return;
    if (channel1_length == 64) {
        channel1_playing = false;
    } else {
        channel1_length++;
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
                channel1_length = *NR11 & 0b00111111;
                channel1_length_enabled = (b & 0b01000000) != 0;
                channel1_sweep_pace = (*NR10 & 0b01110000) >> 4;
                channel1_envelope_pace = (*NR12 & 0b00000111);
                channel1_envelope_dir = (*NR12 & 0b00001000) >> 4;
                if (apu_debug) {
                    puts("Ch1 triggered");
                    printf("\tinitial volume = %u\n", channel1_volume);
                    printf("\tperiod = %u (%f Hz)\n", period, frequency);
                    printf("\tlength = %u\n", channel1_length);
                    printf("\tlength enabled = %s\n", channel1_length_enabled ? "YES": "NO");
                    printf("\tsweep pace = %u\n", channel1_sweep_pace);
                    if (channel1_envelope_pace != 0) {
                        printf("\tEnvelope sweep pace = %u\n", channel1_envelope_pace);
                        printf("\tEnvelope direction = %s\n", channel1_envelope_dir == 0 ? "DOWN" : "UP");
                    }
                    
                }
             }
             break;
        case 0xFF17: // NR22
            if ((b & 0b11111000) == 0) { 
               channel2_playing = false;
               if (apu_debug) puts("Ch2: DAC turned OFF");
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
                channel2_length = *NR21 & 0b00111111;
                channel2_length_enabled = (b & 0b01000000) != 0;
                channel2_envelope_pace = (*NR22 & 0b00000111);
                channel2_envelope_dir = (*NR22 & 0b00001000) >> 4;
                if (apu_debug) {
                    puts("Ch2 triggered");
                    printf("\tinitial volume = %u\n", channel2_volume);
                    printf("\tperiod = %u (%f Hz)\n", period, frequency);
                    printf("\tlength = %u\n", channel2_length);
                    printf("\tlength enabled = %s\n", channel1_length_enabled ? "YES": "NO");
                    if (channel2_envelope_pace != 0) {
                        printf("\tEnvelope sweep pace = %u\n", channel2_envelope_pace);
                        printf("\tEnvelope direction = %s\n", channel2_envelope_dir == 0 ? "DOWN" : "UP");
                    }
                }
             }
             break;
    }

}

// ESP32-S3 - 6-VOICE POLYPHONIC SYNTH - (V34.1-ExtendedWaves - Original V34.0 + 35 New Waves)

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <driver/i2s_std.h>
#include <MIDI.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -----------------------------------------------------------------------------
// KONSTANTEN & DEFINITIONEN
// -----------------------------------------------------------------------------
#define SAMPLE_RATE         441000 // Wie in deinem Code
#define NUM_CHANNELS        2
#define I2S_PORT_NUM        I2S_NUM_0
#define I2S_BCLK_PIN        (GPIO_NUM_27)
#define I2S_LRCK_PIN        (GPIO_NUM_26)
#define I2S_DOUT_PIN        (GPIO_NUM_25)
#define I2S_MCLK_PIN        (GPIO_NUM_NC)
#define DMA_BUFFER_COUNT    8
#define DMA_BUFFER_FRAMES   256
const int BYTES_PER_FRAME_STEREO = NUM_CHANNELS * (I2S_DATA_BIT_WIDTH_16BIT / 8);
const int I2S_WRITE_BUFFER_SIZE_BYTES = DMA_BUFFER_FRAMES * BYTES_PER_FRAME_STEREO;
const float MAX_SYSTEM_AMPLITUDE_FLOAT = 32760.0f;
#define MIDI_RX_PIN         GPIO_NUM_3
#define MIDI_TX_PIN_FOR_SERIAL2 GPIO_NUM_NC
#define MIDI_BAUD_RATE      31250
#define MIDI_LISTEN_CHANNEL MIDI_CHANNEL_OMNI

// --- Potis --- (unverändert)
#define POT_ATTACK_PIN      GPIO_NUM_33
#define POT_DECAY_PIN       GPIO_NUM_32
#define POT_SUSTAIN_PIN     GPIO_NUM_35
#define POT_RELEASE_PIN     GPIO_NUM_34
#define POT_FILTER_CUTOFF_PIN GPIO_NUM_13
#define POT_FILTER_RESO_PIN   GPIO_NUM_14
#define POT_LFO_RATE_PIN      GPIO_NUM_15
#define POT_LFO_PITCH_DEPTH_PIN GPIO_NUM_4
#define POT_LFO_FILTER_DEPTH_PIN GPIO_NUM_2
#define POT_MASTER_VOLUME_PIN   GPIO_NUM_12
#define POT_PITCHBEND_PIN       GPIO_NUM_36
#define POT_MOD_WHEEL_PIN       GPIO_NUM_39

// --- Taster --- (unverändert)
#define WAVE_SWITCH_PIN      GPIO_NUM_19
#define ARP_ONOFF_SWITCH_PIN GPIO_NUM_18
#define ARP_MODE_SWITCH_PIN  GPIO_NUM_17

// --- OLED Display --- (unverändert)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_I2C_ADDRESS 0x3C

// --- Synthesizer Allgemein ---
#define NUM_VOICES_ACTIVE   3  // Wie in deinem Code
#define WAVETABLE_SIZE      256 // Wie in deinem Code

const float POLYPHONY_DAMPING_FACTOR = 4.0f; // Wie in deinem Code
const float PRE_FILTER_DRIVE_ATTENUATION = 0.5f; // Wie in deinem Code

// Originale Wellenform-defines
#define WAVE_SINE           0
#define WAVE_SAW_DOWN       1
#define WAVE_SQUARE         2
#define WAVE_TRIANGLE       3
#define WAVE_PULSE_25       4
#define WAVE_PULSE_12       5
#define WAVE_SYNC_SAW       6
#define WAVE_FORMANT_A_ORG  7 // Umbenannt, da "Formant A" auch in neuen Waves vorkommt
#define WAVE_RAMP_UP        8
#define WAVE_HOLLOW_SQ_ORG  9 // Umbenannt
#define WAVE_FORMANT_O_ORG 10 // Umbenannt

// Gesamtzahl der Wellenformen: 11 Original + 30 Neue + 5 Weiche = 46
#define NUM_WAVETABLES_TOTAL 46


// --- Arpeggiator --- (unverändert aus deinem Code)
enum ArpMode {
    ARP_UP, ARP_DOWN, ARP_UP_DOWN, ARP_RANDOM,
    ARP_OCT_UP, ARP_OCT_DOWN, ARP_CHORD, ARP_ORDER_PLAYED,
    ARP_UP_DOUBLE, ARP_DOWN_DOUBLE, ARP_UP_DOWN_DOUBLE,
    ARP_UP_HALF, ARP_DOWN_HALF, ARP_UP_DOWN_HALF,
    NUM_ARP_MODES_SELECTABLE
};
const char* arp_mode_names[NUM_ARP_MODES_SELECTABLE] = {
    "Up", "Down", "Up/Down", "Random",
    "Oct Up", "Oct Down", "Chord", "Played",
    "Up x2", "Down x2", "Up/Dn x2",
    "Up /2", "Down /2", "Up/Dn /2"
};
volatile ArpMode current_arp_mode = ARP_UP;
volatile bool arp_is_on = false;

#define ARP_MAX_NOTES 10
volatile uint8_t arp_held_notes[ARP_MAX_NOTES];
volatile uint8_t arp_notes_by_order[ARP_MAX_NOTES];
volatile int arp_num_held_notes = 0;
volatile int arp_num_played_order_notes = 0;
volatile int arp_current_note_index = 0;
volatile int arp_current_played_order_index = 0;
unsigned long last_arp_step_time = 0;
const unsigned long ARP_BASE_INTERVAL_MS = 150;
volatile unsigned long current_arp_interval_ms = ARP_BASE_INTERVAL_MS;
volatile uint8_t arp_last_played_note_value = 0;
volatile uint8_t arp_last_velocity = 0;
bool arp_up_down_direction_is_up = true;
int arp_octave_shift = 0;
const int ARP_MAX_OCTAVE_STEPS = 3;


enum EnvelopeStage { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };
struct VoiceState { // Unverändert aus deinem Code
  volatile bool note_on_trigger; volatile bool note_off_trigger;
  uint8_t note; float velocity_gain;
  bool is_playing;
  float phase_accumulator; float phase_increment;
  int current_wave_idx;
  EnvelopeStage env_stage; float env_level;
  float env_attack_rate; float env_decay_rate;
  float env_sustain_level_voice; float env_release_rate;
};

// -----------------------------------------------------------------------------
// GLOBALE VARIABLEN
// -----------------------------------------------------------------------------
i2s_chan_handle_t i2s_tx_chan;
int16_t i2s_audio_buffer[DMA_BUFFER_FRAMES * NUM_CHANNELS];
VoiceState voices[NUM_VOICES_ACTIVE];
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI);

// Array für ALLE Wellenformen
float wavetables[NUM_WAVETABLES_TOTAL][WAVETABLE_SIZE];
// Array für ALLE Wellenform-Namen
const char* wavetable_display_names[NUM_WAVETABLES_TOTAL];


unsigned long last_direct_pot_read_time = 0;
const unsigned long DIRECT_POT_READ_INTERVAL_MS = 20;
SemaphoreHandle_t voicesMutex = NULL;
TaskHandle_t audioMainTaskHandle = NULL;
TaskHandle_t controlMainTaskHandle = NULL;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
volatile bool display_update_needed = true;
volatile int global_osc_wave_select_idx = WAVE_SINE; // Startet mit der ersten (originalen) Sinuswelle
// Restliche globale Variablen unverändert aus deinem Code
volatile float global_pitchbend_factor = 1.0f;
volatile float global_attack_time_ms = 10.0f;
volatile float global_decay_time_ms = 50.0f;
volatile float global_sustain_level = 0.8f;
volatile float global_release_time_ms = 100.0f;
volatile float global_master_volume = 0.7f;
volatile float global_filter_cutoff_hz_target = 10000.0f;
volatile float smoothed_filter_cutoff_hz = 10000.0f;
const float CUTOFF_SMOOTHING_FACTOR = 0.05f;
volatile float global_filter_resonance_target = 0.707f;
volatile float smoothed_filter_resonance = 0.707f;
const float RESO_SMOOTHING_FACTOR = 0.08f;
volatile float global_lfo_rate_hz_target = 0.0f;
volatile float smoothed_lfo_rate_hz = 0.0f;
const float LFO_RATE_SMOOTHING_FACTOR = 0.1f;
volatile float global_lfo_pitch_depth_base_poti = 0.0f;
volatile float global_mod_wheel_value = 0.0f;
volatile float global_lfo_pitch_depth = 0.0f;
volatile float global_lfo_filter_depth = 0.0f;
float lfo_phase = 0.0f; float lfo_increment = 0.0f;
float global_filter_integrator1_state = 0.0f;
float global_filter_integrator2_state = 0.0f;
float global_filter_lp_output = 0.0f;
float global_filter_bp_output = 0.0f;
int raw_pot_attack, raw_pot_decay, raw_pot_sustain, raw_pot_release;
int raw_pot_filter_cutoff, raw_pot_filter_reso;
int raw_pot_lfo_rate, raw_pot_lfo_pitch_depth_poti_raw, raw_pot_lfo_filter_depth_raw;
int raw_pot_master_volume, raw_pot_mod_wheel, raw_pot_pitchbend;
const int LFO_PITCH_DEPTH_POTI_DEADZONE = 200;
const int LFO_RATE_POTI_DEADZONE = 200;
const int LFO_FILTER_DEPTH_POTI_DEADZONE = 200;
const int MOD_WHEEL_POTI_DEADZONE = 200;
const int PITCHBEND_POTI_DEADZONE_CENTER = 60;
const float MAX_PITCHBEND_SEMITONES = 2.0f;
const int MASTER_VOLUME_POTI_DEADZONE = 50;
unsigned long last_wave_switch_press_time = 0;
const unsigned long WAVE_SWITCH_DEBOUNCE_TIME = 200;
unsigned long last_arp_onoff_press_time = 0;
const unsigned long ARP_ONOFF_DEBOUNCE_TIME = 200;
unsigned long last_arp_mode_press_time = 0;
const unsigned long ARP_MODE_DEBOUNCE_TIME = 200;


// -----------------------------------------------------------------------------
// HILFSFUNKTIONEN
// -----------------------------------------------------------------------------
float midiNoteToFreq(uint8_t noteVal) { return 440.0f * powf(2.0f, (noteVal - 69.0f) / 12.0f); }
float interpolateWavetable(float table[WAVETABLE_SIZE], float index) { int i0 = (int)floorf(index); int i1 = (i0 + 1); float fraction = index - i0; i0 %= WAVETABLE_SIZE; if (i0 < 0) i0 += WAVETABLE_SIZE; i1 %= WAVETABLE_SIZE; if (i1 < 0) i1 += WAVETABLE_SIZE; return table[i0] * (1.0f - fraction) + table[i1] * fraction; }

void normalizeWavetable(int wave_idx) {
    if (wave_idx >= NUM_WAVETABLES_TOTAL) return; // Sicherheitscheck
    float max_abs_val = 0.0f;
    for (int i = 0; i < WAVETABLE_SIZE; ++i) {
        if (fabsf(wavetables[wave_idx][i]) > max_abs_val) {
            max_abs_val = fabsf(wavetables[wave_idx][i]);
        }
    }
    if (max_abs_val > 0.00001f) {
        for (int i = 0; i < WAVETABLE_SIZE; ++i) {
            wavetables[wave_idx][i] /= max_abs_val;
        }
    }
}

// --- ERWEITERTE WELLENFORMGENERIERUNG ---
void generateAllSynthWavetables() {
    for (int w = 0; w < NUM_WAVETABLES_TOTAL; ++w) {
        for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[w][i] = 0.0f;
        wavetable_display_names[w] = "N/A"; // Default für nicht zugewiesene
    }

    int current_wave_gen_idx = 0;

    // Originale 11 Wellenformen (Index 0-10)
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[current_wave_gen_idx][i] = sinf(2.0f * PI * (float)i / WAVETABLE_SIZE); wavetable_display_names[current_wave_gen_idx++] = "Sine"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[current_wave_gen_idx][i] = 1.0f - 2.0f * (float)i / WAVETABLE_SIZE; wavetable_display_names[current_wave_gen_idx++] = "Saw Down"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int k = 1; k <= 15; k += 2) for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[current_wave_gen_idx][i] += (1.0f/k)*sinf(2.0f*PI*k*(float)i/WAVETABLE_SIZE); normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Square"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float val = 2.0f*(float)i/WAVETABLE_SIZE; if(val > 1.0f) val=2.0f-val; wavetables[current_wave_gen_idx][i] = 2.0f*val-1.0f; } wavetable_display_names[current_wave_gen_idx++] = "Triangle"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { int pw=WAVETABLE_SIZE/4; for(int i=0;i<WAVETABLE_SIZE;++i) wavetables[current_wave_gen_idx][i]=(i<pw)?1.0f:-1.0f; wavetable_display_names[current_wave_gen_idx++] = "Pulse 25%"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { int pw=WAVETABLE_SIZE/8; for(int i=0;i<WAVETABLE_SIZE;++i) wavetables[current_wave_gen_idx][i]=(i<pw)?1.0f:-1.0f; wavetable_display_names[current_wave_gen_idx++] = "Pulse 12%"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { float mr=4.5f; for(int i=0;i<WAVETABLE_SIZE;++i){float p=(float)i/WAVETABLE_SIZE; float sp=fmod(p*mr,1.0f); wavetables[current_wave_gen_idx][i]=1.0f-2.0f*sp;} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Sync Saw"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float t=(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=0.5f*sinf(2*PI*t)+0.3f*sinf(2*PI*t*2.0f)+0.2f*sinf(2*PI*t*3.7f)+0.1f*sinf(2*PI*t*5.2f);} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Formant A"; } // Original Formant A
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i) wavetables[current_wave_gen_idx][i]=-1.0f+2.0f*(float)i/WAVETABLE_SIZE; wavetable_display_names[current_wave_gen_idx++] = "Ramp Up"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=sinf(t)+(1.0f/3.0f)*sinf(3*t)+(1.0f/5.0f)*sinf(5*t);} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "HollowSquare"; } // Original Hollow Square
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float t=(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=0.6f*sinf(2*PI*t)+0.4f*sinf(2*PI*t*1.8f)+0.1f*sinf(2*PI*t*4.5f);} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Formant O"; } // Original Formant O

    // Zusätzliche 30 Wellenformen (Index 11-40)
    // Startet bei current_wave_gen_idx = 11
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float p = 2.0f * PI * (float)i / WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i] = 0.6f * sinf(p) + 0.3f * sinf(2*p) + 0.2f * sinf(3*p); } normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "OrganPipe 1"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float p = 2.0f * PI * (float)i / WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i] = 0.5f*sinf(p) + 0.25f*sinf(2*p) + 0.12f*sinf(3*p) + 0.08f*sinf(4*p) + 0.05f*sinf(5*p); } normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "OrganPipe 2"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int k = 1; k <= 9; k += 2) for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[current_wave_gen_idx][i] += (1.0f / (k*k)) * sinf(2.0f * PI * k * (float)i / WAVETABLE_SIZE); normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Hollow Sq X"; } // Neues Hollow Sq
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = (float)i / WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i] = 0.4f*sinf(2*PI*t) + 0.3f*sinf(2*PI*t*2.1f) + 0.2f*sinf(2*PI*t*3.4f) + 0.1f*sinf(2*PI*t*4.9f); } normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Formant X"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = (float)i / WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i] = 0.5f*sinf(2*PI*t) + 0.35f*sinf(2*PI*t*1.5f) + 0.15f*sinf(2*PI*t*5.5f); } normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Formant Y"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float phase_mod = sinf(2.0f * PI * (float)i / WAVETABLE_SIZE); float distorted_phase = (float)i / WAVETABLE_SIZE + 0.3f * phase_mod; while(distorted_phase < 0.0f) distorted_phase += 1.0f; while(distorted_phase >= 1.0f) distorted_phase -= 1.0f; wavetables[current_wave_gen_idx][i] = sinf(2.0f * PI * distorted_phase); } normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "PD Saw"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float phase_mod = sinf(4.0f * PI * (float)i / WAVETABLE_SIZE); float distorted_phase = (float)i / WAVETABLE_SIZE + 0.2f * phase_mod; while(distorted_phase < 0.0f) distorted_phase += 1.0f; while(distorted_phase >= 1.0f) distorted_phase -= 1.0f; wavetables[current_wave_gen_idx][i] = sinf(2.0f * PI * distorted_phase) > 0 ? 1.0f : -1.0f; } normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "PD Square"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { float mf = 1.5f, md = 0.2f; for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = 2.0f * PI * (float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i] = sinf(t + md*sinf(mf*t));} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Simple FM 1"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { float mf = 2.718f, md = 0.35f; for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = 2.0f * PI * (float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i] = sinf(t + md*sinf(mf*t));} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Simple FM 2"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int k = 8; k <= 16; k++) for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[current_wave_gen_idx][i] += (1.0f / (k-7)) * sinf(2.0f * PI * k * (float)i / WAVETABLE_SIZE); normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Buzz Add"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float saw = 1.0f - 2.0f * (float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i] = tanhf(saw * 1.5f); } normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "SoftClip Saw"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float v = sinf(2*PI*(float)i/WAVETABLE_SIZE); wavetables[current_wave_gen_idx][i] = (v > 0) ? v : 0;} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "HalfRect Sine"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[current_wave_gen_idx][i] = fabsf(sinf(2*PI*(float)i/WAVETABLE_SIZE)); float s=0; for(int i=0;i<WAVETABLE_SIZE;++i)s+=wavetables[current_wave_gen_idx][i]; float a=s/WAVETABLE_SIZE; for(int i=0;i<WAVETABLE_SIZE;++i)wavetables[current_wave_gen_idx][i]-=a; normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "FullRect Sine"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float x = sinf(2*PI*(float)i/WAVETABLE_SIZE); wavetables[current_wave_gen_idx][i] = 8*powf(x,4) - 8*powf(x,2) + 1; } normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Cheby T4 Sine"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { if ((i % (WAVETABLE_SIZE/4)) < WAVETABLE_SIZE/16) wavetables[current_wave_gen_idx][i] = ((float)rand()/RAND_MAX)*2.0f-1.0f; else wavetables[current_wave_gen_idx][i] = 0;} wavetable_display_names[current_wave_gen_idx++] = "Noise Bursts"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { float rp=0.6f; for(int i=0;i<WAVETABLE_SIZE;++i){float p=(float)i/WAVETABLE_SIZE; if(p<rp)wavetables[current_wave_gen_idx][i]=1.0f-2.0f*(p/rp);else wavetables[current_wave_gen_idx][i]=1.0f-2.0f*((p-rp)/(1.0f-rp));} wavetable_display_names[current_wave_gen_idx++] = "Table SyncSaw"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { float o1=0.02f*WAVETABLE_SIZE, o2=-0.015f*WAVETABLE_SIZE; for(int i=0;i<WAVETABLE_SIZE;++i)wavetables[current_wave_gen_idx][i]=0.4f*(1.0f-2.0f*fmod((float)i/WAVETABLE_SIZE,1.0f)) + 0.3f*(1.0f-2.0f*fmod(((float)i+o1)/WAVETABLE_SIZE,1.0f)) + 0.3f*(1.0f-2.0f*fmod(((float)i+o2)/WAVETABLE_SIZE,1.0f)); normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Chorus Saws"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i)wavetables[current_wave_gen_idx][i]=sinf(2*PI*3.0f*(float)i/WAVETABLE_SIZE)*expf(-5.0f*(float)i/WAVETABLE_SIZE); normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Damped Sine"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { int s=8; for(int i=0;i<WAVETABLE_SIZE;++i){float sv=sinf(2*PI*(float)i/WAVETABLE_SIZE); float qv=floorf((sv+1.0f)*0.5f*s); wavetables[current_wave_gen_idx][i]=(qv/(s-1))*2.0f-1.0f;} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Staircase 8"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=0.5f*sinf(t*1.00f)*expf(-0.002f*i)+0.3f*sinf(t*1.66f)*expf(-0.004f*i)+0.2f*sinf(t*2.53f)*expf(-0.006f*i)+0.15f*sinf(t*3.78f)*expf(-0.008f*i);} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Rich Bell"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float sv=1.0f-2.0f*(float)i/WAVETABLE_SIZE; float dsv=(i>0)?(1.0f-2.0f*(float)(i-1)/WAVETABLE_SIZE):sv; float sp=2*PI*(float)i/WAVETABLE_SIZE; float rff=2.0f+1.8f*sinf(0.5f*PI*(float)i/WAVETABLE_SIZE); wavetables[current_wave_gen_idx][i]=0.6f*(sv-0.5f*dsv)+0.4f*sinf(rff*sp);} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "ResoSweepSaw"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float tn=(float)i/WAVETABLE_SIZE; float v=0; v+=1.0f*sinf(2*PI*tn*1.0f)*expf(-tn*5.0f); v+=0.6f*sinf(2*PI*tn*2.0f)*expf(-tn*6.0f); v+=0.4f*sinf(2*PI*tn*3.0f)*expf(-tn*7.0f); v+=0.2f*sinf(2*PI*tn*4.0f)*expf(-tn*8.0f); wavetables[current_wave_gen_idx][i]=v;} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Guitar Pluck"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { float mf=3.14159f,md=0.8f; for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=sinf(t+md*sinf(mf*t));} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Metallic FM"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { float mf=0.5f, md=1.0f; for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=sinf(t + md*sinf(mf*t));} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "FM Bass Growl"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=0.7f*sinf(t) + 0.3f*sinf(1.98f*t);} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Detuned Sines"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=powf(sinf(t),3);} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Sine Cubed"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { int pw = WAVETABLE_SIZE/3; for(int i=0;i<WAVETABLE_SIZE;++i) wavetables[current_wave_gen_idx][i]=(i<pw)?1.0f:-0.5f; normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Pulse 33%"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=0.5f*(sinf(t) + sinf(1.5*t + 0.5f*sinf(6.0f*t)));} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "DX EPiano-ish"; }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=sinf(t) * (0.5f + 0.5f * sinf(0.5f*t));} normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Sine AM"; }


    // 5 zusätzliche, weniger obertonreiche Wellenformen (Index 41-45)
    // Startet bei current_wave_gen_idx = 11 (original) + 30 (neu) = 41
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { // Soft Sine (gefilterter Sinus)
        for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[current_wave_gen_idx][i] = 0.8f * sinf(2.0f*PI*(float)i/WAVETABLE_SIZE) + 0.2f * sinf(4.0f*PI*(float)i/WAVETABLE_SIZE); }
        normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Soft Sine";
    }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { // Rounded Square (wenige Harmonische)
        for (int k = 1; k <= 5; k += 2) for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[current_wave_gen_idx][i] += (1.0f / k) * sinf(2.0f * PI * k * (float)i / WAVETABLE_SIZE);
        normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Rounded Sq";
    }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { // Fundamental + Terz (Dur-artig)
        for (int i = 0; i < WAVETABLE_SIZE; ++i) { float p = 2.0f * PI * (float)i / WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i] = 0.7f * sinf(p) + 0.3f * sinf(p * 1.2599f); } // 1.2599 ist 2^(4/12) = große Terz
        normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Sine +Maj3rd";
    }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { // Zwei Sines leicht verstimmt (Chorus-artig, weich)
        float detune_factor = 1.015f;
        for (int i = 0; i < WAVETABLE_SIZE; ++i) {
            wavetables[current_wave_gen_idx][i] = 0.5f * sinf(2.0f * PI * (float)i / WAVETABLE_SIZE) +
                                                  0.5f * sinf(2.0f * PI * (float)i * detune_factor / WAVETABLE_SIZE);
        }
        normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Soft Chorus";
    }
    if (current_wave_gen_idx < NUM_WAVETABLES_TOTAL) { // Sinus hoch 5 (noch weicher als hoch 3)
        for(int i=0;i<WAVETABLE_SIZE;++i){float t=2*PI*(float)i/WAVETABLE_SIZE; wavetables[current_wave_gen_idx][i]=powf(sinf(t),5);}
        normalizeWavetable(current_wave_gen_idx); wavetable_display_names[current_wave_gen_idx++] = "Sine Quintic";
    }
    
    Serial.printf("Total %d wavetables generated.\n", current_wave_gen_idx);
}


// Voice Allocation Logik aus dem Original-Code (V33.4) - bleibt unverändert
int findAvailableVoice(uint8_t noteVal) {
    for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) {
        if (voices[i].is_playing && voices[i].note == noteVal) { return i; }
    }
    for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) {
        if (!voices[i].is_playing && voices[i].env_stage == ENV_IDLE) { return i; }
    }
    int best_candidate_release = -1; float min_level_release = 2.0f;
    for (int i = 0; i < NUM_VOICES_ACTIVE; i++) {
        if (voices[i].is_playing && voices[i].env_stage == ENV_RELEASE && voices[i].env_level < min_level_release) {
            min_level_release = voices[i].env_level; best_candidate_release = i;
        }
    }
    if (best_candidate_release != -1) { return best_candidate_release; }

    float min_level_overall = 2.0f; int best_candidate_overall = 0;
    for (int i = 0; i < NUM_VOICES_ACTIVE; i++) {
        if (voices[i].env_stage != ENV_ATTACK) {
            if (voices[i].env_level < min_level_overall) {
                min_level_overall = voices[i].env_level;
                best_candidate_overall = i;
            }
        }
    }
    return best_candidate_overall;
}

// readAllDirectPotis, processWaveformSwitch, processArpControls, updateDisplay, 
// arpInternalNoteOn, arpInternalNoteOff, handleNoteOn, handleNoteOff, processArpeggiator,
// processGlobalSVF, fillAudioBuffer, audioMainTask, controlMainTask, setup, loop
// bleiben exakt so, wie sie in deinem "V34.0-ArpExpansion" Code waren.
// Die einzige Änderung in processWaveformSwitch und updateDisplay ist,
// dass sie NUM_WAVETABLES_TOTAL statt NUM_WAVETABLES verwenden und auf
// wavetable_display_names zugreifen.

void readAllDirectPotis() { /* ... (Code aus V34.0) ... */ const int num_adc_samples = 3; long adc_sum; const int adc_delay_us = 5; adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_ATTACK_PIN); delayMicroseconds(adc_delay_us); } raw_pot_attack = adc_sum / num_adc_samples; global_attack_time_ms = map(raw_pot_attack, 0, 4095, 1, 2000); adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_DECAY_PIN); delayMicroseconds(adc_delay_us); } raw_pot_decay = adc_sum / num_adc_samples; global_decay_time_ms = map(raw_pot_decay, 0, 4095, 1, 2000); adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_SUSTAIN_PIN); delayMicroseconds(adc_delay_us); } raw_pot_sustain = adc_sum / num_adc_samples; global_sustain_level = (float)raw_pot_sustain / 4095.0f; adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_RELEASE_PIN); delayMicroseconds(adc_delay_us); } raw_pot_release = adc_sum / num_adc_samples; global_release_time_ms = map(raw_pot_release, 0, 4095, 1, 3000); adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_FILTER_CUTOFF_PIN); delayMicroseconds(adc_delay_us); } raw_pot_filter_cutoff = adc_sum / num_adc_samples; float cutoff_norm = (float)raw_pot_filter_cutoff / 4095.0f; global_filter_cutoff_hz_target = 20.0f + (cutoff_norm * cutoff_norm * ( (float)SAMPLE_RATE / 2.5f - 20.0f) ); if (global_filter_cutoff_hz_target > (float)SAMPLE_RATE / 2.2f) global_filter_cutoff_hz_target = (float)SAMPLE_RATE / 2.2f; if (global_filter_cutoff_hz_target < 20.0f) global_filter_cutoff_hz_target = 20.0f; smoothed_filter_cutoff_hz = smoothed_filter_cutoff_hz * (1.0f - CUTOFF_SMOOTHING_FACTOR) + global_filter_cutoff_hz_target * CUTOFF_SMOOTHING_FACTOR; adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_FILTER_RESO_PIN); delayMicroseconds(adc_delay_us); } raw_pot_filter_reso = adc_sum / num_adc_samples; global_filter_resonance_target = map(raw_pot_filter_reso, 0, 4095, 70, 600) / 100.0f; if (global_filter_resonance_target < 0.707f) global_filter_resonance_target = 0.707f; if (global_filter_resonance_target > 6.0f) global_filter_resonance_target = 6.0f; smoothed_filter_resonance = smoothed_filter_resonance * (1.0f - RESO_SMOOTHING_FACTOR) + global_filter_resonance_target * RESO_SMOOTHING_FACTOR; adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_RATE_PIN); delayMicroseconds(adc_delay_us); } raw_pot_lfo_rate = adc_sum / num_adc_samples; if (raw_pot_lfo_rate < LFO_RATE_POTI_DEADZONE) { global_lfo_rate_hz_target = 0.0f; smoothed_lfo_rate_hz = 0.0f; } else { float mapped_rate = (float)(raw_pot_lfo_rate - LFO_RATE_POTI_DEADZONE) / (4095.0f - LFO_RATE_POTI_DEADZONE); if (mapped_rate < 0.0f) mapped_rate = 0.0f; if (mapped_rate > 1.0f) mapped_rate = 1.0f; global_lfo_rate_hz_target = mapped_rate * 10.0f; smoothed_lfo_rate_hz = smoothed_lfo_rate_hz * (1.0f - LFO_RATE_SMOOTHING_FACTOR) + global_lfo_rate_hz_target * LFO_RATE_SMOOTHING_FACTOR;} if (global_lfo_rate_hz_target < 0.001f) { smoothed_lfo_rate_hz = 0.0f;} adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_PITCH_DEPTH_PIN); delayMicroseconds(adc_delay_us); } raw_pot_lfo_pitch_depth_poti_raw = adc_sum / num_adc_samples; if (raw_pot_lfo_pitch_depth_poti_raw < LFO_PITCH_DEPTH_POTI_DEADZONE) { global_lfo_pitch_depth_base_poti = 0.0f; } else { float mapped_value = (float)(raw_pot_lfo_pitch_depth_poti_raw - LFO_PITCH_DEPTH_POTI_DEADZONE) / (4095.0f - LFO_PITCH_DEPTH_POTI_DEADZONE); if (mapped_value < 0.0f) mapped_value = 0.0f; if (mapped_value > 1.0f) mapped_value = 1.0f;  global_lfo_pitch_depth_base_poti = mapped_value * 1.0f;  } adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_MOD_WHEEL_PIN); delayMicroseconds(adc_delay_us); } raw_pot_mod_wheel = adc_sum / num_adc_samples; if (raw_pot_mod_wheel < MOD_WHEEL_POTI_DEADZONE) { global_mod_wheel_value = 0.0f; } else { float mapped_mw = (float)(raw_pot_mod_wheel - MOD_WHEEL_POTI_DEADZONE) / (4095.0f - MOD_WHEEL_POTI_DEADZONE); if (mapped_mw < 0.0f) mapped_mw = 0.0f; if (mapped_mw > 1.0f) mapped_mw = 1.0f; global_mod_wheel_value = mapped_mw;  } global_lfo_pitch_depth = global_lfo_pitch_depth_base_poti * global_mod_wheel_value; adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_FILTER_DEPTH_PIN); delayMicroseconds(adc_delay_us); } raw_pot_lfo_filter_depth_raw = adc_sum / num_adc_samples; if (raw_pot_lfo_filter_depth_raw < LFO_FILTER_DEPTH_POTI_DEADZONE) { global_lfo_filter_depth = 0.0f; } else { float mapped_value = (float)(raw_pot_lfo_filter_depth_raw - LFO_FILTER_DEPTH_POTI_DEADZONE) / (4095.0f - LFO_FILTER_DEPTH_POTI_DEADZONE); if (mapped_value < 0.0f) mapped_value = 0.0f; if (mapped_value > 1.0f) mapped_value = 1.0f; global_lfo_filter_depth = mapped_value * 1.0f; } adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_PITCHBEND_PIN); delayMicroseconds(adc_delay_us); } raw_pot_pitchbend = adc_sum / num_adc_samples; int center_adc = 2048; if (abs(raw_pot_pitchbend - center_adc) < PITCHBEND_POTI_DEADZONE_CENTER) { global_pitchbend_factor = 1.0f; } else { float bend_norm; if (raw_pot_pitchbend < center_adc) { bend_norm = (float)(raw_pot_pitchbend - (center_adc - PITCHBEND_POTI_DEADZONE_CENTER)) / (float)(center_adc - PITCHBEND_POTI_DEADZONE_CENTER); bend_norm = -1.0f * (1.0f - bend_norm); } else { bend_norm = (float)(raw_pot_pitchbend - (center_adc + PITCHBEND_POTI_DEADZONE_CENTER)) / (float)((4095 - (center_adc + PITCHBEND_POTI_DEADZONE_CENTER))); } if (bend_norm < -1.0f) bend_norm = -1.0f; if (bend_norm > 1.0f) bend_norm = 1.0f; global_pitchbend_factor = powf(2.0f, (bend_norm * MAX_PITCHBEND_SEMITONES) / 12.0f); } adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_MASTER_VOLUME_PIN); delayMicroseconds(adc_delay_us); } raw_pot_master_volume = adc_sum / num_adc_samples; if (raw_pot_master_volume < MASTER_VOLUME_POTI_DEADZONE) { global_master_volume = 0.0f; } else { global_master_volume = (float)(raw_pot_master_volume - MASTER_VOLUME_POTI_DEADZONE) / (4095.0f - MASTER_VOLUME_POTI_DEADZONE); if(global_master_volume > 1.0f) global_master_volume = 1.0f; if(global_master_volume < 0.0f) global_master_volume = 0.0f; } }

void processWaveformSwitch() {
    if (digitalRead(WAVE_SWITCH_PIN) == LOW) {
        if (millis() - last_wave_switch_press_time > WAVE_SWITCH_DEBOUNCE_TIME) {
            if (xSemaphoreTake(voicesMutex, (TickType_t)10) == pdTRUE) {
                global_osc_wave_select_idx = (global_osc_wave_select_idx + 1) % NUM_WAVETABLES_TOTAL; // Geändert auf TOTAL
                display_update_needed = true;
                xSemaphoreGive(voicesMutex);
            }
            last_wave_switch_press_time = millis();
        }
    }
}

void processArpControls() { /* ... (Code aus V34.0) ... */ if (digitalRead(ARP_ONOFF_SWITCH_PIN) == LOW) { if (millis() - last_arp_onoff_press_time > ARP_ONOFF_DEBOUNCE_TIME) { arp_is_on = !arp_is_on; if (!arp_is_on && arp_last_played_note_value != 0) { arpInternalNoteOff(arp_last_played_note_value); arp_last_played_note_value = 0; arp_num_held_notes = 0; arp_num_played_order_notes = 0; } display_update_needed = true; last_arp_onoff_press_time = millis(); } } if (digitalRead(ARP_MODE_SWITCH_PIN) == LOW) { if (millis() - last_arp_mode_press_time > ARP_MODE_DEBOUNCE_TIME) { current_arp_mode = (ArpMode)((current_arp_mode + 1) % NUM_ARP_MODES_SELECTABLE); arp_current_note_index = 0; arp_current_played_order_index = 0; arp_up_down_direction_is_up = true; arp_octave_shift = 0; switch(current_arp_mode) { case ARP_UP_DOUBLE: case ARP_DOWN_DOUBLE: case ARP_UP_DOWN_DOUBLE: current_arp_interval_ms = ARP_BASE_INTERVAL_MS / 2; break; case ARP_UP_HALF: case ARP_DOWN_HALF: case ARP_UP_DOWN_HALF: current_arp_interval_ms = ARP_BASE_INTERVAL_MS * 2; break; default: current_arp_interval_ms = ARP_BASE_INTERVAL_MS; break; } display_update_needed = true; last_arp_mode_press_time = millis(); } } }

void updateDisplay() {
    if (!display_update_needed) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); display.print("Wave: ");
    // Zugriff auf wavetable_display_names statt original_wavetable_names
    if (global_osc_wave_select_idx < NUM_WAVETABLES_TOTAL) { display.print(wavetable_display_names[global_osc_wave_select_idx]); }
    else { display.print("Err"); }
    display.setCursor(0,10); display.print("Arp: ");
    if (arp_is_on) { display.print(arp_mode_names[current_arp_mode]); }
    else { display.print("Off"); }
    display.setCursor(0, 20); display.print("Cut: "); display.print((int)smoothed_filter_cutoff_hz);
    display.setCursor(0, 30); display.print("LFO: "); display.print(smoothed_lfo_rate_hz, 1); display.print("Hz");
    display.display();
    display_update_needed = false;
}

void arpInternalNoteOn(byte note, byte velocity) { /* ... (Code aus V34.0) ... */ if (xSemaphoreTake(voicesMutex, (TickType_t)10) == pdTRUE) { int voice_idx = findAvailableVoice(note); VoiceState& v = voices[voice_idx]; v.note = note; v.velocity_gain = (float)velocity / 127.0f; v.note_on_trigger = true; v.note_off_trigger = false; v.current_wave_idx = global_osc_wave_select_idx; v.phase_accumulator = 0.0f; v.env_attack_rate = (global_attack_time_ms > 0.001f) ? (1.0f / (global_attack_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f; v.env_decay_rate = (global_decay_time_ms > 0.001f) ? (1.0f / (global_decay_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f; v.env_sustain_level_voice = global_sustain_level; v.env_release_rate = (global_release_time_ms > 0.001f) ? (1.0f / (global_release_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f; xSemaphoreGive(voicesMutex); } }
void arpInternalNoteOff(byte note) { /* ... (Code aus V34.0) ... */ if (xSemaphoreTake(voicesMutex, (TickType_t)10) == pdTRUE) { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { if (voices[i].is_playing && voices[i].note == note) { voices[i].note_off_trigger = true; } } xSemaphoreGive(voicesMutex); } }

void handleNoteOn(byte channel, byte note, byte velocity) { /* ... (Code aus V34.0) ... */ bool note_already_held_sorted = false; for (int i = 0; i < arp_num_held_notes; ++i) { if (arp_held_notes[i] == note) { note_already_held_sorted = true; break; } } bool note_already_held_order = false; for (int i = 0; i < arp_num_played_order_notes; ++i) { if (arp_notes_by_order[i] == note) { note_already_held_order = true; break; } } if (!note_already_held_sorted && arp_num_held_notes < ARP_MAX_NOTES) { arp_held_notes[arp_num_held_notes] = note; arp_num_held_notes = arp_num_held_notes + 1; for (int i = 0; i < arp_num_held_notes - 1; i++) { for (int j = i + 1; j < arp_num_held_notes; j++) { if (arp_held_notes[i] > arp_held_notes[j]) { uint8_t temp = arp_held_notes[i]; arp_held_notes[i] = arp_held_notes[j]; arp_held_notes[j] = temp; } } } } if (!note_already_held_order && arp_num_played_order_notes < ARP_MAX_NOTES) { arp_notes_by_order[arp_num_played_order_notes] = note; arp_num_played_order_notes = arp_num_played_order_notes + 1; } arp_last_velocity = velocity; if (!arp_is_on) { arpInternalNoteOn(note, velocity); } else { if (arp_num_held_notes == 1) { arp_current_note_index = 0; arp_current_played_order_index = 0; arp_octave_shift = 0; } } }
void handleNoteOff(byte channel, byte note, byte velocity) { /* ... (Code aus V34.0) ... */ bool found_and_removed_sorted = false; int removed_idx_sorted = -1; for (int i = 0; i < arp_num_held_notes; ++i) { if (arp_held_notes[i] == note) { found_and_removed_sorted = true; removed_idx_sorted = i; for (int j = i; j < arp_num_held_notes - 1; ++j) { arp_held_notes[j] = arp_held_notes[j+1]; } arp_num_held_notes = arp_num_held_notes - 1; break; } } bool found_and_removed_order = false; int removed_idx_order = -1; for (int i = 0; i < arp_num_played_order_notes; ++i) { if (arp_notes_by_order[i] == note) { found_and_removed_order = true; removed_idx_order = i; for (int j = i; j < arp_num_played_order_notes - 1; ++j) { arp_notes_by_order[j] = arp_notes_by_order[j+1]; } arp_num_played_order_notes = arp_num_played_order_notes - 1; break; } } if (found_and_removed_sorted || found_and_removed_order) { if (!arp_is_on) { arpInternalNoteOff(note); } else { if (arp_last_played_note_value == note || arp_last_played_note_value == note + 12 || arp_last_played_note_value == note + 24 || arp_last_played_note_value == note - 12 || arp_last_played_note_value == note - 24 ) { arpInternalNoteOff(arp_last_played_note_value); arp_last_played_note_value = 0; } if (arp_num_held_notes == 0) { if (arp_last_played_note_value != 0) arpInternalNoteOff(arp_last_played_note_value); arp_last_played_note_value = 0; arp_current_note_index = 0; arp_current_played_order_index = 0; arp_octave_shift = 0; } else { if (found_and_removed_sorted && removed_idx_sorted != -1 && removed_idx_sorted < arp_current_note_index) { arp_current_note_index = arp_current_note_index -1; } if (arp_current_note_index >= arp_num_held_notes && arp_num_held_notes > 0) { arp_current_note_index = 0; } if (arp_current_note_index < 0) arp_current_note_index = 0; if (found_and_removed_order && removed_idx_order != -1 && removed_idx_order < arp_current_played_order_index) { arp_current_played_order_index = arp_current_played_order_index - 1; } if (arp_current_played_order_index >= arp_num_played_order_notes && arp_num_played_order_notes > 0) { arp_current_played_order_index = 0; } if (arp_current_played_order_index < 0) arp_current_played_order_index = 0; } } } }

void processArpeggiator() { /* ... (Code aus V34.0) ... */ if (!arp_is_on || arp_num_held_notes == 0) { if (arp_last_played_note_value != 0) { arpInternalNoteOff(arp_last_played_note_value); arp_last_played_note_value = 0; } return; } if (millis() - last_arp_step_time >= current_arp_interval_ms) { if (arp_last_played_note_value != 0) { arpInternalNoteOff(arp_last_played_note_value); arp_last_played_note_value = 0; } if (arp_num_held_notes > 0) { uint8_t base_note_to_play = 0; int temp_arp_index = arp_current_note_index; int temp_played_order_index = arp_current_played_order_index; ArpMode effective_mode = current_arp_mode; if (current_arp_mode == ARP_UP_DOUBLE || current_arp_mode == ARP_UP_HALF) effective_mode = ARP_UP; if (current_arp_mode == ARP_DOWN_DOUBLE || current_arp_mode == ARP_DOWN_HALF) effective_mode = ARP_DOWN; if (current_arp_mode == ARP_UP_DOWN_DOUBLE || current_arp_mode == ARP_UP_DOWN_HALF) effective_mode = ARP_UP_DOWN; switch (effective_mode) { case ARP_UP: temp_arp_index = (temp_arp_index + 1) % arp_num_held_notes; base_note_to_play = arp_held_notes[temp_arp_index]; break; case ARP_DOWN: temp_arp_index = temp_arp_index - 1; if (temp_arp_index < 0) { temp_arp_index = arp_num_held_notes - 1; } base_note_to_play = arp_held_notes[temp_arp_index]; break; case ARP_UP_DOWN: if (arp_up_down_direction_is_up) { temp_arp_index = temp_arp_index + 1; if (temp_arp_index >= arp_num_held_notes) { temp_arp_index = arp_num_held_notes > 1 ? arp_num_held_notes - 2 : 0; arp_up_down_direction_is_up = false; } } else { temp_arp_index = temp_arp_index - 1; if (temp_arp_index < 0) { temp_arp_index = arp_num_held_notes > 1 ? 1 : 0; arp_up_down_direction_is_up = true; } } if (arp_num_held_notes > 0) base_note_to_play = arp_held_notes[temp_arp_index]; else { arp_last_played_note_value = 0; return; } break; case ARP_RANDOM: if (arp_num_held_notes > 0) temp_arp_index = random(arp_num_held_notes); base_note_to_play = arp_held_notes[temp_arp_index]; break; case ARP_ORDER_PLAYED: if (arp_num_played_order_notes > 0) { temp_played_order_index = (temp_played_order_index + 1) % arp_num_played_order_notes; base_note_to_play = arp_notes_by_order[temp_played_order_index]; } else { arp_last_played_note_value = 0; return; } break; case ARP_CHORD: for(int i=0; i < arp_num_held_notes; ++i) { arpInternalNoteOn(arp_held_notes[i], arp_last_velocity); } arp_last_played_note_value = arp_held_notes[0]; last_arp_step_time = millis(); return; case ARP_OCT_UP: case ARP_OCT_DOWN: temp_arp_index = (temp_arp_index + 1); if (temp_arp_index >= arp_num_held_notes) { temp_arp_index = 0; if (effective_mode == ARP_OCT_UP) { arp_octave_shift++; if (arp_octave_shift >= ARP_MAX_OCTAVE_STEPS) arp_octave_shift = 0; } else { arp_octave_shift--; if (arp_octave_shift <= -ARP_MAX_OCTAVE_STEPS) arp_octave_shift = 0; } } base_note_to_play = arp_held_notes[temp_arp_index]; break; default: base_note_to_play = arp_held_notes[arp_current_note_index]; break; } arp_current_note_index = temp_arp_index; arp_current_played_order_index = temp_played_order_index; uint8_t final_note_to_play = base_note_to_play; if (current_arp_mode == ARP_OCT_UP || current_arp_mode == ARP_OCT_DOWN) { final_note_to_play = base_note_to_play + (arp_octave_shift * 12); if (final_note_to_play > 127) final_note_to_play = base_note_to_play; if (final_note_to_play < 0) final_note_to_play = base_note_to_play; } if (arp_num_held_notes > 0) { if (arp_current_note_index < 0 || arp_current_note_index >= arp_num_held_notes) { arp_current_note_index = 0; } if (arp_num_played_order_notes > 0 && (arp_current_played_order_index < 0 || arp_current_played_order_index >= arp_num_played_order_notes)) { arp_current_played_order_index = 0; } } else { arp_last_played_note_value = 0; return; } if (current_arp_mode != ARP_CHORD) { arp_last_played_note_value = final_note_to_play; arpInternalNoteOn(final_note_to_play, arp_last_velocity); } } else { arp_last_played_note_value = 0; } last_arp_step_time = millis(); } }

inline float processGlobalSVF(float input_sample) { /* ... (Code aus V34.0) ... */ float cutoff_freq_hz = smoothed_filter_cutoff_hz; if (lfo_increment > 0.000001f && global_lfo_filter_depth > 0.001f) { float lfo_val_for_filter = interpolateWavetable(wavetables[0], lfo_phase); float mod_range = cutoff_freq_hz * 0.8f;  cutoff_freq_hz += lfo_val_for_filter * global_lfo_filter_depth * mod_range; if (cutoff_freq_hz < 20.0f) cutoff_freq_hz = 20.0f; if (cutoff_freq_hz > (float)SAMPLE_RATE / 2.1f) cutoff_freq_hz = (float)SAMPLE_RATE / 2.1f; } float resonance_q = smoothed_filter_resonance; if (resonance_q < 0.5f) resonance_q = 0.5f; if (resonance_q > 6.0f) resonance_q = 6.0f; float g = tanf(PI * cutoff_freq_hz / SAMPLE_RATE);  float k = 1.0f / resonance_q; float hp_output = input_sample - global_filter_integrator2_state - k * global_filter_integrator1_state; float current_bp_output = g * hp_output + global_filter_integrator1_state; float current_lp_output = g * current_bp_output + global_filter_integrator2_state; global_filter_integrator1_state = current_bp_output;  global_filter_integrator2_state = current_lp_output;  global_filter_bp_output = current_bp_output; global_filter_lp_output = current_lp_output; if (isnan(global_filter_integrator1_state) || isinf(global_filter_integrator1_state) || isnan(global_filter_integrator2_state) || isinf(global_filter_integrator2_state)) { global_filter_integrator1_state = 0.0f; global_filter_integrator2_state = 0.0f; global_filter_bp_output = 0.0f; global_filter_lp_output = 0.0f;  return 0.0f;  } return global_filter_lp_output;  }

void fillAudioBuffer() { /* ... (Code aus V34.0) ... */ if (smoothed_lfo_rate_hz > 0.0001f) { lfo_increment = (smoothed_lfo_rate_hz * WAVETABLE_SIZE) / SAMPLE_RATE; lfo_phase += lfo_increment * DMA_BUFFER_FRAMES; while (lfo_phase >= WAVETABLE_SIZE) { lfo_phase -= WAVETABLE_SIZE; } } else { lfo_increment = 0.0f; } float lfo_val_for_pitch = interpolateWavetable(wavetables[0], lfo_phase); VoiceState local_voices[NUM_VOICES_ACTIVE]; if (xSemaphoreTake(voicesMutex, portMAX_DELAY) == pdTRUE) { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { local_voices[i] = voices[i]; if (voices[i].note_on_trigger) { local_voices[i].is_playing = true; local_voices[i].env_stage = ENV_ATTACK; local_voices[i].env_level = 0.0f; local_voices[i].phase_accumulator = 0.0f; voices[i].note_on_trigger = false; } if (voices[i].note_off_trigger) { if (local_voices[i].is_playing && local_voices[i].env_stage != ENV_RELEASE) { local_voices[i].env_stage = ENV_RELEASE; } voices[i].note_off_trigger = false; } } xSemaphoreGive(voicesMutex); } else { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { local_voices[i].is_playing = false; local_voices[i].env_stage = ENV_IDLE; local_voices[i].env_level = 0.0f; } } for (int frame = 0; frame < DMA_BUFFER_FRAMES; ++frame) { float sum_of_voices_before_filter = 0.0f; for (int v_idx = 0; v_idx < NUM_VOICES_ACTIVE; ++v_idx) { VoiceState& v_local = local_voices[v_idx]; if (!v_local.is_playing && v_local.env_stage == ENV_IDLE) { continue; } switch (v_local.env_stage) { case ENV_ATTACK: v_local.env_level += v_local.env_attack_rate; if (v_local.env_level >= 1.0f) { v_local.env_level = 1.0f; v_local.env_stage = ENV_DECAY; } break; case ENV_DECAY: if (v_local.env_level > v_local.env_sustain_level_voice) { v_local.env_level -= v_local.env_decay_rate; if (v_local.env_level <= v_local.env_sustain_level_voice) { v_local.env_level = v_local.env_sustain_level_voice;} } if (v_local.env_level <= v_local.env_sustain_level_voice) { if (v_local.env_sustain_level_voice < 0.001f) { v_local.env_stage = ENV_RELEASE; } else { v_local.env_stage = ENV_SUSTAIN; } } break; case ENV_SUSTAIN: break; case ENV_RELEASE: v_local.env_level -= v_local.env_release_rate; if (v_local.env_level <= 0.0f) { v_local.env_level = 0.0f; v_local.env_stage = ENV_IDLE; v_local.is_playing = false; } break; case ENV_IDLE: default: v_local.env_level = 0.0f; v_local.is_playing = false; continue; } if (v_local.env_level < 0.0f) v_local.env_level = 0.0f; if (v_local.is_playing && (v_local.env_level > 0.0001f || v_local.env_stage == ENV_ATTACK)) { float base_freq = midiNoteToFreq(v_local.note) * global_pitchbend_factor; float pitch_mod_factor = 1.0f; if (lfo_increment > 0.000001f && global_lfo_pitch_depth > 0.0001f) { pitch_mod_factor = powf(2.0f, (lfo_val_for_pitch * global_lfo_pitch_depth * 12.0f) / 12.0f); } float current_note_freq = base_freq * pitch_mod_factor; v_local.phase_increment = (current_note_freq * WAVETABLE_SIZE) / SAMPLE_RATE; v_local.phase_accumulator += v_local.phase_increment; if (v_local.phase_accumulator >= WAVETABLE_SIZE) { v_local.phase_accumulator -= WAVETABLE_SIZE; } float main_osc_output = interpolateWavetable(wavetables[v_local.current_wave_idx], v_local.phase_accumulator); sum_of_voices_before_filter += main_osc_output * v_local.velocity_gain * v_local.env_level; } } sum_of_voices_before_filter *= PRE_FILTER_DRIVE_ATTENUATION; float filtered_output = processGlobalSVF(sum_of_voices_before_filter); filtered_output *= global_master_volume; if (POLYPHONY_DAMPING_FACTOR > 0.01f) { filtered_output /= POLYPHONY_DAMPING_FACTOR; } if (filtered_output > 1.0f) filtered_output = 1.0f; else if (filtered_output < -1.0f) filtered_output = -1.0f; int16_t sample_out = (int16_t)(filtered_output * MAX_SYSTEM_AMPLITUDE_FLOAT); i2s_audio_buffer[frame * NUM_CHANNELS + 0] = sample_out; i2s_audio_buffer[frame * NUM_CHANNELS + 1] = sample_out; } if (xSemaphoreTake(voicesMutex, portMAX_DELAY) == pdTRUE) { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { voices[i].is_playing = local_voices[i].is_playing; voices[i].env_stage = local_voices[i].env_stage; voices[i].env_level = local_voices[i].env_level; voices[i].phase_accumulator = local_voices[i].phase_accumulator; } xSemaphoreGive(voicesMutex); } size_t bytes_written; i2s_channel_write(i2s_tx_chan, i2s_audio_buffer, I2S_WRITE_BUFFER_SIZE_BYTES, &bytes_written, portMAX_DELAY); if (bytes_written != I2S_WRITE_BUFFER_SIZE_BYTES && bytes_written > 0) { /* Serial.printf("I2S underrun? Wrote %d, expected %d\n", bytes_written, I2S_WRITE_BUFFER_SIZE_BYTES); */ } }

void audioMainTask(void *pvParameters) { Serial.println("Audio Task started on Core 1."); while (true) { fillAudioBuffer(); } }
void controlMainTask(void *pvParameters) { /* ... (Code aus V34.0) ... */ Serial.println("Control Task started on Core 0."); unsigned long last_pot_read_control_task = 0; unsigned long last_debug_print_time = 0; unsigned long last_display_update_time = 0; while (true) { MIDI.read(); processWaveformSwitch(); processArpControls(); processArpeggiator(); unsigned long current_time_ms = millis(); if (current_time_ms - last_pot_read_control_task >= DIRECT_POT_READ_INTERVAL_MS) { readAllDirectPotis(); last_pot_read_control_task = current_time_ms; display_update_needed = true; if (current_time_ms - last_debug_print_time > 500) { /* Serial.print("Wave: "); Serial.print(wavetable_display_names[global_osc_wave_select_idx]); Serial.print(" | Arp: "); Serial.print(arp_is_on ? arp_mode_names[current_arp_mode] : "Off"); Serial.print(" | NotesHeld: "); Serial.print(arp_num_held_notes); Serial.println(); */ last_debug_print_time = current_time_ms; } } if (display_update_needed && (current_time_ms - last_display_update_time > 50)) { updateDisplay(); last_display_update_time = current_time_ms; } vTaskDelay(pdMS_TO_TICKS(1)); } }

void setup() { /* ... (Code aus V34.0, mit Anpassung der Versionsnummer und Initialisierung von wavetable_display_names) ... */ Serial.begin(115200); vTaskDelay(pdMS_TO_TICKS(500)); Serial.println("\nESP32-S3 Poly Synth - (V34.1-ExtendedWaves)"); Wire.begin(); if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) { Serial.println(F("SSD1306 allocation failed")); } else { Serial.println(F("SSD1306 Initialized")); display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.println("Synth V34.1 Waves+"); display.display(); display_update_needed = true; } pinMode(WAVE_SWITCH_PIN, INPUT_PULLUP); pinMode(ARP_ONOFF_SWITCH_PIN, INPUT_PULLUP); pinMode(ARP_MODE_SWITCH_PIN, INPUT_PULLUP); pinMode(POT_ATTACK_PIN, INPUT); pinMode(POT_DECAY_PIN, INPUT); pinMode(POT_SUSTAIN_PIN, INPUT); pinMode(POT_RELEASE_PIN, INPUT); pinMode(POT_FILTER_CUTOFF_PIN, INPUT); pinMode(POT_FILTER_RESO_PIN, INPUT); pinMode(POT_LFO_RATE_PIN, INPUT); pinMode(POT_LFO_PITCH_DEPTH_PIN, INPUT); pinMode(POT_LFO_FILTER_DEPTH_PIN, INPUT); pinMode(POT_MASTER_VOLUME_PIN, INPUT); pinMode(POT_PITCHBEND_PIN, INPUT); pinMode(POT_MOD_WHEEL_PIN, INPUT); Serial.println("Poti & Switch Pins initialized."); generateAllSynthWavetables(); Serial.println("Wavetables generated."); voicesMutex = xSemaphoreCreateMutex(); if (voicesMutex == NULL) { Serial.println("FATAL: Mutex creation failed!"); while (1) vTaskDelay(1000); } for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { voices[i].note_on_trigger = false; voices[i].note_off_trigger = false; voices[i].is_playing = false; voices[i].env_stage = ENV_IDLE; voices[i].env_level = 0.0f; voices[i].phase_accumulator = 0.0f; voices[i].current_wave_idx = 0; } global_filter_integrator1_state = 0.0f; global_filter_integrator2_state = 0.0f; global_filter_lp_output = 0.0f; global_filter_bp_output = 0.0f; Serial.println("Global resources & Voices initialized."); Serial2.begin(MIDI_BAUD_RATE, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN_FOR_SERIAL2); MIDI.setHandleNoteOn(handleNoteOn); MIDI.setHandleNoteOff(handleNoteOff); MIDI.begin(MIDI_LISTEN_CHANNEL); Serial.println("MIDI Initialized on Serial2."); i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); chan_cfg.dma_desc_num = DMA_BUFFER_COUNT; chan_cfg.dma_frame_num = DMA_BUFFER_FRAMES; chan_cfg.auto_clear = true; esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL); if (err != ESP_OK) { Serial.printf("I2S: Channel creation failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); } i2s_std_config_t std_cfg = { .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE), .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), .gpio_cfg = { .mclk = I2S_MCLK_PIN, .bclk = I2S_BCLK_PIN, .ws = I2S_LRCK_PIN, .dout = I2S_DOUT_PIN, .din = GPIO_NUM_NC, .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false, } } }; std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL; err = i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg); if (err != ESP_OK) { Serial.printf("I2S: Standard mode init failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); } err = i2s_channel_enable(i2s_tx_chan); if (err != ESP_OK) { Serial.printf("I2S: Channel enable failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); } Serial.println("I2S Initialized and Enabled (SR: " + String(SAMPLE_RATE) + " Hz)."); xTaskCreatePinnedToCore( audioMainTask, "AudioMain", 16384, NULL, configMAX_PRIORITIES - 1, &audioMainTaskHandle, 1 ); xTaskCreatePinnedToCore( controlMainTask, "ControlMain", 4096 + 2048, NULL, 5, &controlMainTaskHandle, 0 ); Serial.println("--- Setup Complete. Tasks Running. ---"); }

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
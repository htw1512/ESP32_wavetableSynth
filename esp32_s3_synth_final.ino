// ESP32-S3 - 6-VOICE POLYPHONIC SYNTH - (V33 - Master Vol Fix, Erweiterter Arp)

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
#define SAMPLE_RATE         22050
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

// --- Potis ---
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

// --- Taster ---
#define WAVE_SWITCH_PIN      GPIO_NUM_19 
#define ARP_ONOFF_SWITCH_PIN GPIO_NUM_18 // << NEU für Arp An/Aus >>
#define ARP_MODE_SWITCH_PIN  GPIO_NUM_17 // << NEU für Arp Modus >>


// --- OLED Display ---
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define OLED_RESET    -1 
#define OLED_I2C_ADDRESS 0x3C 

// --- Synthesizer Allgemein ---
#define NUM_VOICES_ACTIVE   8   
#define WAVETABLE_SIZE      256
const float POLYPHONY_DAMPING_FACTOR = 4.0f; 
const float PRE_FILTER_DRIVE_ATTENUATION = 0.5f; 
#define WAVE_SINE           0
#define WAVE_SAW_DOWN       1 
#define WAVE_SQUARE         2 
#define WAVE_TRIANGLE       3
#define WAVE_PULSE_25       4 
#define WAVE_PULSE_12       5 
#define WAVE_SYNC_SAW       6 
#define WAVE_FORMANT_A      7 
#define WAVE_RAMP_UP        8 
#define WAVE_HOLLOW_SQUARE  9 
#define WAVE_FORMANT_O     10 
#define NUM_WAVETABLES     11 

// --- Arpeggiator ---
enum ArpMode { ARP_OFF, ARP_UP, ARP_DOWN, ARP_UP_DOWN, ARP_RANDOM, NUM_ARP_MODES };
const char* arp_mode_names[NUM_ARP_MODES] = {"Off", "Up", "Down", "Up/Down", "Random"};
volatile ArpMode current_arp_mode = ARP_UP; // Startmodus
volatile bool arp_is_on = false; // Ist der Arpeggiator global aktiv?

#define ARP_MAX_NOTES 10        
volatile uint8_t arp_held_notes[ARP_MAX_NOTES];
volatile int arp_num_held_notes = 0;
volatile int arp_current_note_index = 0;
// volatile bool arp_active = false; // Ersetzt durch arp_is_on und arp_num_held_notes > 1 Logik
unsigned long last_arp_step_time = 0;
const unsigned long ARP_STEP_INTERVAL_MS = 150; 
volatile uint8_t arp_last_played_note = 0; 
volatile uint8_t arp_last_velocity = 0;
bool arp_up_down_direction_is_up = true; // Für Up/Down Modus


enum EnvelopeStage { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };
struct VoiceState { 
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
float wavetables[NUM_WAVETABLES][WAVETABLE_SIZE]; 
unsigned long last_direct_pot_read_time = 0;
const unsigned long DIRECT_POT_READ_INTERVAL_MS = 20; 
SemaphoreHandle_t voicesMutex = NULL;
TaskHandle_t audioMainTaskHandle = NULL;
TaskHandle_t controlMainTaskHandle = NULL;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
volatile bool display_update_needed = true; 
volatile int global_osc_wave_select_idx = WAVE_SINE; 
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
const int MASTER_VOLUME_POTI_DEADZONE = 50; // << NEU für Master Volume >>

unsigned long last_wave_switch_press_time = 0;
const unsigned long WAVE_SWITCH_DEBOUNCE_TIME = 70; 
unsigned long last_arp_onoff_press_time = 0;
const unsigned long ARP_ONOFF_DEBOUNCE_TIME = 70;
unsigned long last_arp_mode_press_time = 0;
const unsigned long ARP_MODE_DEBOUNCE_TIME = 70;

const char* wavetable_names[NUM_WAVETABLES] = { "Sine", "Saw Down", "Square", "Triangle", "Pulse 25%", "Pulse 12%", "Sync Saw", "Formant A", "Ramp Up", "HollowSq", "Formant O" };

// -----------------------------------------------------------------------------
// HILFSFUNKTIONEN
// -----------------------------------------------------------------------------
float midiNoteToFreq(uint8_t noteVal) { return 440.0f * powf(2.0f, (noteVal - 69.0f) / 12.0f); }
float interpolateWavetable(float table[WAVETABLE_SIZE], float index) { int i0 = (int)floorf(index); int i1 = (i0 + 1); float fraction = index - i0; i0 %= WAVETABLE_SIZE; if (i0 < 0) i0 += WAVETABLE_SIZE; i1 %= WAVETABLE_SIZE; if (i1 < 0) i1 += WAVETABLE_SIZE; return table[i0] * (1.0f - fraction) + table[i1] * fraction; }
void normalizeWavetable(int wave_idx) { float max_abs_val = 0.0f; for (int i = 0; i < WAVETABLE_SIZE; ++i) { if (fabsf(wavetables[wave_idx][i]) > max_abs_val) { max_abs_val = fabsf(wavetables[wave_idx][i]); } } if (max_abs_val > 0.0001f) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[wave_idx][i] /= max_abs_val; } } }
void generateAllSynthWavetables() {  for (int w = 0; w < NUM_WAVETABLES; ++w) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[w][i] = 0.0f; } } for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_SINE][i] = sinf(2.0f * PI * (float)i / WAVETABLE_SIZE); }  for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_SAW_DOWN][i] = 1.0f - 2.0f * (float)i / WAVETABLE_SIZE; }  for (int k = 1; k <= 15; k += 2) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_SQUARE][i] += (1.0f / k) * sinf(2.0f * PI * k * (float)i / WAVETABLE_SIZE); } } normalizeWavetable(WAVE_SQUARE); for (int i = 0; i < WAVETABLE_SIZE; ++i) { float val = 2.0f * (float)i / WAVETABLE_SIZE; if (val > 1.0f) { val = 2.0f - val; } wavetables[WAVE_TRIANGLE][i] = 2.0f * val - 1.0f; } if (WAVE_PULSE_25 < NUM_WAVETABLES) { int pw = WAVETABLE_SIZE / 4; for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_PULSE_25][i] = (i < pw) ? 1.0f : -1.0f; } } if (WAVE_PULSE_12 < NUM_WAVETABLES) { int pw = WAVETABLE_SIZE / 8; for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_PULSE_12][i] = (i < pw) ? 1.0f : -1.0f; } } if (WAVE_SYNC_SAW < NUM_WAVETABLES) { float master_freq_ratio = 4.5f; for (int i = 0; i < WAVETABLE_SIZE; ++i) { float phase = (float)i / WAVETABLE_SIZE; float slave_phase = fmod(phase * master_freq_ratio, 1.0f); wavetables[WAVE_SYNC_SAW][i] = 1.0f - 2.0f * slave_phase; } normalizeWavetable(WAVE_SYNC_SAW); } if (WAVE_FORMANT_A < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = (float)i / WAVETABLE_SIZE; wavetables[WAVE_FORMANT_A][i] = 0.5f * sinf(2.0f*PI*t) + 0.3f * sinf(2.0f*PI*t*2.0f) + 0.2f * sinf(2.0f*PI*t*3.7f) + 0.1f * sinf(2.0f*PI*t*5.2f); } normalizeWavetable(WAVE_FORMANT_A); } if (WAVE_RAMP_UP < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_RAMP_UP][i] = -1.0f + 2.0f * (float)i / WAVETABLE_SIZE; } } if (WAVE_HOLLOW_SQUARE < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = 2.0f * PI * (float)i / WAVETABLE_SIZE; wavetables[WAVE_HOLLOW_SQUARE][i] = sinf(t) + (1.0f/3.0f)*sinf(3.0f*t) + (1.0f/5.0f)*sinf(5.0f*t); } normalizeWavetable(WAVE_HOLLOW_SQUARE); } if (WAVE_FORMANT_O < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = (float)i / WAVETABLE_SIZE; wavetables[WAVE_FORMANT_O][i] = 0.6f * sinf(2.0f*PI*t) + 0.4f * sinf(2.0f*PI*t*1.8f) + 0.1f * sinf(2.0f*PI*t*4.5f); } normalizeWavetable(WAVE_FORMANT_O); } }
int findAvailableVoice(uint8_t noteVal) { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { if (voices[i].is_playing && voices[i].note == noteVal) { return i; } } for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { if (!voices[i].is_playing && voices[i].env_stage == ENV_IDLE) { return i; } } int best_candidate_release = -1; float min_level_release = 2.0f; for (int i = 0; i < NUM_VOICES_ACTIVE; i++) { if (voices[i].is_playing && voices[i].env_stage == ENV_RELEASE && voices[i].env_level < min_level_release) { min_level_release = voices[i].env_level; best_candidate_release = i; } } if (best_candidate_release != -1) { return best_candidate_release; } float min_level_overall = 2.0f; int best_candidate_overall = 0;  for (int i = 0; i < NUM_VOICES_ACTIVE; i++) { if (voices[i].env_stage != ENV_ATTACK) { if (voices[i].env_level < min_level_overall) { min_level_overall = voices[i].env_level; best_candidate_overall = i; } } } return best_candidate_overall; }

void readAllDirectPotis() { 
  const int num_adc_samples = 3; long adc_sum; const int adc_delay_us = 5;
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_ATTACK_PIN); delayMicroseconds(adc_delay_us); } raw_pot_attack = adc_sum / num_adc_samples; global_attack_time_ms = map(raw_pot_attack, 0, 4095, 1, 2000);
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_DECAY_PIN); delayMicroseconds(adc_delay_us); } raw_pot_decay = adc_sum / num_adc_samples; global_decay_time_ms = map(raw_pot_decay, 0, 4095, 1, 2000);
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_SUSTAIN_PIN); delayMicroseconds(adc_delay_us); } raw_pot_sustain = adc_sum / num_adc_samples; global_sustain_level = (float)raw_pot_sustain / 4095.0f;
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_RELEASE_PIN); delayMicroseconds(adc_delay_us); } raw_pot_release = adc_sum / num_adc_samples; global_release_time_ms = map(raw_pot_release, 0, 4095, 1, 3000);
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_FILTER_CUTOFF_PIN); delayMicroseconds(adc_delay_us); } raw_pot_filter_cutoff = adc_sum / num_adc_samples; float cutoff_norm = (float)raw_pot_filter_cutoff / 4095.0f; global_filter_cutoff_hz_target = 20.0f + (cutoff_norm * cutoff_norm * ( (float)SAMPLE_RATE / 2.5f - 20.0f) ); if (global_filter_cutoff_hz_target > (float)SAMPLE_RATE / 2.2f) global_filter_cutoff_hz_target = (float)SAMPLE_RATE / 2.2f; if (global_filter_cutoff_hz_target < 20.0f) global_filter_cutoff_hz_target = 20.0f; smoothed_filter_cutoff_hz = smoothed_filter_cutoff_hz * (1.0f - CUTOFF_SMOOTHING_FACTOR) + global_filter_cutoff_hz_target * CUTOFF_SMOOTHING_FACTOR;
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_FILTER_RESO_PIN); delayMicroseconds(adc_delay_us); } raw_pot_filter_reso = adc_sum / num_adc_samples; global_filter_resonance_target = map(raw_pot_filter_reso, 0, 4095, 70, 600) / 100.0f; if (global_filter_resonance_target < 0.707f) global_filter_resonance_target = 0.707f; if (global_filter_resonance_target > 6.0f) global_filter_resonance_target = 6.0f; smoothed_filter_resonance = smoothed_filter_resonance * (1.0f - RESO_SMOOTHING_FACTOR) + global_filter_resonance_target * RESO_SMOOTHING_FACTOR;
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_RATE_PIN); delayMicroseconds(adc_delay_us); } raw_pot_lfo_rate = adc_sum / num_adc_samples; if (raw_pot_lfo_rate < LFO_RATE_POTI_DEADZONE) { global_lfo_rate_hz_target = 0.0f; smoothed_lfo_rate_hz = 0.0f; } else { float mapped_rate = (float)(raw_pot_lfo_rate - LFO_RATE_POTI_DEADZONE) / (4095.0f - LFO_RATE_POTI_DEADZONE); if (mapped_rate < 0.0f) mapped_rate = 0.0f; if (mapped_rate > 1.0f) mapped_rate = 1.0f; global_lfo_rate_hz_target = mapped_rate * 10.0f; smoothed_lfo_rate_hz = smoothed_lfo_rate_hz * (1.0f - LFO_RATE_SMOOTHING_FACTOR) + global_lfo_rate_hz_target * LFO_RATE_SMOOTHING_FACTOR;} if (global_lfo_rate_hz_target < 0.001f) { smoothed_lfo_rate_hz = 0.0f;}
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_PITCH_DEPTH_PIN); delayMicroseconds(adc_delay_us); } raw_pot_lfo_pitch_depth_poti_raw = adc_sum / num_adc_samples; if (raw_pot_lfo_pitch_depth_poti_raw < LFO_PITCH_DEPTH_POTI_DEADZONE) { global_lfo_pitch_depth_base_poti = 0.0f; } else { float mapped_value = (float)(raw_pot_lfo_pitch_depth_poti_raw - LFO_PITCH_DEPTH_POTI_DEADZONE) / (4095.0f - LFO_PITCH_DEPTH_POTI_DEADZONE); if (mapped_value < 0.0f) mapped_value = 0.0f; if (mapped_value > 1.0f) mapped_value = 1.0f;  global_lfo_pitch_depth_base_poti = mapped_value * 1.0f;  }
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_MOD_WHEEL_PIN); delayMicroseconds(adc_delay_us); } raw_pot_mod_wheel = adc_sum / num_adc_samples; if (raw_pot_mod_wheel < MOD_WHEEL_POTI_DEADZONE) { global_mod_wheel_value = 0.0f; } else { float mapped_mw = (float)(raw_pot_mod_wheel - MOD_WHEEL_POTI_DEADZONE) / (4095.0f - MOD_WHEEL_POTI_DEADZONE); if (mapped_mw < 0.0f) mapped_mw = 0.0f; if (mapped_mw > 1.0f) mapped_mw = 1.0f; global_mod_wheel_value = mapped_mw;  }
  global_lfo_pitch_depth = global_lfo_pitch_depth_base_poti * global_mod_wheel_value; 
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_FILTER_DEPTH_PIN); delayMicroseconds(adc_delay_us); } raw_pot_lfo_filter_depth_raw = adc_sum / num_adc_samples; if (raw_pot_lfo_filter_depth_raw < LFO_FILTER_DEPTH_POTI_DEADZONE) { global_lfo_filter_depth = 0.0f; } else { float mapped_value = (float)(raw_pot_lfo_filter_depth_raw - LFO_FILTER_DEPTH_POTI_DEADZONE) / (4095.0f - LFO_FILTER_DEPTH_POTI_DEADZONE); if (mapped_value < 0.0f) mapped_value = 0.0f; if (mapped_value > 1.0f) mapped_value = 1.0f; global_lfo_filter_depth = mapped_value * 1.0f; }
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_PITCHBEND_PIN); delayMicroseconds(adc_delay_us); } raw_pot_pitchbend = adc_sum / num_adc_samples; int center_adc = 2048; if (abs(raw_pot_pitchbend - center_adc) < PITCHBEND_POTI_DEADZONE_CENTER) { global_pitchbend_factor = 1.0f; } else { float bend_norm; if (raw_pot_pitchbend < center_adc) { bend_norm = (float)(raw_pot_pitchbend - (center_adc - PITCHBEND_POTI_DEADZONE_CENTER)) / (float)(center_adc - PITCHBEND_POTI_DEADZONE_CENTER); bend_norm = -1.0f * (1.0f - bend_norm); } else { bend_norm = (float)(raw_pot_pitchbend - (center_adc + PITCHBEND_POTI_DEADZONE_CENTER)) / (float)((4095 - (center_adc + PITCHBEND_POTI_DEADZONE_CENTER))); } if (bend_norm < -1.0f) bend_norm = -1.0f; if (bend_norm > 1.0f) bend_norm = 1.0f; global_pitchbend_factor = powf(2.0f, (bend_norm * MAX_PITCHBEND_SEMITONES) / 12.0f); }
  
  // Master Volume mit Deadzone
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_MASTER_VOLUME_PIN); delayMicroseconds(adc_delay_us); } 
  raw_pot_master_volume = adc_sum / num_adc_samples;
  if (raw_pot_master_volume < MASTER_VOLUME_POTI_DEADZONE) {
      global_master_volume = 0.0f;
  } else {
      // Optional: Mapping anpassen, um den "verlorenen" Deadzone-Bereich zu kompensieren
      // global_master_volume = (float)(raw_pot_master_volume - MASTER_VOLUME_POTI_DEADZONE) / (4095.0f - MASTER_VOLUME_POTI_DEADZONE);
      // Einfache Skalierung:
      global_master_volume = (float)raw_pot_master_volume / 4095.0f; 
  }
  if (global_master_volume > 1.0f) global_master_volume = 1.0f; // Sicherstellen
}

void processWaveformSwitch() { 
    if (digitalRead(WAVE_SWITCH_PIN) == LOW) { 
        if (millis() - last_wave_switch_press_time > WAVE_SWITCH_DEBOUNCE_TIME) { 
            if (xSemaphoreTake(voicesMutex, (TickType_t)10) == pdTRUE) {
                global_osc_wave_select_idx = (global_osc_wave_select_idx + 1) % NUM_WAVETABLES;
                display_update_needed = true; 
                xSemaphoreGive(voicesMutex);
            }
            last_wave_switch_press_time = millis(); 
        }
    }
}

void processArpControls() {
    // Arp On/Off Taster
    if (digitalRead(ARP_ONOFF_SWITCH_PIN) == LOW) {
        if (millis() - last_arp_onoff_press_time > ARP_ONOFF_DEBOUNCE_TIME) {
            arp_is_on = !arp_is_on;
            if (!arp_is_on && arp_last_played_note != 0) { // Wenn Arp ausgeschaltet wird, letzte Note stoppen
                arpInternalNoteOff(arp_last_played_note);
                arp_last_played_note = 0;
                arp_num_held_notes = 0; // Alle gehaltenen Noten für Arp löschen
            }
            display_update_needed = true;
            last_arp_onoff_press_time = millis();
        }
    }

    // Arp Modus Taster
    if (digitalRead(ARP_MODE_SWITCH_PIN) == LOW) {
        if (millis() - last_arp_mode_press_time > ARP_MODE_DEBOUNCE_TIME) {
            current_arp_mode = (ArpMode)((current_arp_mode + 1) % NUM_ARP_MODES);
            if (current_arp_mode == ARP_OFF) arp_is_on = false; // Wenn Modus "Off" gewählt, Arp ausschalten
            arp_current_note_index = 0; // Index zurücksetzen bei Moduswechsel
            arp_up_down_direction_is_up = true; // Für Up/Down zurücksetzen
            display_update_needed = true;
            last_arp_mode_press_time = millis();
        }
    }
}


void updateDisplay() {
    if (!display_update_needed) return;
    display.clearDisplay();
    display.setTextSize(1); 
    display.setTextColor(SSD1306_WHITE);
    
    // Zeile 1: Wellenform
    display.setCursor(0,0);
    display.print("Wave: ");
    if (global_osc_wave_select_idx < NUM_WAVETABLES) { display.print(wavetable_names[global_osc_wave_select_idx]); } 
    else { display.print("Err"); }

    // Zeile 2: Arpeggiator Status
    display.setCursor(0,10);
    display.print("Arp: ");
    if (arp_is_on) {
        display.print(arp_mode_names[current_arp_mode]);
    } else {
        display.print("Off");
    }

    // Zeile 3: Filter Cutoff
    display.setCursor(0, 20); 
    display.print("Cut: "); display.print((int)smoothed_filter_cutoff_hz);
    
    // Zeile 4: LFO Rate
    display.setCursor(0, 30);
    display.print("LFO: "); display.print(smoothed_lfo_rate_hz, 1); display.print("Hz");

    display.display();
    display_update_needed = false;
}


// --- Arpeggiator MIDI-Funktionen ---
void arpInternalNoteOn(byte note, byte velocity) {
    if (xSemaphoreTake(voicesMutex, (TickType_t)10) == pdTRUE) {
        int voice_idx = findAvailableVoice(note);
        VoiceState& v = voices[voice_idx];
        v.note = note; v.velocity_gain = (float)velocity / 127.0f;
        v.note_on_trigger = true; v.note_off_trigger = false;
        v.current_wave_idx = global_osc_wave_select_idx; 
        v.phase_accumulator = 0.0f; 
        v.env_attack_rate = (global_attack_time_ms > 0.001f) ? (1.0f / (global_attack_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f;
        v.env_decay_rate = (global_decay_time_ms > 0.001f) ? (1.0f / (global_decay_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f;
        v.env_sustain_level_voice = global_sustain_level;
        v.env_release_rate = (global_release_time_ms > 0.001f) ? (1.0f / (global_release_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f;
        xSemaphoreGive(voicesMutex);
    }
}

void arpInternalNoteOff(byte note) {
    if (xSemaphoreTake(voicesMutex, (TickType_t)10) == pdTRUE) {
        for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) {
            if (voices[i].is_playing && voices[i].note == note) {
                voices[i].note_off_trigger = true;
            }
        }
        xSemaphoreGive(voicesMutex);
    }
}


// --- MIDI Callbacks (angepasst für Arpeggiator) ---
void handleNoteOn(byte channel, byte note, byte velocity) {
    bool note_already_held = false;
    for (int i = 0; i < arp_num_held_notes; ++i) {
        if (arp_held_notes[i] == note) {
            note_already_held = true;
            break;
        }
    }

    if (!note_already_held && arp_num_held_notes < ARP_MAX_NOTES) {
        arp_held_notes[arp_num_held_notes] = note;
        arp_num_held_notes = arp_num_held_notes + 1;
        
        // Sortiere Noten für Up/Down Modi
        for (int i = 0; i < arp_num_held_notes - 1; i++) {
            for (int j = i + 1; j < arp_num_held_notes; j++) {
                if (arp_held_notes[i] > arp_held_notes[j]) {
                    uint8_t temp = arp_held_notes[i];
                    arp_held_notes[i] = arp_held_notes[j];
                    arp_held_notes[j] = temp;
                }
            }
        }
    }
    arp_last_velocity = velocity; 

    if (!arp_is_on) { // Wenn Arp aus ist, spiele Note normal
        arpInternalNoteOn(note, velocity);
    } else { // Arp ist an
        if (arp_num_held_notes == 1) { // Erste Note bei aktivem Arp, starte Zyklus neu
            arp_current_note_index = 0;
            // last_arp_step_time = millis() - ARP_STEP_INTERVAL_MS; // Um sofortigen ersten Step zu triggern
        }
        // Der Arpeggiator kümmert sich um das Spielen in processArpeggiator()
    }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
    bool found_and_removed = false;
    int removed_idx = -1;
    for (int i = 0; i < arp_num_held_notes; ++i) {
        if (arp_held_notes[i] == note) {
            found_and_removed = true;
            removed_idx = i;
            for (int j = i; j < arp_num_held_notes - 1; ++j) {
                arp_held_notes[j] = arp_held_notes[j+1];
            }
            arp_num_held_notes = arp_num_held_notes - 1;
            break;
        }
    }

    if (found_and_removed) {
        if (!arp_is_on) { // Wenn Arp aus ist, normale Note Off
            arpInternalNoteOff(note);
        } else { // Arp ist an
            if (arp_last_played_note == note) { // Wenn die losgelassene Note die aktuell spielende Arp-Note war
                 arpInternalNoteOff(arp_last_played_note);
                 arp_last_played_note = 0; // Verhindert doppeltes NoteOff
            }
            if (arp_num_held_notes == 0) { // Keine Noten mehr gehalten, Arp stoppen
                // arp_is_on = false; // Arp nicht automatisch ausschalten, Nutzer macht das per Taster
                if (arp_last_played_note != 0) arpInternalNoteOff(arp_last_played_note);
                arp_last_played_note = 0;
                arp_current_note_index = 0;
            } else { // Arp läuft weiter mit den verbleibenden Noten
                 // Stelle sicher, dass der Index gültig bleibt
                 if (removed_idx < arp_current_note_index) {
                    arp_current_note_index--;
                 }
                 if (arp_current_note_index >= arp_num_held_notes) {
                    arp_current_note_index = 0;
                 }
            }
        }
    }
}

// --- Arpeggiator Processing ---
void processArpeggiator() {
    if (!arp_is_on || arp_num_held_notes == 0) {
        if (arp_last_played_note != 0) { // Sicherstellen, dass die letzte Arp-Note ausgeschaltet wird, wenn Arp stoppt
            arpInternalNoteOff(arp_last_played_note);
            arp_last_played_note = 0;
        }
        return;
    }

    if (millis() - last_arp_step_time >= ARP_STEP_INTERVAL_MS) {
        if (arp_last_played_note != 0) {
            arpInternalNoteOff(arp_last_played_note); 
        }

        if (arp_num_held_notes > 0) { 
            switch (current_arp_mode) {
                case ARP_UP:
                    arp_current_note_index = (arp_current_note_index + 1) % arp_num_held_notes;
                    break;
                case ARP_DOWN:
                    arp_current_note_index--;
                    if (arp_current_note_index < 0) {
                        arp_current_note_index = arp_num_held_notes - 1;
                    }
                    break;
                case ARP_UP_DOWN:
                    if (arp_up_down_direction_is_up) {
                        arp_current_note_index++;
                        if (arp_current_note_index >= arp_num_held_notes) {
                            arp_current_note_index = arp_num_held_notes - 2; // Zur vorletzten Note
                            if (arp_current_note_index < 0) arp_current_note_index = 0; // Falls nur eine Note
                            arp_up_down_direction_is_up = false;
                        }
                    } else {
                        arp_current_note_index--;
                        if (arp_current_note_index < 0) {
                            arp_current_note_index = (arp_num_held_notes > 1) ? 1 : 0; // Zur zweiten Note oder ersten
                            arp_up_down_direction_is_up = true;
                        }
                    }
                    break;
                case ARP_RANDOM:
                    if (arp_num_held_notes > 0) {
                        arp_current_note_index = random(arp_num_held_notes);
                    }
                    break;
                case ARP_OFF: // Sollte hier nicht hinkommen, da !arp_is_on oben prüft
                default:
                    break; 
            }
            if (arp_current_note_index < 0) arp_current_note_index = 0; // Sicherheit
            if (arp_current_note_index >= arp_num_held_notes && arp_num_held_notes > 0) arp_current_note_index = arp_num_held_notes -1;


            arp_last_played_note = arp_held_notes[arp_current_note_index];
            arpInternalNoteOn(arp_last_played_note, arp_last_velocity); 
        } else { 
            arp_last_played_note = 0;
        }
        last_arp_step_time = millis();
    }
}


// -----------------------------------------------------------------------------
// AUDIO PROCESSING 
// -----------------------------------------------------------------------------
inline float processGlobalSVF(float input_sample) { /* ... unverändert ... */ float cutoff_freq_hz = smoothed_filter_cutoff_hz; if (lfo_increment > 0.000001f && global_lfo_filter_depth > 0.001f) { float lfo_val_for_filter = interpolateWavetable(wavetables[0], lfo_phase); float mod_range = cutoff_freq_hz * 0.8f;  cutoff_freq_hz += lfo_val_for_filter * global_lfo_filter_depth * mod_range; if (cutoff_freq_hz < 20.0f) cutoff_freq_hz = 20.0f; if (cutoff_freq_hz > (float)SAMPLE_RATE / 2.1f) cutoff_freq_hz = (float)SAMPLE_RATE / 2.1f; } float resonance_q = smoothed_filter_resonance; if (resonance_q < 0.5f) resonance_q = 0.5f; if (resonance_q > 6.0f) resonance_q = 6.0f; float g = tanf(PI * cutoff_freq_hz / SAMPLE_RATE);  float k = 1.0f / resonance_q; float hp_output = input_sample - global_filter_integrator2_state - k * global_filter_integrator1_state; float current_bp_output = g * hp_output + global_filter_integrator1_state; float current_lp_output = g * current_bp_output + global_filter_integrator2_state; global_filter_integrator1_state = current_bp_output;  global_filter_integrator2_state = current_lp_output;  global_filter_bp_output = current_bp_output; global_filter_lp_output = current_lp_output; if (isnan(global_filter_integrator1_state) || isinf(global_filter_integrator1_state) || isnan(global_filter_integrator2_state) || isinf(global_filter_integrator2_state)) { global_filter_integrator1_state = 0.0f; global_filter_integrator2_state = 0.0f; global_filter_bp_output = 0.0f; global_filter_lp_output = 0.0f;  return 0.0f;  } return global_filter_lp_output;  }
void fillAudioBuffer() { 
  if (smoothed_lfo_rate_hz > 0.0001f) { 
    lfo_increment = (smoothed_lfo_rate_hz * WAVETABLE_SIZE) / SAMPLE_RATE; 
    lfo_phase += lfo_increment * DMA_BUFFER_FRAMES; 
    while (lfo_phase >= WAVETABLE_SIZE) { lfo_phase -= WAVETABLE_SIZE; }
  } else { 
    lfo_increment = 0.0f; 
  }
  float lfo_val_for_pitch = interpolateWavetable(wavetables[0], lfo_phase); 
  VoiceState local_voices[NUM_VOICES_ACTIVE]; 
  if (xSemaphoreTake(voicesMutex, (TickType_t)5) == pdTRUE) { 
      for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { 
          local_voices[i] = voices[i]; 
          if (voices[i].note_on_trigger) {
              local_voices[i].is_playing = true; local_voices[i].env_stage = ENV_ATTACK; local_voices[i].env_level = 0.0f;
              local_voices[i].note = voices[i].note; local_voices[i].velocity_gain = voices[i].velocity_gain;
              local_voices[i].current_wave_idx = voices[i].current_wave_idx; local_voices[i].phase_accumulator = voices[i].phase_accumulator;
              local_voices[i].env_attack_rate = voices[i].env_attack_rate; local_voices[i].env_decay_rate = voices[i].env_decay_rate;
              local_voices[i].env_sustain_level_voice = voices[i].env_sustain_level_voice; local_voices[i].env_release_rate = voices[i].env_release_rate;
              voices[i].note_on_trigger = false; 
          }
          if (voices[i].note_off_trigger) {
              if (local_voices[i].is_playing && local_voices[i].env_stage != ENV_RELEASE) {
                 local_voices[i].env_stage = ENV_RELEASE;
              }
              voices[i].note_off_trigger = false; 
          }
      }
      xSemaphoreGive(voicesMutex);
  } else { 
      for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { 
          local_voices[i].is_playing = false; local_voices[i].env_stage = ENV_IDLE; local_voices[i].env_level = 0.0f;
      }
  }

  for (int frame = 0; frame < DMA_BUFFER_FRAMES; ++frame) {
    float sum_of_voices_before_filter = 0.0f;
    for (int v_idx = 0; v_idx < NUM_VOICES_ACTIVE; ++v_idx) {
      VoiceState& v_local = local_voices[v_idx]; 
      if (!v_local.is_playing && v_local.env_stage == ENV_IDLE) { continue; }
      switch (v_local.env_stage) { 
          case ENV_ATTACK: v_local.env_level += v_local.env_attack_rate; if (v_local.env_level >= 1.0f) { v_local.env_level = 1.0f; v_local.env_stage = ENV_DECAY; } break; 
          case ENV_DECAY: if (v_local.env_level > v_local.env_sustain_level_voice) { v_local.env_level -= v_local.env_decay_rate; if (v_local.env_level <= v_local.env_sustain_level_voice) { v_local.env_level = v_local.env_sustain_level_voice;} } if (v_local.env_level <= v_local.env_sustain_level_voice) { if (v_local.env_sustain_level_voice < 0.001f) { v_local.env_stage = ENV_RELEASE; } else { v_local.env_stage = ENV_SUSTAIN; } } break; 
          case ENV_SUSTAIN: break; 
          case ENV_RELEASE: v_local.env_level -= v_local.env_release_rate; if (v_local.env_level <= 0.0f) { v_local.env_level = 0.0f; v_local.env_stage = ENV_IDLE; v_local.is_playing = false; } break; 
          case ENV_IDLE: default: v_local.env_level = 0.0f; v_local.is_playing = false; continue;  
      }
      if (v_local.env_level < 0.0f) v_local.env_level = 0.0f; 
      
      if (v_local.is_playing && (v_local.env_level > 0.0001f || v_local.env_stage == ENV_ATTACK)) {
        float base_freq = midiNoteToFreq(v_local.note) * global_pitchbend_factor; 
        float pitch_mod_factor = 1.0f;
        if (lfo_increment > 0.000001f && global_lfo_pitch_depth > 0.0001f) { 
           pitch_mod_factor = powf(2.0f, (lfo_val_for_pitch * global_lfo_pitch_depth * 12.0f) / 12.0f); 
        }
        float current_note_freq = base_freq * pitch_mod_factor;
        v_local.phase_increment = (current_note_freq * WAVETABLE_SIZE) / SAMPLE_RATE;
        v_local.phase_accumulator += v_local.phase_increment;
        if (v_local.phase_accumulator >= WAVETABLE_SIZE) { v_local.phase_accumulator -= WAVETABLE_SIZE; }
        float main_osc_output = interpolateWavetable(wavetables[v_local.current_wave_idx], v_local.phase_accumulator);
        float combined_osc_output = main_osc_output; 
        sum_of_voices_before_filter += combined_osc_output * v_local.velocity_gain * v_local.env_level;
      }
    } 
    sum_of_voices_before_filter *= PRE_FILTER_DRIVE_ATTENUATION; 
    float filtered_output = processGlobalSVF(sum_of_voices_before_filter);
    filtered_output *= global_master_volume;
    if (POLYPHONY_DAMPING_FACTOR > 0.01f) { filtered_output /= POLYPHONY_DAMPING_FACTOR; }
    if (filtered_output > 1.0f) filtered_output = 1.0f; else if (filtered_output < -1.0f) filtered_output = -1.0f;
    int16_t sample_out = (int16_t)(filtered_output * MAX_SYSTEM_AMPLITUDE_FLOAT);
    i2s_audio_buffer[frame * NUM_CHANNELS + 0] = sample_out; 
    i2s_audio_buffer[frame * NUM_CHANNELS + 1] = sample_out; 
  } 

  if (xSemaphoreTake(voicesMutex, (TickType_t)5) == pdTRUE) { 
      for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { 
          voices[i].is_playing = local_voices[i].is_playing; 
          voices[i].env_stage = local_voices[i].env_stage; 
          voices[i].env_level = local_voices[i].env_level; 
          voices[i].phase_accumulator = local_voices[i].phase_accumulator; 
      } 
      xSemaphoreGive(voicesMutex); 
  }
  size_t bytes_written; i2s_channel_write(i2s_tx_chan, i2s_audio_buffer, I2S_WRITE_BUFFER_SIZE_BYTES, &bytes_written, portMAX_DELAY);
}

// -----------------------------------------------------------------------------
// FREERTOS TASKS
// -----------------------------------------------------------------------------
void audioMainTask(void *pvParameters) { Serial.println("Audio Task started on Core 1."); while (true) { fillAudioBuffer(); } }
void controlMainTask(void *pvParameters) {
  Serial.println("Control Task started on Core 0.");
  unsigned long last_pot_read_control_task = 0;
  unsigned long last_debug_print_time = 0; 
  unsigned long last_display_update_time = 0;

  while (true) {
    MIDI.read(); 
    processWaveformSwitch(); 
    processArpControls(); // << ARP TASTER VERARBEITEN >>
    processArpeggiator(); 
    
    unsigned long current_time_ms = millis();

    if (current_time_ms - last_pot_read_control_task >= DIRECT_POT_READ_INTERVAL_MS) {
      readAllDirectPotis(); 
      last_pot_read_control_task = current_time_ms;
      display_update_needed = true; // Poti-Änderungen sollen Display aktualisieren
      
      // Debug-Ausgaben können hier bleiben oder entfernt werden
      /*
      if (current_time_ms - last_debug_print_time > 250) { 
          Serial.print("Wave: "); Serial.print(wavetable_names[global_osc_wave_select_idx]);
          // ...
          last_debug_print_time = current_time_ms;
      }
      */
    }

    if (display_update_needed && (current_time_ms - last_display_update_time > 50)) { 
        updateDisplay();
        last_display_update_time = current_time_ms;
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // Sehr kurze Verzögerung für reaktionsschnelle Taster/Arp
  }
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); 
  Serial.println("\nESP32-S3 Poly Synth - (V33 - Master Vol Fix, Erweiterter Arp)"); 

  Wire.begin(); 
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed")); 
  } else {
    Serial.println(F("SSD1306 Initialized"));
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); display.println("Synth V33 ARP"); 
    display_update_needed = true; // Initiale Anzeige erzwingen
  }

  pinMode(WAVE_SWITCH_PIN, INPUT_PULLUP); 
  pinMode(ARP_ONOFF_SWITCH_PIN, INPUT_PULLUP); // << NEU >>
  pinMode(ARP_MODE_SWITCH_PIN, INPUT_PULLUP);  // << NEU >>

  pinMode(POT_ATTACK_PIN, INPUT); pinMode(POT_DECAY_PIN, INPUT); pinMode(POT_SUSTAIN_PIN, INPUT); pinMode(POT_RELEASE_PIN, INPUT);
  pinMode(POT_FILTER_CUTOFF_PIN, INPUT); pinMode(POT_FILTER_RESO_PIN, INPUT); pinMode(POT_LFO_RATE_PIN, INPUT); 
  pinMode(POT_LFO_PITCH_DEPTH_PIN, INPUT); pinMode(POT_LFO_FILTER_DEPTH_PIN, INPUT); 
  pinMode(POT_MASTER_VOLUME_PIN, INPUT); 
  pinMode(POT_PITCHBEND_PIN, INPUT);   
  pinMode(POT_MOD_WHEEL_PIN, INPUT);   
  Serial.println("Poti & Switch Pins initialized.");
  
  generateAllSynthWavetables(); Serial.println("Wavetables generated.");
  voicesMutex = xSemaphoreCreateMutex(); if (voicesMutex == NULL) { Serial.println("FATAL: Mutex creation failed!"); while (1) vTaskDelay(1000); } 
  for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { voices[i].note_on_trigger = false; voices[i].note_off_trigger = false; voices[i].is_playing = false; voices[i].env_stage = ENV_IDLE; voices[i].env_level = 0.0f; voices[i].phase_accumulator = 0.0f; voices[i].current_wave_idx = 0;  }
  global_filter_integrator1_state = 0.0f; global_filter_integrator2_state = 0.0f; global_filter_lp_output = 0.0f; global_filter_bp_output = 0.0f;
  Serial.println("Global resources & Voices initialized.");
  Serial2.begin(MIDI_BAUD_RATE, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN_FOR_SERIAL2);
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.begin(MIDI_LISTEN_CHANNEL); Serial.println("MIDI Initialized on Serial2.");
  i2s_chan_config_t chan_cfg = { .id = I2S_PORT_NUM, .role = I2S_ROLE_MASTER, .dma_desc_num = DMA_BUFFER_COUNT, .dma_frame_num = DMA_BUFFER_FRAMES, .auto_clear = true, };
  esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL); if (err != ESP_OK) { Serial.printf("I2S: Channel creation failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); }
  i2s_std_config_t std_cfg = { .clk_cfg = { .sample_rate_hz = SAMPLE_RATE, .clk_src = I2S_CLK_SRC_APLL, .mclk_multiple = I2S_MCLK_MULTIPLE_256 }, .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), .gpio_cfg = { .mclk = I2S_MCLK_PIN, .bclk = I2S_BCLK_PIN, .ws = I2S_LRCK_PIN, .dout = I2S_DOUT_PIN, .din = GPIO_NUM_NC, .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false, } } };
  err = i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg); if (err != ESP_OK) { Serial.printf("I2S: Standard mode init failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); }
  err = i2s_channel_enable(i2s_tx_chan); if (err != ESP_OK) { Serial.printf("I2S: Channel enable failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); }
  Serial.println("I2S Initialized and Enabled (SR: " + String(SAMPLE_RATE) + " Hz).");
  xTaskCreatePinnedToCore( audioMainTask, "AudioMain", 16384, NULL, configMAX_PRIORITIES - 1, &audioMainTaskHandle, 1 );
  xTaskCreatePinnedToCore( controlMainTask, "ControlMain", 4096 + 1024, NULL, 5, &controlMainTaskHandle, 0 ); // Stack für Control Task leicht erhöht für Arp/Display
  Serial.println("--- Setup Complete. Tasks Running. ---"); 
}
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }

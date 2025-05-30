// ESP32-S3 - 6-VOICE POLYPHONIC SYNTH - 1 OSC/Voice (Wave Sel), ADSR, VCF, LFO - Direct Potis
// FreeRTOS Zwei-Task-Architektur. (KORRIGIERT V20 - SR22050, LFO Debug, ModWheel Prep)

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

#define POT_OSC_WAVE_PIN    GPIO_NUM_39 
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
#define POT_MOD_WHEEL_PIN       GPIO_NUM_36 // << NEUES POTI FÜR MODWHEEL >>

#define NUM_VOICES_ACTIVE   6
#define NUM_WAVETABLES      4
#define WAVETABLE_SIZE      256
const float POLYPHONY_DAMPING_FACTOR = 4.0f; 
const float PRE_FILTER_DRIVE_ATTENUATION = 0.5f;

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

volatile int global_osc_wave_select_idx = 0;
volatile float global_attack_time_ms = 50.0f;
volatile float global_decay_time_ms = 200.0f;
volatile float global_sustain_level = 0.7f;
volatile float global_release_time_ms = 300.0f;
volatile float global_master_volume = 0.7f;

volatile float global_filter_cutoff_hz_target = 10000.0f;
volatile float smoothed_filter_cutoff_hz = 10000.0f;
const float CUTOFF_SMOOTHING_FACTOR = 0.05f;

volatile float global_filter_resonance_target = 0.707f;
volatile float smoothed_filter_resonance = 0.707f;
const float RESO_SMOOTHING_FACTOR = 0.08f;

volatile float global_lfo_rate_hz_target = 0.0f; // Startwert auf 0
volatile float smoothed_lfo_rate_hz = 0.0f;
const float LFO_RATE_SMOOTHING_FACTOR = 0.1f;

// Für Mod Wheel Logik
volatile float global_lfo_pitch_depth_base_poti = 0.0f; // Gesteuert vom LFO Pitch Depth Poti
volatile float global_mod_wheel_value = 0.0f;         // Gesteuert vom Mod Wheel Poti
volatile float global_lfo_pitch_depth = 0.0f;         // Kombinierter Wert für Audio Engine

volatile float global_lfo_filter_depth = 0.0f;

float lfo_phase = 0.0f; float lfo_increment = 0.0f; 

float global_filter_integrator1_state = 0.0f; 
float global_filter_integrator2_state = 0.0f; 
float global_filter_lp_output = 0.0f; 
float global_filter_bp_output = 0.0f; 

int raw_pot_osc_wave, raw_pot_attack, raw_pot_decay, raw_pot_sustain, raw_pot_release;
int raw_pot_filter_cutoff, raw_pot_filter_reso;
int raw_pot_lfo_rate, raw_pot_lfo_pitch_depth_poti_raw, raw_pot_lfo_filter_depth_raw; 
int raw_pot_master_volume, raw_pot_mod_wheel; // << NEU für Modwheel Rohwert >>

const int LFO_PITCH_DEPTH_POTI_DEADZONE = 150; // << ERHÖHTER TESTWERT >>
const int LFO_RATE_POTI_DEADZONE = 50;
const int MOD_WHEEL_POTI_DEADZONE = 50;

// -----------------------------------------------------------------------------
// HILFSFUNKTIONEN (generateAllSynthWavetables, midiNoteToFreq, interpolateWavetable bleiben gleich)
// -----------------------------------------------------------------------------
float midiNoteToFreq(uint8_t noteVal) { return 440.0f * powf(2.0f, (noteVal - 69.0f) / 12.0f); }
float interpolateWavetable(float table[WAVETABLE_SIZE], float index) { int i0 = (int)floorf(index); int i1 = (i0 + 1); float fraction = index - i0; i0 %= WAVETABLE_SIZE; if (i0 < 0) i0 += WAVETABLE_SIZE; i1 %= WAVETABLE_SIZE; if (i1 < 0) i1 += WAVETABLE_SIZE; return table[i0] * (1.0f - fraction) + table[i1] * fraction; }
void generateAllSynthWavetables() { for (int w = 0; w < NUM_WAVETABLES; ++w) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[w][i] = 0.0f; } } for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[0][i] = sinf(2.0f * PI * (float)i / WAVETABLE_SIZE); } for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[1][i] = 1.0f - 2.0f * (float)i / WAVETABLE_SIZE; } for (int k = 1; k <= 15; k += 2) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[2][i] += (1.0f / k) * sinf(2.0f * PI * k * (float)i / WAVETABLE_SIZE); } } float max_val_sq = 0.0f; for (int i = 0; i < WAVETABLE_SIZE; ++i) { if (fabsf(wavetables[2][i]) > max_val_sq) { max_val_sq = fabsf(wavetables[2][i]); } } if (max_val_sq > 0.0001f) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[2][i] /= max_val_sq; } } for (int i = 0; i < WAVETABLE_SIZE; ++i) { float val = 2.0f * (float)i / WAVETABLE_SIZE; if (val > 1.0f) { val = 2.0f - val; } wavetables[3][i] = 2.0f * val - 1.0f; } }

void readAllDirectPotis() {
  const int num_adc_samples = 3; long adc_sum; const int adc_delay_us = 5;

  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_OSC_WAVE_PIN); delayMicroseconds(adc_delay_us); } raw_pot_osc_wave = adc_sum / num_adc_samples; global_osc_wave_select_idx = map(raw_pot_osc_wave, 0, 4095, 0, NUM_WAVETABLES - 1);
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_ATTACK_PIN); delayMicroseconds(adc_delay_us); } raw_pot_attack = adc_sum / num_adc_samples; global_attack_time_ms = map(raw_pot_attack, 0, 4095, 1, 2000);
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_DECAY_PIN); delayMicroseconds(adc_delay_us); } raw_pot_decay = adc_sum / num_adc_samples; global_decay_time_ms = map(raw_pot_decay, 0, 4095, 1, 2000);
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_SUSTAIN_PIN); delayMicroseconds(adc_delay_us); } raw_pot_sustain = adc_sum / num_adc_samples; global_sustain_level = (float)raw_pot_sustain / 4095.0f;
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_RELEASE_PIN); delayMicroseconds(adc_delay_us); } raw_pot_release = adc_sum / num_adc_samples; global_release_time_ms = map(raw_pot_release, 0, 4095, 1, 3000);
  
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_FILTER_CUTOFF_PIN); delayMicroseconds(adc_delay_us); } raw_pot_filter_cutoff = adc_sum / num_adc_samples; float cutoff_norm = (float)raw_pot_filter_cutoff / 4095.0f; 
  global_filter_cutoff_hz_target = 20.0f + (cutoff_norm * cutoff_norm * ( (float)SAMPLE_RATE / 2.5f - 20.0f) ); 
  if (global_filter_cutoff_hz_target > (float)SAMPLE_RATE / 2.2f) global_filter_cutoff_hz_target = (float)SAMPLE_RATE / 2.2f; 
  if (global_filter_cutoff_hz_target < 20.0f) global_filter_cutoff_hz_target = 20.0f; 
  smoothed_filter_cutoff_hz = smoothed_filter_cutoff_hz * (1.0f - CUTOFF_SMOOTHING_FACTOR) + global_filter_cutoff_hz_target * CUTOFF_SMOOTHING_FACTOR;
  
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_FILTER_RESO_PIN); delayMicroseconds(adc_delay_us); } raw_pot_filter_reso = adc_sum / num_adc_samples; 
  global_filter_resonance_target = map(raw_pot_filter_reso, 0, 4095, 70, 400) / 100.0f; 
  if (global_filter_resonance_target < 0.707f) global_filter_resonance_target = 0.707f; 
  if (global_filter_resonance_target > 15.0f) global_filter_resonance_target = 15.0f; 
  smoothed_filter_resonance = smoothed_filter_resonance * (1.0f - RESO_SMOOTHING_FACTOR) + global_filter_resonance_target * RESO_SMOOTHING_FACTOR;
  
  // LFO Rate Poti mit Deadzone
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_RATE_PIN); delayMicroseconds(adc_delay_us); } 
  raw_pot_lfo_rate = adc_sum / num_adc_samples;
  if (raw_pot_lfo_rate < LFO_RATE_POTI_DEADZONE) {
      global_lfo_rate_hz_target = 0.0f;
  } else {
      float mapped_rate = (float)(raw_pot_lfo_rate - LFO_RATE_POTI_DEADZONE) / (4095.0f - LFO_RATE_POTI_DEADZONE);
      if (mapped_rate < 0.0f) mapped_rate = 0.0f;
      if (mapped_rate > 1.0f) mapped_rate = 1.0f;
      global_lfo_rate_hz_target = mapped_rate * 10.0f; // 0 bis 10 Hz
  }
  smoothed_lfo_rate_hz = smoothed_lfo_rate_hz * (1.0f - LFO_RATE_SMOOTHING_FACTOR) + global_lfo_rate_hz_target * LFO_RATE_SMOOTHING_FACTOR;

  // LFO Pitch Depth Poti (setzt die Basis-Tiefe)
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_PITCH_DEPTH_PIN); delayMicroseconds(adc_delay_us); } 
  raw_pot_lfo_pitch_depth_poti_raw = adc_sum / num_adc_samples; // Eigene Rohwertvariable
  if (raw_pot_lfo_pitch_depth_poti_raw < LFO_PITCH_DEPTH_POTI_DEADZONE) {
      global_lfo_pitch_depth_base_poti = 0.0f;
  } else {
      float mapped_value = (float)(raw_pot_lfo_pitch_depth_poti_raw - LFO_PITCH_DEPTH_POTI_DEADZONE) / (4095.0f - LFO_PITCH_DEPTH_POTI_DEADZONE);
      if (mapped_value < 0.0f) mapped_value = 0.0f;
      if (mapped_value > 1.0f) mapped_value = 1.0f; 
      global_lfo_pitch_depth_base_poti = mapped_value * 1.0f; 
  }

  // Mod Wheel Poti (GPIO36)
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_MOD_WHEEL_PIN); delayMicroseconds(adc_delay_us); }
  raw_pot_mod_wheel = adc_sum / num_adc_samples; 
  if (raw_pot_mod_wheel < MOD_WHEEL_POTI_DEADZONE) {
      global_mod_wheel_value = 0.0f;
  } else {
      float mapped_mw = (float)(raw_pot_mod_wheel - MOD_WHEEL_POTI_DEADZONE) / (4095.0f - MOD_WHEEL_POTI_DEADZONE);
      if (mapped_mw < 0.0f) mapped_mw = 0.0f;
      if (mapped_mw > 1.0f) mapped_mw = 1.0f;
      global_mod_wheel_value = mapped_mw; 
  }

  // Kombiniere Poti-Basis-Tiefe mit Mod Wheel Wert für die effektive LFO-Pitch-Tiefe
  global_lfo_pitch_depth = global_lfo_pitch_depth_base_poti * global_mod_wheel_value;
  
  // LFO Filter Depth (optional mit Deadzone und/oder Mod Wheel Einfluss)
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_LFO_FILTER_DEPTH_PIN); delayMicroseconds(adc_delay_us); } 
  raw_pot_lfo_filter_depth_raw = adc_sum / num_adc_samples;
  global_lfo_filter_depth = (float)raw_pot_lfo_filter_depth_raw / 4095.0f; // Hier noch ohne Deadzone
  
  adc_sum = 0; for (int k = 0; k < num_adc_samples; k++) { adc_sum += analogRead(POT_MASTER_VOLUME_PIN); delayMicroseconds(adc_delay_us); } raw_pot_master_volume = adc_sum / num_adc_samples; global_master_volume = (float)raw_pot_master_volume / 4095.0f;
}

int findAvailableVoice(uint8_t noteVal) { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { if (voices[i].is_playing && voices[i].note == noteVal) { return i; } } for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { if (!voices[i].is_playing && voices[i].env_stage == ENV_IDLE) { return i; } } int best_candidate_release = -1; float min_level_release = 2.0f; for (int i = 0; i < NUM_VOICES_ACTIVE; i++) { if (voices[i].is_playing && voices[i].env_stage == ENV_RELEASE && voices[i].env_level < min_level_release) { min_level_release = voices[i].env_level; best_candidate_release = i; } } if (best_candidate_release != -1) { return best_candidate_release; } float min_level_overall = 2.0f; int best_candidate_overall = 0;  for (int i = 0; i < NUM_VOICES_ACTIVE; i++) { if (voices[i].env_stage != ENV_ATTACK) { if (voices[i].env_level < min_level_overall) { min_level_overall = voices[i].env_level; best_candidate_overall = i; } } } return best_candidate_overall; }

// -----------------------------------------------------------------------------
// MIDI CALLBACKS (bleiben gleich)
// -----------------------------------------------------------------------------
void midiNoteOnHandler(byte channel, byte note, byte velocity) { if (velocity > 0) { if (xSemaphoreTake(voicesMutex, (TickType_t)10) == pdTRUE) { int voice_idx = findAvailableVoice(note); VoiceState& v = voices[voice_idx];  v.note = note; v.velocity_gain = (float)velocity / 127.0f; v.note_on_trigger = true; v.note_off_trigger = false;  v.current_wave_idx = global_osc_wave_select_idx; v.env_attack_rate = (global_attack_time_ms > 0.001f) ? (1.0f / (global_attack_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f; v.env_decay_rate = (global_decay_time_ms > 0.001f) ? (1.0f / (global_decay_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f; v.env_sustain_level_voice = global_sustain_level; v.env_release_rate = (global_release_time_ms > 0.001f) ? (1.0f / (global_release_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f; xSemaphoreGive(voicesMutex); } } else {  midiNoteOffHandler(channel, note, velocity); } }
void midiNoteOffHandler(byte channel, byte note, byte velocity) { if (xSemaphoreTake(voicesMutex, (TickType_t)10) == pdTRUE) { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { if (voices[i].is_playing && voices[i].note == note) { voices[i].note_off_trigger = true; } } xSemaphoreGive(voicesMutex); } }

// -----------------------------------------------------------------------------
// AUDIO PROCESSING (SVF bleibt gleich)
// -----------------------------------------------------------------------------
inline float processGlobalSVF(float input_sample) { float cutoff_freq_hz = smoothed_filter_cutoff_hz; if (global_lfo_filter_depth > 0.001f && lfo_increment > 0.00001f) { float lfo_val_for_filter = interpolateWavetable(wavetables[0], lfo_phase); float mod_range = cutoff_freq_hz * 0.8f;  cutoff_freq_hz += lfo_val_for_filter * global_lfo_filter_depth * mod_range; if (cutoff_freq_hz < 20.0f) cutoff_freq_hz = 20.0f; if (cutoff_freq_hz > (float)SAMPLE_RATE / 2.1f) cutoff_freq_hz = (float)SAMPLE_RATE / 2.1f; } float resonance_q = smoothed_filter_resonance; if (resonance_q < 0.5f) resonance_q = 0.5f; if (resonance_q > 15.0f) resonance_q = 15.0f; float g = tanf(PI * cutoff_freq_hz / SAMPLE_RATE);  float k = 1.0f / resonance_q; float hp_output = input_sample - global_filter_integrator2_state - k * global_filter_integrator1_state; float current_bp_output = g * hp_output + global_filter_integrator1_state; float current_lp_output = g * current_bp_output + global_filter_integrator2_state; global_filter_integrator1_state = current_bp_output;  global_filter_integrator2_state = current_lp_output;  global_filter_bp_output = current_bp_output; global_filter_lp_output = current_lp_output; if (isnan(global_filter_integrator1_state) || isinf(global_filter_integrator1_state) || isnan(global_filter_integrator2_state) || isinf(global_filter_integrator2_state)) { global_filter_integrator1_state = 0.0f; global_filter_integrator2_state = 0.0f; global_filter_bp_output = 0.0f; global_filter_lp_output = 0.0f;  return 0.0f;  } return global_filter_lp_output;  }

void fillAudioBuffer() {
  // LFO Update: Nur wenn Rate > 0 ist, sonst Inkrement und Phase auf 0
  if (smoothed_lfo_rate_hz > 0.001f) { 
    lfo_increment = (smoothed_lfo_rate_hz * WAVETABLE_SIZE) / SAMPLE_RATE;
    lfo_phase += lfo_increment * DMA_BUFFER_FRAMES; 
    while (lfo_phase >= WAVETABLE_SIZE) { lfo_phase -= WAVETABLE_SIZE; }
  } else {
    lfo_increment = 0.0f; 
    // lfo_phase = 0.0f; // Optional: Phase resetten, wenn LFO aus ist
  }
  float lfo_val_for_pitch = interpolateWavetable(wavetables[0], lfo_phase); 

  VoiceState local_voices[NUM_VOICES_ACTIVE];
  if (xSemaphoreTake(voicesMutex, (TickType_t)5) == pdTRUE) { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { local_voices[i] = voices[i];  if (voices[i].note_on_trigger) { local_voices[i].note = voices[i].note; local_voices[i].velocity_gain = voices[i].velocity_gain; local_voices[i].is_playing = true; local_voices[i].env_stage = ENV_ATTACK; local_voices[i].env_level = 0.0f; local_voices[i].phase_accumulator = 0.0f;  local_voices[i].current_wave_idx = voices[i].current_wave_idx;  local_voices[i].env_attack_rate = voices[i].env_attack_rate; local_voices[i].env_decay_rate = voices[i].env_decay_rate; local_voices[i].env_sustain_level_voice = voices[i].env_sustain_level_voice; local_voices[i].env_release_rate = voices[i].env_release_rate; voices[i].note_on_trigger = false; voices[i].note_off_trigger = false;  } else if (voices[i].note_off_trigger) { if (local_voices[i].is_playing && local_voices[i].env_stage != ENV_RELEASE) { local_voices[i].env_stage = ENV_RELEASE; } voices[i].note_off_trigger = false;  } } xSemaphoreGive(voicesMutex);
  } else { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) {  local_voices[i].is_playing = false; local_voices[i].env_stage = ENV_IDLE; local_voices[i].env_level = 0.0f; } }

  for (int frame = 0; frame < DMA_BUFFER_FRAMES; ++frame) {
    float sum_of_voices_before_filter = 0.0f;
    for (int v_idx = 0; v_idx < NUM_VOICES_ACTIVE; ++v_idx) {
      VoiceState& v_local = local_voices[v_idx]; 
      if (!v_local.is_playing && v_local.env_stage == ENV_IDLE) { continue; }
      switch (v_local.env_stage) { case ENV_ATTACK: v_local.env_level += v_local.env_attack_rate; if (v_local.env_level >= 1.0f) { v_local.env_level = 1.0f; v_local.env_stage = ENV_DECAY; } break; case ENV_DECAY: v_local.env_level -= v_local.env_decay_rate; if (v_local.env_level <= v_local.env_sustain_level_voice) { v_local.env_level = v_local.env_sustain_level_voice; if (v_local.env_sustain_level_voice <= 0.001f) { v_local.env_stage = ENV_RELEASE; } else { v_local.env_stage = ENV_SUSTAIN; } } break; case ENV_SUSTAIN: if (v_local.env_sustain_level_voice <= 0.001f && v_local.env_level <=0.001f) { v_local.env_stage = ENV_RELEASE; } break; case ENV_RELEASE: v_local.env_level -= v_local.env_release_rate; if (v_local.env_level <= 0.0f) { v_local.env_level = 0.0f; v_local.env_stage = ENV_IDLE; v_local.is_playing = false; } break; case ENV_IDLE: default: v_local.env_level = 0.0f; v_local.is_playing = false; continue;  }
      if (v_local.env_level < 0.0f) v_local.env_level = 0.0f; 
      if (v_local.is_playing && (v_local.env_level > 0.0001f || v_local.env_stage == ENV_ATTACK)) {
        float base_freq = midiNoteToFreq(v_local.note);
        float pitch_mod_factor = 1.0f;

        // << LFO PITCH MODULATION HIER ZUM TESTEN DEAKTIVIEREN / AKTIVIEREN >>
        // Wenn global_lfo_pitch_depth (kombinierter Wert aus Poti und ModWheel) > 0 UND LFO Rate > 0
        if (global_lfo_pitch_depth > 0.0001f && lfo_increment > 0.00001f) { 
           pitch_mod_factor = powf(2.0f, (lfo_val_for_pitch * global_lfo_pitch_depth * 12.0f) / 12.0f); 
        }
        // Zum Testen, ob das Schweben vom LFO kommt, die obige if-Bedingung auskommentieren
        // und pitch_mod_factor = 1.0f; lassen.

        float current_note_freq = base_freq * pitch_mod_factor;
        v_local.phase_increment = (current_note_freq * WAVETABLE_SIZE) / SAMPLE_RATE;
        v_local.phase_accumulator += v_local.phase_increment;
        if (v_local.phase_accumulator >= WAVETABLE_SIZE) { v_local.phase_accumulator -= WAVETABLE_SIZE; }
        float osc_output = interpolateWavetable(wavetables[v_local.current_wave_idx], v_local.phase_accumulator);
        sum_of_voices_before_filter += osc_output * v_local.velocity_gain * v_local.env_level;
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

  if (xSemaphoreTake(voicesMutex, (TickType_t)5) == pdTRUE) { for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { voices[i].is_playing = local_voices[i].is_playing; voices[i].env_stage = local_voices[i].env_stage; voices[i].env_level = local_voices[i].env_level; voices[i].phase_accumulator = local_voices[i].phase_accumulator; } xSemaphoreGive(voicesMutex); }
  size_t bytes_written; i2s_channel_write(i2s_tx_chan, i2s_audio_buffer, I2S_WRITE_BUFFER_SIZE_BYTES, &bytes_written, portMAX_DELAY);
}

// -----------------------------------------------------------------------------
// FREERTOS TASKS (bleiben gleich)
// -----------------------------------------------------------------------------
void audioMainTask(void *pvParameters) { Serial.println("Audio Task started on Core 1."); while (true) { fillAudioBuffer(); } }
void controlMainTask(void *pvParameters) { Serial.println("Control Task started on Core 0."); unsigned long last_pot_read_control_task = 0; while (true) { MIDI.read();  unsigned long current_time_control_task = millis(); if (current_time_control_task - last_pot_read_control_task >= DIRECT_POT_READ_INTERVAL_MS) { readAllDirectPotis();  last_pot_read_control_task = current_time_control_task; } vTaskDelay(pdMS_TO_TICKS(5));  } }

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  unsigned long setup_start_time = millis();
  while (!Serial && (millis() - setup_start_time < 2000)); 
  Serial.println("\nESP32-S3 Poly Synth - (KORRIGIERT V20 - SR22050, LFO Debug, ModWheel Prep)");

  pinMode(POT_OSC_WAVE_PIN, INPUT); pinMode(POT_ATTACK_PIN, INPUT); pinMode(POT_DECAY_PIN, INPUT); pinMode(POT_SUSTAIN_PIN, INPUT); pinMode(POT_RELEASE_PIN, INPUT);
  pinMode(POT_FILTER_CUTOFF_PIN, INPUT); pinMode(POT_FILTER_RESO_PIN, INPUT); pinMode(POT_LFO_RATE_PIN, INPUT); pinMode(POT_LFO_PITCH_DEPTH_PIN, INPUT);
  pinMode(POT_LFO_FILTER_DEPTH_PIN, INPUT); pinMode(POT_MASTER_VOLUME_PIN, INPUT);
  pinMode(POT_MOD_WHEEL_PIN, INPUT); // << NEUES POTI INITIALISIEREN >>
  Serial.println("Poti Pins (INPUT mode set - Pin-Zuweisungen für ESP32-S3 ADC1 prüfen!).");

  generateAllSynthWavetables(); Serial.println("Wavetables generated.");
  voicesMutex = xSemaphoreCreateMutex(); if (voicesMutex == NULL) { Serial.println("FATAL: Mutex creation failed!"); while (1) vTaskDelay(1000); }

  for (int i = 0; i < NUM_VOICES_ACTIVE; ++i) { voices[i].note_on_trigger = false; voices[i].note_off_trigger = false; voices[i].is_playing = false; voices[i].env_stage = ENV_IDLE; voices[i].env_level = 0.0f; voices[i].phase_accumulator = 0.0f; voices[i].current_wave_idx = 0;  }
  global_filter_integrator1_state = 0.0f; global_filter_integrator2_state = 0.0f; global_filter_lp_output = 0.0f; global_filter_bp_output = 0.0f;
  Serial.println("Global resources & Voices initialized.");

  Serial2.begin(MIDI_BAUD_RATE, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN_FOR_SERIAL2);
  MIDI.setHandleNoteOn(midiNoteOnHandler); MIDI.setHandleNoteOff(midiNoteOffHandler);
  MIDI.begin(MIDI_LISTEN_CHANNEL); Serial.println("MIDI Initialized on Serial2.");

  i2s_chan_config_t chan_cfg = { .id = I2S_PORT_NUM, .role = I2S_ROLE_MASTER, .dma_desc_num = DMA_BUFFER_COUNT, .dma_frame_num = DMA_BUFFER_FRAMES, .auto_clear = true, };
  esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL); if (err != ESP_OK) { Serial.printf("I2S: Channel creation failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); }
  i2s_std_config_t std_cfg = { .clk_cfg = { .sample_rate_hz = SAMPLE_RATE, .clk_src = I2S_CLK_SRC_APLL, .mclk_multiple = I2S_MCLK_MULTIPLE_256 }, .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), .gpio_cfg = { .mclk = I2S_MCLK_PIN, .bclk = I2S_BCLK_PIN, .ws = I2S_LRCK_PIN, .dout = I2S_DOUT_PIN, .din = GPIO_NUM_NC, .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false, } } };
  err = i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg); if (err != ESP_OK) { Serial.printf("I2S: Standard mode init failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); }
  err = i2s_channel_enable(i2s_tx_chan); if (err != ESP_OK) { Serial.printf("I2S: Channel enable failed: %s\n", esp_err_to_name(err)); while (1) vTaskDelay(1000); }
  Serial.println("I2S Initialized and Enabled (SR: " + String(SAMPLE_RATE) + " Hz).");

  xTaskCreatePinnedToCore( audioMainTask, "AudioMain", 16384, NULL, configMAX_PRIORITIES - 1, &audioMainTaskHandle, 1 );
  xTaskCreatePinnedToCore( controlMainTask, "ControlMain", 4096, NULL, 5, &controlMainTaskHandle, 0 );
  Serial.println("--- Setup Complete. Tasks Running. ---");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
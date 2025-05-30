// ESP32-S3 DDS Wavetable Synthesizer - Single Sketch
// I2S mit direktem ESP-IDF Treiber (i2s_std.h) und angepasster Amplitude

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <driver/i2s_std.h> // Direkter ESP-IDF I2S Standard Treiber
#include <MIDI.h>           // MIDI Library von FortySevenEffects
#include <math.h>           // Für sinf, powf, tanf etc.

// -----------------------------------------------------------------------------
// KONSTANTEN & DEFINITIONEN
// -----------------------------------------------------------------------------

// Audio
#define SAMPLE_RATE         44100
#define NUM_CHANNELS        2      // Stereo
#define I2S_PORT_NUM        I2S_NUM_0   // I2S Port 0

// I2S Pins (DEINE AKTUELLE, FUNKTIONIERENDE BELEGUNG)
#define I2S_BCLK_PIN        (GPIO_NUM_27) // Dein BCLK
#define I2S_LRCK_PIN        (GPIO_NUM_26) // Dein LRCK/WS
#define I2S_DOUT_PIN        (GPIO_NUM_25) // Dein DOUT
#define I2S_MCLK_PIN        (GPIO_NUM_NC) // Nicht verwendet für PCM5102A

// DMA Puffer Konfiguration
#define DMA_BUFFER_COUNT    8  // Etwas mehr Puffer für Stabilität
#define DMA_BUFFER_FRAMES   256 // Stereo-Frames pro DMA-Puffer
const int BYTES_PER_FRAME_STEREO = NUM_CHANNELS * (I2S_DATA_BIT_WIDTH_16BIT / 8);
const int I2S_WRITE_BUFFER_SIZE_BYTES = DMA_BUFFER_FRAMES * BYTES_PER_FRAME_STEREO;

// Synthesizer
#define NUM_VOICES          8
#define NUM_WAVETABLES      4
#define WAVETABLE_SIZE      256    // Samples pro Wavetable
#define PI_F                3.14159265358979323846f
#define TWO_PI_F            (2.0f * PI_F)

// << MAXIMALE DIGITALE AMPLITUDE für DEIN System (aus Test ermittelt) >>
const float MAX_SYSTEM_AMPLITUDE_FLOAT = 2000.0f;

// MIDI
#define MIDI_SERIAL         Serial1 // UART für MIDI IN (Passe RX Pin ggf. an)
#define MIDI_CHANNEL        MIDI_CHANNEL_OMNI

// Multiplexer (CD4067) - Pins von dir genannt
#define MUX_SIG_PIN         GPIO_NUM_12 // Analog Eingang für MUX Signal
#define MUX_S0_PIN          GPIO_NUM_33
#define MUX_S1_PIN          GPIO_NUM_32
#define MUX_S2_PIN          GPIO_NUM_13
#define MUX_S3_PIN          GPIO_NUM_34
#define NUM_POTS            15          // 0-14 für den CD4067

// Potentiometer Mapping (Beispiel, passe ggf. an)
#define POT_MASTER_VOLUME     0  // Gesamt Lautstärke
#define POT_WAVETABLE_SELECT  1  // Wählt 1 von 4 Wavetables
#define POT_ATTACK            2
#define POT_DECAY             3
#define POT_SUSTAIN           4
#define POT_RELEASE           5
#define POT_LFO_RATE          6
#define POT_LFO_PITCH_DEPTH   7  // LFO auf Pitch
#define POT_LFO_FILTER_DEPTH  8  // LFO auf Filter Cutoff
#define POT_FILTER_CUTOFF     9
#define POT_FILTER_RESO       10
#define POT_UNUSED_11         11 // Zur freien Verfügung
#define POT_UNUSED_12         12
#define POT_UNUSED_13         13
#define POT_UNUSED_14         14


// -----------------------------------------------------------------------------
// DATENTYPEN
// -----------------------------------------------------------------------------

enum EnvelopeStage {
  ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE
};

struct Voice {
  bool active;
  uint8_t note;
  uint8_t velocity;
  float phase_accumulator;
  float phase_increment;
  int current_wavetable_idx; // Wird durch global_wavetable_select bestimmt

  EnvelopeStage env_stage;
  float env_level;
  float env_attack_rate;
  float env_decay_rate;
  float env_sustain_level;
  float env_release_rate;

  // Filterzustand pro Stimme (wichtig für SVF)
  float f_lp, f_bp, f_hp;
  float f_integrator1, f_integrator2;
};

// -----------------------------------------------------------------------------
// GLOBALE VARIABLEN
// -----------------------------------------------------------------------------

// I2S
i2s_chan_handle_t tx_chan;
int16_t i2s_write_buffer[DMA_BUFFER_FRAMES * NUM_CHANNELS];

// Synth Engine
float wavetables[NUM_WAVETABLES][WAVETABLE_SIZE];
Voice voices[NUM_VOICES];

// Globale Synth Parameter (gesteuert durch Pots)
float global_master_volume = 0.7f; // Startwert Master Volume
float global_attack_time_ms = 50.0f;
float global_decay_time_ms = 200.0f;
float global_sustain_level = 0.7f;
float global_release_time_ms = 300.0f;
int   global_wavetable_select_idx = 0; // Index der global ausgewählten Wavetable

// Globaler LFO
float lfo_phase = 0.0f;
float lfo_increment = 0.0f;
float global_lfo_rate_hz = 1.0f;
float global_lfo_pitch_depth = 0.0f;  // Semitones für Pitch Mod
float global_lfo_filter_depth = 0.0f; // Faktor (0-1) für Filter Cutoff Mod

// Globaler Filter
float global_filter_cutoff_base_hz = 8000.0f; // Basis-Cutoff
float global_filter_resonance = 1.0f;    // Basis-Resonanz

// Multiplexer & Potis
int pot_raw_values[NUM_POTS];
const int MUX_CTRL_PINS[] = {MUX_S0_PIN, MUX_S1_PIN, MUX_S2_PIN, MUX_S3_PIN};
unsigned long last_pot_read_time = 0;
const unsigned long POT_READ_INTERVAL_MS = 20;

// MIDI
MIDI_CREATE_INSTANCE(HardwareSerial, MIDI_SERIAL, MIDI);

// -----------------------------------------------------------------------------
// HILFSFUNKTIONEN
// -----------------------------------------------------------------------------

float midiNoteToFreq(uint8_t note) {
  return 440.0f * powf(2.0f, (note - 69.0f) / 12.0f);
}

// Lineare Interpolation für Wavetables
float interpolate(float* table, float index) {
  int idx0 = (int)floorf(index);
  int idx1 = (idx0 + 1);
  float frac = index - idx0;
  idx0 = idx0 % WAVETABLE_SIZE;
  if (idx0 < 0) idx0 += WAVETABLE_SIZE;
  idx1 = idx1 % WAVETABLE_SIZE;
  if (idx1 < 0) idx1 += WAVETABLE_SIZE;
  return table[idx0] * (1.0f - frac) + table[idx1] * frac;
}

// Erzeugt die Wavetables
void generateWavetables() {
  // 0: Sine
  for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[0][i] = sinf(TWO_PI_F * (float)i / WAVETABLE_SIZE);
  // 1: Sawtooth
  for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[1][i] = 1.0f - 2.0f * (float)i / WAVETABLE_SIZE;
  // 2: Square (mit einfacher Bandlimitierung)
  for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[2][i] = 0.0f;
  for (int k = 1; k <= 9; k += 2) { // Additive Synth: Grundton + ungerade Harmonische
      for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[2][i] += (1.0f/k) * sinf(TWO_PI_F * k * (float)i / WAVETABLE_SIZE);
  }
  float max_sq = 0; for (int i = 0; i < WAVETABLE_SIZE; ++i) if (fabsf(wavetables[2][i]) > max_sq) max_sq = fabsf(wavetables[2][i]);
  if (max_sq > 0.0001f) for (int i = 0; i < WAVETABLE_SIZE; ++i) wavetables[2][i] /= max_sq; // Normalisieren
  // 3: Triangle
  for (int i = 0; i < WAVETABLE_SIZE; ++i) { float v = 2.0f*(float)i/WAVETABLE_SIZE; if(v>1.0f) v=2.0f-v; wavetables[3][i]=2.0f*v-1.0f; }
}

// Liest alle Potentiometer über den Multiplexer
void readPotentiometers() {
  for (int i = 0; i < NUM_POTS; i++) {
    for (int j = 0; j < 4; j++) {
      digitalWrite(MUX_CTRL_PINS[j], (i >> j) & 0x01);
    }
    delayMicroseconds(50); // Kurze Wartezeit für MUX
    pot_raw_values[i] = analogRead(MUX_SIG_PIN);
  }
}

// Aktualisiert globale Synth-Parameter basierend auf Poti-Werten
void updateGlobalParameters() {
  // Master Volume (0.0 bis 1.0)
  global_master_volume = (float)pot_raw_values[POT_MASTER_VOLUME] / 4095.0f;

  // Wavetable Select (0 bis NUM_WAVETABLES-1)
  global_wavetable_select_idx = map(pot_raw_values[POT_WAVETABLE_SELECT], 0, 4095, 0, NUM_WAVETABLES - 1);
  if (global_wavetable_select_idx >= NUM_WAVETABLES) global_wavetable_select_idx = NUM_WAVETABLES -1;
  if (global_wavetable_select_idx < 0) global_wavetable_select_idx = 0;

  // ADSR Zeiten (1ms bis 5000ms) - lineare Map, könnte man logarithmisch machen
  global_attack_time_ms = map(pot_raw_values[POT_ATTACK], 0, 4095, 1, 5000);
  global_decay_time_ms = map(pot_raw_values[POT_DECAY], 0, 4095, 1, 5000);
  global_sustain_level = (float)pot_raw_values[POT_SUSTAIN] / 4095.0f;
  global_release_time_ms = map(pot_raw_values[POT_RELEASE], 0, 4095, 1, 5000);

  // LFO (Rate: 0.1Hz bis 20Hz, Tiefe Pitch: 0-2 Halbtöne, Tiefe Filter: 0-1 Faktor)
  global_lfo_rate_hz = map(pot_raw_values[POT_LFO_RATE], 0, 4095, 10, 2000) / 100.0f; // 0.1 to 20.0 Hz
  global_lfo_pitch_depth = ((float)pot_raw_values[POT_LFO_PITCH_DEPTH] / 4095.0f) * 2.0f; // 0 to 2 semitones
  global_lfo_filter_depth = (float)pot_raw_values[POT_LFO_FILTER_DEPTH] / 4095.0f; // 0 to 1

  // Filter (Cutoff: 20Hz bis ~18kHz logarithmisch, Reso: 0.7 bis 10)
  float pot_cutoff_norm = (float)pot_raw_values[POT_FILTER_CUTOFF] / 4095.0f;
  global_filter_cutoff_base_hz = 20.0f * powf(900.0f, pot_cutoff_norm); // Log Skala: 20Hz * (18000/20)^norm -> 20 * 900^norm
  if(global_filter_cutoff_base_hz > 18000.0f) global_filter_cutoff_base_hz = 18000.0f;
  if(global_filter_cutoff_base_hz < 20.0f) global_filter_cutoff_base_hz = 20.0f;
  global_filter_resonance = map(pot_raw_values[POT_FILTER_RESO], 0, 4095, 70, 1000) / 100.0f; // 0.7 to 10.0
  if (global_filter_resonance < 0.707f) global_filter_resonance = 0.707f; // Mindest-Q

  // LFO-Increment berechnen
  lfo_increment = (global_lfo_rate_hz * WAVETABLE_SIZE) / SAMPLE_RATE;
}

// Findet eine freie Stimme oder stiehlt eine (einfache Logik)
int findFreeVoice(uint8_t note) {
  // Priorität 1: Finde eine inaktive Stimme
  for (int i = 0; i < NUM_VOICES; ++i) {
    if (!voices[i].active && voices[i].env_stage == ENV_IDLE) return i;
  }
  // Priorität 2: Finde die gleiche Note (für Retrigger)
  for (int i = 0; i < NUM_VOICES; ++i) {
      if (voices[i].active && voices[i].note == note) return i;
  }
  // Priorität 3: Finde eine Stimme in der Release-Phase mit niedrigstem Pegel
  int quietest_release_idx = -1;
  float min_release_level = 1.1f; // Höher als max Pegel
  for(int i=0; i<NUM_VOICES; ++i){
      if(voices[i].active && voices[i].env_stage == ENV_RELEASE && voices[i].env_level < min_release_level){
          min_release_level = voices[i].env_level;
          quietest_release_idx = i;
      }
  }
  if(quietest_release_idx != -1) return quietest_release_idx;

  // Notfall: Nimm einfach die erste Stimme (kann hart klingen)
  return 0;
}

// -----------------------------------------------------------------------------
// MIDI CALLBACKS
// -----------------------------------------------------------------------------

void handleNoteOn(byte channel, byte note, byte velocity) {
  if (velocity == 0) { handleNoteOff(channel, note, velocity); return; }

  int voice_idx = findFreeVoice(note);
  Voice& v = voices[voice_idx];

  v.active = true;
  v.note = note;
  v.velocity = velocity;
  v.phase_accumulator = 0.0f; // Reset Phase bei Note On
  v.current_wavetable_idx = global_wavetable_select_idx; // Globale Wavetable

  // Setze Envelope Raten basierend auf aktuellen globalen Zeiten
  v.env_attack_rate = (global_attack_time_ms > 0) ? (1.0f / (global_attack_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f; // Schneller als 1 Sample wenn Zeit=0
  v.env_decay_rate = (global_decay_time_ms > 0) ? (1.0f / (global_decay_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f;
  v.env_sustain_level = global_sustain_level;
  v.env_release_rate = (global_release_time_ms > 0) ? (1.0f / (global_release_time_ms * 0.001f * SAMPLE_RATE)) : 1.0f;

  // Starte Envelope neu
  if (v.env_stage == ENV_IDLE || v.env_stage == ENV_RELEASE) {
    v.env_level = 0.0f; // Starte von 0, wenn aus oder in Release
  }
  // Ansonsten (Retrigger während Attack/Decay/Sustain) startet die Attack-Phase vom aktuellen Level
  v.env_stage = ENV_ATTACK;

  // Reset Filterzustand für die neue Note
  v.f_lp = 0.0f; v.f_bp = 0.0f; v.f_hp = 0.0f;
  v.f_integrator1 = 0.0f; v.f_integrator2 = 0.0f;
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  for (int i = 0; i < NUM_VOICES; ++i) {
    // Löse Release nur aus, wenn die Stimme aktiv ist UND die Note übereinstimmt
    if (voices[i].active && voices[i].note == note) {
      // Nicht sofort inaktiv setzen, nur Release starten
      if (voices[i].env_stage != ENV_IDLE) { // Nur wenn nicht schon idle
           voices[i].env_stage = ENV_RELEASE;
      }
    }
  }
}

void handleControlChange(byte channel, byte control, byte value) {
  // Hier könnten CCs für Echtzeitsteuerung implementiert werden
  // Serial.printf("CC Ch%d, Ctrl %d, Val %d\n", channel, control, value);
}

// -----------------------------------------------------------------------------
// AUDIO PROCESSING
// -----------------------------------------------------------------------------

// State Variable Filter (pro Sample)
inline float processSVF(Voice& v, float input_sample, float cutoff_hz, float resonance_q) {
    // Frequenzabhängiger Koeffizient 'g' (tan approximation)
    float g = tanf(PI_F * cutoff_hz / SAMPLE_RATE);
    // Dämpfungsfaktor 'k' (abhängig von Resonanz/Q)
    float k = 1.0f / resonance_q;

    // Berechne Tiefpass (LP), Bandpass (BP) und Hochpass (HP) Ausgänge
    // Verwende die Zustandsvariablen v.f_integrator1 (BP) und v.f_integrator2 (LP)
    float hp = (input_sample - v.f_integrator1 * k - v.f_integrator2) / (1.0f + g * k + g * g);
    float bp = g * hp + v.f_integrator1;
    float lp = g * bp + v.f_integrator2;

    // Aktualisiere Zustandsvariablen für den nächsten Sample
    v.f_integrator1 = g * hp + bp; // Neuer BP-Zustand
    v.f_integrator2 = g * bp + lp; // Neuer LP-Zustand

    // Speichere die Ausgänge (optional, falls man sie anderweitig braucht)
    v.f_hp = hp;
    v.f_bp = bp;
    v.f_lp = lp;

    return lp; // Gib den Tiefpass-Ausgang zurück
}


// Haupt-Audio-Callback / Buffer-Füllfunktion
void fill_audio_buffer() {
  // --- Globale Berechnungen (einmal pro Buffer) ---
  // LFO Update
  lfo_phase += lfo_increment;
  if (lfo_phase >= WAVETABLE_SIZE) lfo_phase -= WAVETABLE_SIZE;
  // LFO nutzt Sinus (wavetable 0), Wert zwischen -1.0 und 1.0
  float lfo_value = interpolate(wavetables[0], lfo_phase);

  // Berechne den aktuellen Filter-Cutoff und die Resonanz inkl. LFO-Modulation
  float cutoff_mod_factor = 1.0f + (lfo_value * global_lfo_filter_depth * 0.5f); // Moduliert um +/- (depth * 50%)
  float current_cutoff_hz = global_filter_cutoff_base_hz * cutoff_mod_factor;
  if (current_cutoff_hz < 20.0f) current_cutoff_hz = 20.0f;
  if (current_cutoff_hz > 20000.0f) current_cutoff_hz = 20000.0f; // Max Frequenz begrenzen
  float current_resonance_q = global_filter_resonance; // Resonanz wird hier nicht moduliert

  // --- Buffer füllen (Sample für Sample) ---
  for (int i = 0; i < DMA_BUFFER_FRAMES; ++i) {
    float mixed_sample = 0.0f; // Sample für diesen Frame initialisieren

    // --- Stimmen durchlaufen ---
    for (int v_idx = 0; v_idx < NUM_VOICES; ++v_idx) {
      Voice& v = voices[v_idx];

      // Nur aktive Stimmen bearbeiten
      if (!v.active && v.env_stage == ENV_IDLE) continue;

      // 1. Envelope Processing
      switch (v.env_stage) {
        case ENV_ATTACK:
          v.env_level += v.env_attack_rate;
          if (v.env_level >= 1.0f) { v.env_level = 1.0f; v.env_stage = ENV_DECAY; }
          break;
        case ENV_DECAY:
          v.env_level -= v.env_decay_rate;
          if (v.env_level <= v.env_sustain_level) {
            v.env_level = v.env_sustain_level;
            // Direkt in Release gehen, wenn Sustain Level sehr niedrig ist
            if (v.env_sustain_level <= 0.001f) v.env_stage = ENV_RELEASE;
            else v.env_stage = ENV_SUSTAIN;
          }
          break;
        case ENV_SUSTAIN:
          // Level bleibt konstant, außer wenn Sustain Level 0 ist
          if (v.env_sustain_level <= 0.001f) v.env_stage = ENV_RELEASE;
          break;
        case ENV_RELEASE:
          v.env_level -= v.env_release_rate;
          if (v.env_level <= 0.0f) {
            v.env_level = 0.0f;
            v.env_stage = ENV_IDLE;
            v.active = false; // Stimme wird erst hier wirklich inaktiv
          }
          break;
        case ENV_IDLE: default:
             v.env_level = 0.0f; // Sicherstellen, dass Level 0 ist
             // Nicht weitermachen, wenn idle
             continue; // Gehe zur nächsten Stimme
      }
      // Clamp envelope level just in case
      if (v.env_level < 0.0f) v.env_level = 0.0f;

      // Wenn die Hüllkurve auf 0 ist, brauchen wir den Rest nicht zu berechnen
      // Außer sie ist gerade erst auf IDLE gesetzt worden (dann wurde oben schon continue aufgerufen)
      if (v.env_level == 0.0f && v.env_stage != ENV_ATTACK ) continue;


      // 2. Oscillator (DDS) mit Pitch-Modulation
      float base_freq = midiNoteToFreq(v.note);
      // LFO Pitch Modulation (Vibrato) - wirkt multiplikativ auf Frequenz
      float pitch_mod_factor = powf(2.0f, (lfo_value * global_lfo_pitch_depth) / 12.0f);
      float current_freq = base_freq * pitch_mod_factor;

      v.phase_increment = (current_freq * WAVETABLE_SIZE) / SAMPLE_RATE;
      v.phase_accumulator += v.phase_increment;
      // Wrap phase accumulator
      if (v.phase_accumulator >= WAVETABLE_SIZE) v.phase_accumulator -= WAVETABLE_SIZE;
      else if (v.phase_accumulator < 0.0f) v.phase_accumulator += WAVETABLE_SIZE; // Für FM etc. später

      // Sample aus Wavetable holen (mit Interpolation)
      float osc_sample = interpolate(wavetables[v.current_wavetable_idx], v.phase_accumulator);

      // 3. Filter anwenden (mit global berechneten Parametern)
      float filtered_sample = processSVF(v, osc_sample, current_cutoff_hz, current_resonance_q);

      // 4. Amplitude anwenden (Velocity & Envelope)
      float velocity_gain = (float)v.velocity / 127.0f;
      float final_voice_sample = filtered_sample * v.env_level * velocity_gain;

      // Sample zum Mix hinzufügen
      mixed_sample += final_voice_sample;
    } // Ende Stimmen-Loop

    // --- Post-Processing & Output ---
    // Gesamtlautstärke anwenden
    mixed_sample *= global_master_volume;

    // Clipping / Limiting (optional, aber gut bei vielen Stimmen)
    // Einfaches Hard-Clipping:
    if (mixed_sample > 1.0f) mixed_sample = 1.0f;
    else if (mixed_sample < -1.0f) mixed_sample = -1.0f;
    // Alternativ: Soft Clipping (tanh)
    // mixed_sample = tanhf(mixed_sample);

    // Skalierung auf den Ziel-Integer-Bereich für DIESES System
    int16_t sample_out = (int16_t)(mixed_sample * MAX_SYSTEM_AMPLITUDE_FLOAT);

    // In I2S Puffer schreiben (Stereo)
    i2s_write_buffer[i * NUM_CHANNELS + 0] = sample_out; // Links
    i2s_write_buffer[i * NUM_CHANNELS + 1] = sample_out; // Rechts
  } // Ende Buffer-Frame-Loop

  // Schreibe den kompletten Puffer an I2S
  size_t bytes_written;
  esp_err_t err = i2s_channel_write(tx_chan, i2s_write_buffer, I2S_WRITE_BUFFER_SIZE_BYTES, &bytes_written, portMAX_DELAY);

  // Fehlerprüfung (optional, aber empfohlen)
  if (err != ESP_OK) {
    // Serial.printf("I2S write error: %s\n", esp_err_to_name(err));
  } else if (bytes_written != I2S_WRITE_BUFFER_SIZE_BYTES) {
    // Serial.printf("I2S underrun? Wrote %u, expected %d\n", bytes_written, I2S_WRITE_BUFFER_SIZE_BYTES);
  }
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 DDS Wavetable Synthesizer - IDF I2S Driver");
  Serial.printf("Target max digital amplitude: +/- %.0f\n", MAX_SYSTEM_AMPLITUDE_FLOAT);

  // MUX Pins initialisieren
  pinMode(MUX_SIG_PIN, INPUT);
  for (int i = 0; i < 4; i++) {
    pinMode(MUX_CTRL_PINS[i], OUTPUT);
    digitalWrite(MUX_CTRL_PINS[i], LOW); // Initialzustand
  }
  Serial.println("MUX Pins Initialized.");

  // Wavetables erzeugen
  generateWavetables();
  Serial.println("Wavetables generated.");

  // Stimmen initialisieren
  for (int i = 0; i < NUM_VOICES; ++i) {
    voices[i].active = false;
    voices[i].env_stage = ENV_IDLE;
    voices[i].env_level = 0.0f;
    voices[i].phase_accumulator = 0.0f;
    voices[i].f_lp = 0.0f; voices[i].f_bp = 0.0f; voices[i].f_hp = 0.0f;
    voices[i].f_integrator1 = 0.0f; voices[i].f_integrator2 = 0.0f;
  }
  Serial.println("Voices initialized.");

  // MIDI Setup
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleControlChange(handleControlChange);
  MIDI_SERIAL.begin(31250); // Standard MIDI Baudrate
  MIDI.begin(MIDI_CHANNEL); // Auf allen Kanälen hören
  Serial.println("MIDI Initialized.");

  // I2S Setup (Direkter IDF Treiber)
  Serial.println("Configuring I2S with IDF driver...");
  i2s_chan_config_t chan_cfg = {
      .id = I2S_PORT_NUM,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = DMA_BUFFER_COUNT,
      .dma_frame_num = DMA_BUFFER_FRAMES,
      .auto_clear = true,
  };
  esp_err_t err = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
  if (err != ESP_OK) { Serial.printf("I2S: Failed channel create: %s\n", esp_err_to_name(err)); while(1); }

  i2s_std_config_t std_cfg = {
      .clk_cfg = {
          .sample_rate_hz = SAMPLE_RATE,
          .clk_src = I2S_CLK_SRC_APLL,
          .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      },
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
          .mclk = I2S_MCLK_PIN,
          .bclk = I2S_BCLK_PIN,
          .ws = I2S_LRCK_PIN,
          .dout = I2S_DOUT_PIN,
          .din = GPIO_NUM_NC,
          .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false, },
      },
  };
  err = i2s_channel_init_std_mode(tx_chan, &std_cfg);
  if (err != ESP_OK) { Serial.printf("I2S: Failed init std mode: %s\n", esp_err_to_name(err)); while(1); }

  err = i2s_channel_enable(tx_chan);
  if (err != ESP_OK) { Serial.printf("I2S: Failed channel enable: %s\n", esp_err_to_name(err)); while(1); }

  Serial.println("I2S Initialized and Enabled.");
  Serial.println("Setup complete. Synthesizer running.");
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  // MIDI Nachrichten verarbeiten
  MIDI.read();

  // Potentiometer lesen (regelmäßig, aber nicht bei jedem Durchlauf)
  unsigned long current_time = millis();
  if (current_time - last_pot_read_time >= POT_READ_INTERVAL_MS) {
    readPotentiometers();
    updateGlobalParameters();
    last_pot_read_time = current_time;

    // Optional: Debug Output der Poti-Werte / globalen Parameter
    /*
    Serial.printf("V:%.2f WT:%d Att:%.0f Dec:%.0f Sus:%.2f Rel:%.0f LFO:%.1fHz P:%.2f F:%.2f Cut:%.0f Res:%.2f\n",
        global_master_volume, global_wavetable_select_idx, global_attack_time_ms, global_decay_time_ms,
        global_sustain_level, global_release_time_ms, global_lfo_rate_hz, global_lfo_pitch_depth,
        global_lfo_filter_depth, global_filter_cutoff_base_hz, global_filter_resonance);
    */
  }

  // Audio Puffer füllen und an I2S senden (blockiert, bis gesendet)
  fill_audio_buffer();

  // Kurzes Delay, um Watchdog zu beruhigen, falls fill_audio_buffer zu schnell ist
  // Normalerweise durch portMAX_DELAY in i2s_channel_write abgedeckt, aber sicher ist sicher.
  // vTaskDelay(1); // Normalerweise nicht nötig, wenn i2s_channel_write blockiert.
}
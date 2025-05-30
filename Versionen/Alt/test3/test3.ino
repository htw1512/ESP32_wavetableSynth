// ESP32-S3 - 6-VOICE POLYPHONIC SYNTH - (V30.2 - Korrektur fehlender Deklarationen)

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <driver/i2s_std.h>
#include <MIDI.h>
#include <math.h> 

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
#define DMA_BUFFER_FRAMES   256
#define DMA_BUFFER_COUNT    8               
const int BYTES_PER_FRAME_STEREO = NUM_CHANNELS * (I2S_DATA_BIT_WIDTH_16BIT / 8);
const int I2S_WRITE_BUFFER_SIZE_BYTES = DMA_BUFFER_FRAMES * BYTES_PER_FRAME_STEREO;
const float MAX_AMPLITUDE_INT16 = 15000.0f; 

// --- MIDI Config ---
#define MIDI_RX_PIN         GPIO_NUM_3
#define MIDI_TX_PIN_SERIAL2 GPIO_NUM_NC // << HIER DEFINIERT >>
#define MIDI_BAUD_RATE      31250
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI);

// --- Taster für Wellenform ---
#define WAVE_SWITCH_PIN      GPIO_NUM_19 
unsigned long last_wave_switch_press_time = 0;
const unsigned long WAVE_SWITCH_DEBOUNCE_TIME = 70; 

// --- OLED Display ---
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define OLED_RESET    -1 
#define OLED_I2C_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
volatile bool display_update_needed = true; 

// --- Wavetable ---
#define WAVETABLE_SIZE      256 
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

const char* wavetable_names[NUM_WAVETABLES] = { 
  "Sine", "Saw Down", "Square", "Triangle", "Pulse 25%", "Pulse 12%", 
  "Sync Saw", "Formant A", "Ramp Up", "HollowSq", "Formant O"
};
float wavetables[NUM_WAVETABLES][WAVETABLE_SIZE];
volatile int global_osc_wave_select_idx = WAVE_SINE;


// --- Simple Monophonic Oscillator & Envelope State ---
enum EnvelopeStage { ENV_IDLE, ENV_PLAYING, ENV_RELEASING }; 
volatile EnvelopeStage current_env_stage = ENV_IDLE;
volatile float current_env_level = 0.0f; 

volatile float current_frequency = 0.0f;
volatile float current_velocity_gain = 0.0f;
float phase_accumulator = 0.0f; 

// --- I2S Globale Variablen ---
i2s_chan_handle_t i2s_tx_chan; // << HIER DEKLARIERT >>
int16_t i2s_audio_buffer[DMA_BUFFER_FRAMES * NUM_CHANNELS]; // << HIER DEKLARIERT >>


// -----------------------------------------------------------------------------
// HILFSFUNKTIONEN
// -----------------------------------------------------------------------------
float midiNoteToFreq(uint8_t noteVal) {
  return 440.0f * powf(2.0f, (noteVal - 69.0f) / 12.0f);
}

float interpolateWavetable(float table[WAVETABLE_SIZE], float index) { 
  int i0 = (int)floorf(index); 
  int i1 = (i0 + 1); 
  float fraction = index - i0; 
  i0 %= WAVETABLE_SIZE; if (i0 < 0) i0 += WAVETABLE_SIZE; 
  i1 %= WAVETABLE_SIZE; if (i1 < 0) i1 += WAVETABLE_SIZE; 
  return table[i0] * (1.0f - fraction) + table[i1] * fraction; 
}

void normalizeWavetable(int wave_idx) {
    float max_abs_val = 0.0f;
    for (int i = 0; i < WAVETABLE_SIZE; ++i) {
        if (fabsf(wavetables[wave_idx][i]) > max_abs_val) {
            max_abs_val = fabsf(wavetables[wave_idx][i]);
        }
    }
    if (max_abs_val > 0.0001f) {
        for (int i = 0; i < WAVETABLE_SIZE; ++i) {
            wavetables[wave_idx][i] /= max_abs_val;
        }
    }
}

void generateAllSynthWavetables() {  
    for (int w = 0; w < NUM_WAVETABLES; ++w) { 
        for (int i = 0; i < WAVETABLE_SIZE; ++i) { 
            wavetables[w][i] = 0.0f; 
        } 
    }
    if (WAVE_SINE < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_SINE][i] = sinf(2.0f * PI * (float)i / WAVETABLE_SIZE); } }
    if (WAVE_SAW_DOWN < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_SAW_DOWN][i] = 1.0f - 2.0f * (float)i / WAVETABLE_SIZE; } }
    if (WAVE_SQUARE < NUM_WAVETABLES) { for (int k = 1; k <= 15; k += 2) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_SQUARE][i] += (1.0f / k) * sinf(2.0f * PI * k * (float)i / WAVETABLE_SIZE); } } normalizeWavetable(WAVE_SQUARE); }
    if (WAVE_TRIANGLE < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float val = 2.0f * (float)i / WAVETABLE_SIZE; if (val > 1.0f) { val = 2.0f - val; } wavetables[WAVE_TRIANGLE][i] = 2.0f * val - 1.0f; } }
    if (WAVE_PULSE_25 < NUM_WAVETABLES) { int pw = WAVETABLE_SIZE / 4; for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_PULSE_25][i] = (i < pw) ? 1.0f : -1.0f; } }
    if (WAVE_PULSE_12 < NUM_WAVETABLES) { int pw = WAVETABLE_SIZE / 8; for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_PULSE_12][i] = (i < pw) ? 1.0f : -1.0f; } }
    if (WAVE_SYNC_SAW < NUM_WAVETABLES) { float master_freq_ratio = 4.5f; for (int i = 0; i < WAVETABLE_SIZE; ++i) { float phase = (float)i / WAVETABLE_SIZE; float slave_phase = fmod(phase * master_freq_ratio, 1.0f); wavetables[WAVE_SYNC_SAW][i] = 1.0f - 2.0f * slave_phase; } normalizeWavetable(WAVE_SYNC_SAW); }
    if (WAVE_FORMANT_A < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = (float)i / WAVETABLE_SIZE; wavetables[WAVE_FORMANT_A][i] = 0.5f * sinf(2.0f*PI*t) + 0.3f * sinf(2.0f*PI*t*2.0f) + 0.2f * sinf(2.0f*PI*t*3.7f) + 0.1f * sinf(2.0f*PI*t*5.2f); } normalizeWavetable(WAVE_FORMANT_A); }
    if (WAVE_RAMP_UP < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { wavetables[WAVE_RAMP_UP][i] = -1.0f + 2.0f * (float)i / WAVETABLE_SIZE; } }
    if (WAVE_HOLLOW_SQUARE < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = 2.0f * PI * (float)i / WAVETABLE_SIZE; wavetables[WAVE_HOLLOW_SQUARE][i] = sinf(t) + (1.0f/3.0f)*sinf(3.0f*t) + (1.0f/5.0f)*sinf(5.0f*t); } normalizeWavetable(WAVE_HOLLOW_SQUARE); }
    if (WAVE_FORMANT_O < NUM_WAVETABLES) { for (int i = 0; i < WAVETABLE_SIZE; ++i) { float t = (float)i / WAVETABLE_SIZE; wavetables[WAVE_FORMANT_O][i] = 0.6f * sinf(2.0f*PI*t) + 0.4f * sinf(2.0f*PI*t*1.8f) + 0.1f * sinf(2.0f*PI*t*4.5f); } normalizeWavetable(WAVE_FORMANT_O); }
    Serial.println("All Wavetables Generated.");
}


// --- MIDI Callbacks ---
void handleNoteOn(byte channel, byte note, byte velocity) {
  current_frequency = midiNoteToFreq(note);
  current_velocity_gain = (float)velocity / 127.0f; 
  phase_accumulator = 0.0f; 
  current_env_level = 1.0f; 
  current_env_stage = ENV_PLAYING; 
  Serial.printf("Note On: %d, Freq: %.2f Hz, Vel: %d, Wave: %s\n", 
                note, current_frequency, velocity, wavetable_names[global_osc_wave_select_idx]);
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  if (current_frequency > 0.0f && fabsf(current_frequency - midiNoteToFreq(note)) < 0.1f) { 
    current_env_stage = ENV_RELEASING; 
    Serial.printf("Note Off: %d -> Releasing\n", note);
  }
}

// --- Taster Processing ---
void processWaveformSwitch() {
    if (digitalRead(WAVE_SWITCH_PIN) == LOW) { 
        if (millis() - last_wave_switch_press_time > WAVE_SWITCH_DEBOUNCE_TIME) {
            global_osc_wave_select_idx = (global_osc_wave_select_idx + 1) % NUM_WAVETABLES;
            display_update_needed = true; 
            Serial.print("Waveform selected: "); 
            Serial.println(wavetable_names[global_osc_wave_select_idx]); 
            last_wave_switch_press_time = millis();
        }
    }
}

void updateDisplay() {
    if (!display_update_needed) return;
    // if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) { // Dieser Check ist hier nicht ideal
    //     Serial.println(F("SSD1306 error on updateDisplay"));
    //     display_update_needed = false; 
    //     return; 
    // }
    display.clearDisplay();
    display.setTextSize(1); 
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); 
    if (global_osc_wave_select_idx < NUM_WAVETABLES) { 
        display.print("Wave: "); 
        display.println(wavetable_names[global_osc_wave_select_idx]); 
    } 
    else { display.println("Wave Err"); }
    display.display();
    display_update_needed = false;
}


// --- Audio Buffer Fill ---
void fillTestAudioBuffer() {
  float phase_increment = 0.0f;
  if (current_frequency > 0.0f) {
    phase_increment = (current_frequency * WAVETABLE_SIZE) / (float)SAMPLE_RATE;
  }

  if (current_env_stage == ENV_RELEASING) {
      current_env_level = 0.0f; 
      current_env_stage = ENV_IDLE;
  }

  for (int frame = 0; frame < DMA_BUFFER_FRAMES; ++frame) {
    float sample_float = 0.0f;
    if (current_env_level > 0.0f && current_frequency > 0.0f) {
      sample_float = interpolateWavetable(wavetables[global_osc_wave_select_idx], phase_accumulator);
      sample_float *= current_velocity_gain * current_env_level; 
      
      phase_accumulator += phase_increment;
      if (phase_accumulator >= WAVETABLE_SIZE) { 
        phase_accumulator -= WAVETABLE_SIZE;
      }
    } else if (current_env_stage != ENV_IDLE && current_env_level <= 0.0f) {
        current_frequency = 0.0f; 
        current_velocity_gain = 0.0f;
        current_env_stage = ENV_IDLE;
    }

    int16_t sample_int16 = (int16_t)(sample_float * MAX_AMPLITUDE_INT16);
    i2s_audio_buffer[frame * NUM_CHANNELS + 0] = sample_int16; 
    i2s_audio_buffer[frame * NUM_CHANNELS + 1] = sample_int16; 
  }

  size_t bytes_written;
  i2s_channel_write(i2s_tx_chan, i2s_audio_buffer, I2S_WRITE_BUFFER_SIZE_BYTES, &bytes_written, portMAX_DELAY);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Serial.println("V30.2: Minimal + All Waves + Switch GPIO19 + Minimal OLED (Fixes)");

  generateAllSynthWavetables(); 

  pinMode(WAVE_SWITCH_PIN, INPUT_PULLUP); 

  // OLED Init
  Wire.begin(); 
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    Serial.println(F("SSD1306 Initialized"));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Synth V30.2"); 
    display.setCursor(0,10);
    display.print("Wave: ");
    if (global_osc_wave_select_idx < NUM_WAVETABLES) display.print(wavetable_names[global_osc_wave_select_idx]);
    else display.print("Error");
    display.display();
    display_update_needed = false; 
  }

  Serial2.begin(MIDI_BAUD_RATE, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN_SERIAL2); // MIDI_TX_PIN_SERIAL2 ist definiert
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  Serial.println("MIDI Initialized.");

  i2s_chan_config_t chan_cfg = {
    .id = I2S_PORT_NUM, .role = I2S_ROLE_MASTER, .dma_desc_num = DMA_BUFFER_COUNT, 
    .dma_frame_num = DMA_BUFFER_FRAMES, .auto_clear = true,
  };
  esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL);
  if (err != ESP_OK) { Serial.printf("I2S Chan new Error: %s\n", esp_err_to_name(err)); while(1); }
  i2s_std_config_t std_cfg = {
    .clk_cfg = {.sample_rate_hz = SAMPLE_RATE, .clk_src = I2S_CLK_SRC_APLL, .mclk_multiple = I2S_MCLK_MULTIPLE_256 },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = { .mclk = I2S_MCLK_PIN, .bclk = I2S_BCLK_PIN, .ws = I2S_LRCK_PIN, .dout = I2S_DOUT_PIN, .din = GPIO_NUM_NC, .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false, } }
  };
  err = i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg);
  if (err != ESP_OK) { Serial.printf("I2S Std Init Error: %s\n", esp_err_to_name(err)); while(1); }
  err = i2s_channel_enable(i2s_tx_chan);
  if (err != ESP_OK) { Serial.printf("I2S Chan Enable Error: %s\n", esp_err_to_name(err)); while(1); }
  Serial.println("I2S Initialized.");
  Serial.println("--- Test Sketch Ready ---");
}

unsigned long lastCtrlTime = 0;
const unsigned long CTRL_INTERVAL = 10; 
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100; 

void loop() {
  MIDI.read(); 
  
  unsigned long currentTime = millis();
  if (currentTime - lastCtrlTime >= CTRL_INTERVAL) {
    processWaveformSwitch(); 
    lastCtrlTime = currentTime;
  }

  if (display_update_needed && (currentTime - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL)) {
      updateDisplay();
      lastDisplayUpdateTime = currentTime;
  }
  
  fillTestAudioBuffer(); 
}
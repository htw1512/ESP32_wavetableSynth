#include <Arduino.h>
#include <driver/i2s_std.h>
#include <math.h> // Für sinf

// --- I2S Audio Config ---
#define SAMPLE_RATE         22050 // Deine aktuelle Samplerate
#define NUM_CHANNELS        2     // Stereo, auch wenn wir Mono-Signal ausgeben
#define I2S_PORT_NUM        I2S_NUM_0
#define I2S_BCLK_PIN        (GPIO_NUM_27) // Dein I2S BCLK Pin
#define I2S_LRCK_PIN        (GPIO_NUM_26) // Dein I2S LRCK/WS Pin
#define I2S_DOUT_PIN        (GPIO_NUM_25) // Dein I2S Data Out Pin
#define I2S_MCLK_PIN        (GPIO_NUM_NC) // Master Clock nicht verwendet

#define DMA_BUFFER_FRAMES   256   // Größe des DMA Buffers in Frames
#define DMA_BUFFER_COUNT    8     // Anzahl der DMA Buffer

const int BYTES_PER_FRAME_STEREO = NUM_CHANNELS * (I2S_DATA_BIT_WIDTH_16BIT / 8);
const int I2S_WRITE_BUFFER_SIZE_BYTES = DMA_BUFFER_FRAMES * BYTES_PER_FRAME_STEREO;
const float MAX_AMPLITUDE_INT16 = 15000.0f; // Amplitude für den Sinuston (etwas unter max)

i2s_chan_handle_t i2s_tx_chan;
int16_t i2s_audio_buffer[DMA_BUFFER_FRAMES * NUM_CHANNELS];

// --- Sine Oscillator State ---
const float TARGET_FREQUENCY = 400.0f; // 400 Hz
const float AMPLITUDE = 0.5f;          // Amplitude (0.0 bis 1.0)
float phase_accumulator = 0.0f;
const float PI_2 = 2.0f * PI;

// --- Audio Buffer Fill ---
void fillSineWaveBuffer() {
  float phase_increment = (PI_2 * TARGET_FREQUENCY) / (float)SAMPLE_RATE;

  for (int frame = 0; frame < DMA_BUFFER_FRAMES; ++frame) {
    float sample_float = sinf(phase_accumulator) * AMPLITUDE;
    
    phase_accumulator += phase_increment;
    if (phase_accumulator >= PI_2) {
      phase_accumulator -= PI_2;
    }
    
    int16_t sample_int16 = (int16_t)(sample_float * MAX_AMPLITUDE_INT16);
    i2s_audio_buffer[frame * NUM_CHANNELS + 0] = sample_int16; // Links
    i2s_audio_buffer[frame * NUM_CHANNELS + 1] = sample_int16; // Rechts (gleiches Signal für Stereo)
  }

  size_t bytes_written;
  esp_err_t err = i2s_channel_write(i2s_tx_chan, i2s_audio_buffer, I2S_WRITE_BUFFER_SIZE_BYTES, &bytes_written, portMAX_DELAY);
  if (err != ESP_OK) {
    Serial.printf("I2S Write Error: %s\n", esp_err_to_name(err));
  }
  // if (bytes_written != I2S_WRITE_BUFFER_SIZE_BYTES) {
  //   Serial.println("I2S underrun or incomplete write?");
  // }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 I2S 400Hz Sine Wave Test");

  // I2S Init
  i2s_chan_config_t chan_cfg = {
    .id = I2S_PORT_NUM,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUFFER_COUNT,
    .dma_frame_num = DMA_BUFFER_FRAMES,
    .auto_clear = true,
  };
  esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL);
  if (err != ESP_OK) { Serial.printf("I2S Chan new Error: %s\n", esp_err_to_name(err)); while(1); }

  i2s_std_config_t std_cfg = {
    .clk_cfg = {
      .sample_rate_hz = SAMPLE_RATE,
      .clk_src = I2S_CLK_SRC_APLL, // Oder I2S_CLK_SRC_DEFAULT
      .mclk_multiple = I2S_MCLK_MULTIPLE_256 
    },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_MCLK_PIN, 
      .bclk = I2S_BCLK_PIN, 
      .ws = I2S_LRCK_PIN, 
      .dout = I2S_DOUT_PIN, 
      .din = GPIO_NUM_NC, // Nicht als Input verwendet
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false, }
    }
  };
  err = i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg);
  if (err != ESP_OK) { Serial.printf("I2S Std Init Error: %s\n", esp_err_to_name(err)); while(1); }
  
  err = i2s_channel_enable(i2s_tx_chan);
  if (err != ESP_OK) { Serial.printf("I2S Chan Enable Error: %s\n", esp_err_to_name(err)); while(1); }

  Serial.println("I2S Initialized. Playing 400Hz tone...");
}

// --- Loop ---
void loop() {
  fillSineWaveBuffer(); 
  // Kein Delay hier, da i2s_channel_write blockierend ist und das Timing vorgibt.
  // In einem komplexeren Sketch würde man FreeRTOS Tasks verwenden.
}
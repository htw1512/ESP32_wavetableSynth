// ESP32-S3 - MONO SYNTH: 1 OSC (Wave Sel), ADSR, VCF, LFO - Direct Potis (KORRIGIERT V16 - Control Task Call FINAL)
// FreeRTOS Zwei-Task-Architektur.

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

// Audio Ausgabe (I2S)
#define SAMPLE_RATE         44100
#define NUM_CHANNELS        2
#define I2S_PORT_NUM        I2S_NUM_0
#define I2S_BCLK_PIN        (GPIO_NUM_27)
#define I2S_LRCK_PIN        (GPIO_NUM_26)
#define I2S_DOUT_PIN        (GPIO_NUM_25)
#define I2S_MCLK_PIN        (GPIO_NUM_NC)
#define DMA_BUFFER_COUNT    4
#define DMA_BUFFER_FRAMES   256
const int BYTES_PER_FRAME_STEREO = NUM_CHANNELS * (I2S_DATA_BIT_WIDTH_16BIT / 8);
const int I2S_WRITE_BUFFER_SIZE_BYTES = DMA_BUFFER_FRAMES * BYTES_PER_FRAME_STEREO;
const float MAX_SYSTEM_AMPLITUDE_FLOAT = 2000.0f;

// MIDI Eingang
#define MIDI_RX_PIN         3
#define MIDI_TX_PIN_FOR_SERIAL2 -1
#define MIDI_BAUD_RATE      31250
#define MIDI_LISTEN_CHANNEL MIDI_CHANNEL_OMNI

// Potis (direkt angeschlossen)
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

// Synthesizer
#define NUM_VOICES_ACTIVE   1 // MONO SYNTH
#define NUM_WAVETABLES      4
#define WAVETABLE_SIZE      256
#define PI_F                3.14159265358979323846f
const float PRE_FILTER_DRIVE_ATTENUATION = 0.5f;
// const float OSC_OUTPUT_AMPLITUDE = 0.8f; // Wird jetzt durch Master Volume Poti geregelt

// Voice Struktur
enum EnvelopeStage { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };
struct VoiceState {
  volatile bool note_on_trigger; volatile bool note_off_trigger;
  uint8_t note; float velocity_gain;
  bool is_playing;
  float phase_accumulator;
  float phase_increment;
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
VoiceState voice;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI);
float wavetables[NUM_WAVETABLES][WAVETABLE_SIZE];
unsigned long last_direct_pot_read_time = 0;
const unsigned long DIRECT_POT_READ_INTERVAL_MS = 20;
SemaphoreHandle_t voiceMutex = NULL;
TaskHandle_t audioMainTaskHandle = NULL;
TaskHandle_t controlMainTaskHandle = NULL;

volatile int global_osc_wave_select_idx = 0;
volatile float global_attack_time_ms, global_decay_time_ms, global_sustain_level, global_release_time_ms;
volatile float global_master_volume = 0.7f;
volatile float global_filter_cutoff_hz_target, smoothed_filter_cutoff_hz = 15000.0f;
const float CUTOFF_SMOOTHING_FACTOR = 0.05f;
volatile float global_filter_resonance_target, smoothed_filter_resonance = 0.707f;
const float RESO_SMOOTHING_FACTOR = 0.08f;
volatile float global_lfo_rate_hz_target = 0.0f, smoothed_lfo_rate_hz = 0.0f;
const float LFO_RATE_SMOOTHING_FACTOR = 0.1f;
volatile float global_lfo_pitch_depth = 0.0f;
volatile float global_lfo_filter_depth = 0.0f;
float lfo_phase = 0.0f; float lfo_increment = 0.0f;
const float FILTER_KEYTRACK_AMOUNT = 0.4f;

float global_filter_lp_state = 0.0f, global_filter_bp_state = 0.0f;
float global_filter_integrator1_state = 0.0f, global_filter_integrator2_state = 0.0f;

int raw_pot_osc_wave;
int raw_pot_attack, raw_pot_decay, raw_pot_sustain, raw_pot_release;
int raw_pot_filter_cutoff, raw_pot_filter_reso;
int raw_pot_lfo_rate, raw_pot_lfo_pitch_depth, raw_pot_lfo_filter_depth;
int raw_pot_master_volume;

// -----------------------------------------------------------------------------
// HILFSFUNKTIONEN
// -----------------------------------------------------------------------------
float midiNoteToFreq(uint8_t noteVal) { return 440.0f * powf(2.0f, (noteVal - 69.0f) / 12.0f); }
float interpolateWavetable(float table[WAVETABLE_SIZE], float index) { int i0=(int)floorf(index),i1=(i0+1);float fr=index-i0;i0%=WAVETABLE_SIZE;if(i0<0)i0+=WAVETABLE_SIZE;i1%=WAVETABLE_SIZE;if(i1<0)i1+=WAVETABLE_SIZE;return table[i0]*(1.0f-fr)+table[i1]*fr;}
void generateAllSynthWavetables() {for(int w=0;w<NUM_WAVETABLES;++w)for(int i=0;i<WAVETABLE_SIZE;++i)wavetables[w][i]=0.0f;for(int i=0;i<WAVETABLE_SIZE;++i)wavetables[0][i]=sinf(2.0f*PI_F*(float)i/WAVETABLE_SIZE);for(int i=0;i<WAVETABLE_SIZE;++i)wavetables[1][i]=1.0f-2.0f*(float)i/WAVETABLE_SIZE;for(int k=1;k<=15;k+=2){for(int i=0;i<WAVETABLE_SIZE;++i)wavetables[2][i]+=(1.0f/k)*sinf(2.0f*PI_F*k*(float)i/WAVETABLE_SIZE);}float m=0;for(int i=0;i<WAVETABLE_SIZE;++i)if(fabsf(wavetables[2][i])>m)m=fabsf(wavetables[2][i]);if(m>0.0001f)for(int i=0;i<WAVETABLE_SIZE;++i)wavetables[2][i]/=m;for(int i=0;i<WAVETABLE_SIZE;++i){float v=2.0f*(float)i/WAVETABLE_SIZE;if(v>1.0f)v=2.0f-v;wavetables[3][i]=2.0f*v-1.0f;}}

void readAndUpdateAllDirectPotis() { // Konsistenter Name
  int num_adc_samples = 3; long adc_sum;
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_OSC_WAVE_PIN);delayMicroseconds(2);}raw_pot_osc_wave=adc_sum/num_adc_samples;global_osc_wave_select_idx=map(raw_pot_osc_wave,0,4095,0,NUM_WAVETABLES-1);
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_ATTACK_PIN);delayMicroseconds(2);}raw_pot_attack=adc_sum/num_adc_samples;global_attack_time_ms=map(raw_pot_attack,0,4095,1,2000);
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_DECAY_PIN);delayMicroseconds(2);}raw_pot_decay=adc_sum/num_adc_samples;global_decay_time_ms=map(raw_pot_decay,0,4095,1,2000);
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_SUSTAIN_PIN);delayMicroseconds(2);}raw_pot_sustain=adc_sum/num_adc_samples;global_sustain_level=(float)raw_pot_sustain/4095.0f;
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_RELEASE_PIN);delayMicroseconds(2);}raw_pot_release=adc_sum/num_adc_samples;global_release_time_ms=map(raw_pot_release,0,4095,1,3000);
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_FILTER_CUTOFF_PIN);delayMicroseconds(2);}raw_pot_filter_cutoff=adc_sum/num_adc_samples;float cutoff_norm=(float)raw_pot_filter_cutoff/4095.0f;global_filter_cutoff_hz_target=40.0f+(cutoff_norm*cutoff_norm*17000.0f);if(global_filter_cutoff_hz_target>18000.0f)global_filter_cutoff_hz_target=18000.0f;if(global_filter_cutoff_hz_target<40.0f)global_filter_cutoff_hz_target=40.0f;smoothed_filter_cutoff_hz=smoothed_filter_cutoff_hz*(1.0f-CUTOFF_SMOOTHING_FACTOR)+global_filter_cutoff_hz_target*CUTOFF_SMOOTHING_FACTOR;
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_FILTER_RESO_PIN);delayMicroseconds(2);}raw_pot_filter_reso=adc_sum/num_adc_samples;global_filter_resonance_target=map(raw_pot_filter_reso,0,4095,70,300)/100.0f;if(global_filter_resonance_target<0.707f)global_filter_resonance_target=0.707f;smoothed_filter_resonance=smoothed_filter_resonance*(1.0f-RESO_SMOOTHING_FACTOR)+global_filter_resonance_target*RESO_SMOOTHING_FACTOR;
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_LFO_RATE_PIN);delayMicroseconds(2);}raw_pot_lfo_rate=adc_sum/num_adc_samples;global_lfo_rate_hz_target=map(raw_pot_lfo_rate,0,4095,0,700)/100.0f;smoothed_lfo_rate_hz=smoothed_lfo_rate_hz*(1.0f-LFO_RATE_SMOOTHING_FACTOR)+global_lfo_rate_hz_target*LFO_RATE_SMOOTHING_FACTOR;
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_LFO_PITCH_DEPTH_PIN);delayMicroseconds(2);}raw_pot_lfo_pitch_depth=adc_sum/num_adc_samples;global_lfo_pitch_depth=((float)raw_pot_lfo_pitch_depth/4095.0f)*1.0f;
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_LFO_FILTER_DEPTH_PIN);delayMicroseconds(2);}raw_pot_lfo_filter_depth=adc_sum/num_adc_samples;global_lfo_filter_depth=(float)raw_pot_lfo_filter_depth/4095.0f;
  adc_sum=0;for(int k=0;k<num_adc_samples;k++){adc_sum+=analogRead(POT_MASTER_VOLUME_PIN);delayMicroseconds(2);}raw_pot_master_volume=adc_sum/num_adc_samples;global_master_volume=(float)raw_pot_master_volume/4095.0f;
}

// -----------------------------------------------------------------------------
// MIDI CALLBACKS
// -----------------------------------------------------------------------------
void midiNoteOnHandler(byte ch,byte n,byte vel){
  if(vel>0){
    if(xSemaphoreTake(voiceMutex,(TickType_t)10)==pdTRUE){
      voice.note=n; voice.velocity_gain=(float)vel/127.0f;
      voice.note_on_trigger=true; voice.note_off_trigger=false;
      voice.current_wave_idx = global_osc_wave_select_idx;
      voice.env_attack_rate=(global_attack_time_ms>0)?(1.0f/(global_attack_time_ms*0.001f*SAMPLE_RATE)):1.0f;
      voice.env_decay_rate=(global_decay_time_ms>0)?(1.0f/(global_decay_time_ms*0.001f*SAMPLE_RATE)):1.0f;
      voice.env_sustain_level_voice=global_sustain_level;
      voice.env_release_rate=(global_release_time_ms>0)?(1.0f/(global_release_time_ms*0.001f*SAMPLE_RATE)):1.0f;
      xSemaphoreGive(voiceMutex);
    }
  } else {midiNoteOffHandler(ch,n,vel);}
}
void midiNoteOffHandler(byte ch,byte n,byte vel){
  if(xSemaphoreTake(voiceMutex,(TickType_t)10)==pdTRUE){
    if(voice.is_playing && voice.note==n) voice.note_off_trigger=true;
    xSemaphoreGive(voiceMutex);
  }
}

// -----------------------------------------------------------------------------
// AUDIO PROCESSING
// -----------------------------------------------------------------------------
inline float processGlobalSVF(float input_sample) {
    float cutoff = smoothed_filter_cutoff_hz;
    float lfo_val_for_filter = interpolateWavetable(wavetables[0], lfo_phase);
    if (global_lfo_filter_depth > 0.001f && lfo_increment > 0.0001f) {
        float mod_range = cutoff * 0.8f;
        cutoff += lfo_val_for_filter * global_lfo_filter_depth * mod_range;
        if(cutoff < 20.0f) cutoff = 20.0f; if(cutoff > 18000.0f) cutoff = 18000.0f;
    }
    float q=smoothed_filter_resonance; float g=tanf(PI_F*cutoff/SAMPLE_RATE);float k=1.0f/q;if(k<0.01f)k=0.01f;
    float mixed_input=input_sample-global_filter_bp_state*k-global_filter_integrator2_state;
    global_filter_bp_state=g*mixed_input+global_filter_integrator1_state;global_filter_integrator1_state=g*mixed_input+global_filter_bp_state;
    global_filter_lp_state=g*global_filter_bp_state+global_filter_integrator2_state;global_filter_integrator2_state=g*global_filter_bp_state+global_filter_lp_state;
    return global_filter_lp_state;
}

void fillAudioBuffer() {
  if(smoothed_lfo_rate_hz > 0.001f){lfo_increment=(smoothed_lfo_rate_hz*WAVETABLE_SIZE)/SAMPLE_RATE;lfo_phase+=lfo_increment;if(lfo_phase>=WAVETABLE_SIZE)lfo_phase-=WAVETABLE_SIZE;}else{lfo_increment=0.0f;lfo_phase=0.0f;}
  float lfo_val_for_pitch=interpolateWavetable(wavetables[0],lfo_phase);

  VoiceState local_voice; // Lokaler Snapshot für die eine globale Stimme
  // --- Kritischer Abschnitt: Voice State lesen/schreiben ---
  if(xSemaphoreTake(voiceMutex,(TickType_t)2)==pdTRUE){
    if(voice.note_on_trigger){ // Wenn ein NoteOn-Event vom MIDI-Task kam
      local_voice.note=voice.note; local_voice.velocity_gain=voice.velocity_gain;
      local_voice.is_playing=true; local_voice.env_stage=ENV_ATTACK; local_voice.env_level=0.0f;
      local_voice.phase_accumulator=0.0f; local_voice.current_wave_idx=voice.current_wave_idx;
      local_voice.env_attack_rate=voice.env_attack_rate; local_voice.env_decay_rate=voice.env_decay_rate;
      local_voice.env_sustain_level_voice=voice.env_sustain_level_voice; local_voice.env_release_rate=voice.env_release_rate;
      voice.note_on_trigger=false; // Trigger konsumiert
    } else if(voice.note_off_trigger){ // Wenn ein NoteOff-Event kam
      if(local_voice.is_playing && local_voice.env_stage!=ENV_RELEASE) local_voice.env_stage=ENV_RELEASE;
      voice.note_off_trigger=false; // Trigger konsumiert
    } else { // Kein neuer Trigger, übernehme den laufenden Zustand für die lokale Bearbeitung
      local_voice.is_playing=voice.is_playing; local_voice.note=voice.note; local_voice.velocity_gain=voice.velocity_gain;
      local_voice.env_stage=voice.env_stage; local_voice.env_level=voice.env_level;
      local_voice.phase_accumulator=voice.phase_accumulator; local_voice.current_wave_idx=voice.current_wave_idx;
      // ADSR-Raten bleiben die von NoteOn übernommenen
      local_voice.env_attack_rate=voice.env_attack_rate; local_voice.env_decay_rate=voice.env_decay_rate;
      local_voice.env_sustain_level_voice=voice.env_sustain_level_voice; local_voice.env_release_rate=voice.env_release_rate;
    }
    xSemaphoreGive(voiceMutex);
  } // --- Ende Kritischer Abschnitt ---


  for(int frame=0;frame<DMA_BUFFER_FRAMES;++frame){
    float osc_output=0.0f; // Für die eine Stimme
    if(local_voice.is_playing || local_voice.env_level > 0.0f){ // Verarbeite auch im Release
      switch(local_voice.env_stage){
        case ENV_ATTACK:local_voice.env_level+=local_voice.env_attack_rate;if(local_voice.env_level>=1.0f){local_voice.env_level=1.0f;local_voice.env_stage=ENV_DECAY;}break;
        case ENV_DECAY:local_voice.env_level-=local_voice.env_decay_rate;if(local_voice.env_level<=local_voice.env_sustain_level_voice){local_voice.env_level=local_voice.env_sustain_level_voice;if(local_voice.env_sustain_level_voice<=0.001f)local_voice.env_stage=ENV_RELEASE;else local_voice.env_stage=ENV_SUSTAIN;}break;
        case ENV_SUSTAIN:if(local_voice.env_sustain_level_voice<=0.001f)local_voice.env_stage=ENV_RELEASE;break;
        case ENV_RELEASE:local_voice.env_level-=local_voice.env_release_rate;if(local_voice.env_level<=0.0f){local_voice.env_level=0.0f;local_voice.env_stage=ENV_IDLE;local_voice.is_playing=false;}break;
        case ENV_IDLE:default:local_voice.env_level=0.0f;local_voice.is_playing=false; // Sicherstellen
      }
      if(local_voice.env_level<0.0f)local_voice.env_level=0.0f;

      if(local_voice.is_playing && local_voice.env_level > 0.0f){
        float base_freq=midiNoteToFreq(local_voice.note);float pitch_mod_factor=1.0f;
        if(global_lfo_pitch_depth>0.001f&&lfo_increment>0.0001f){pitch_mod_factor=powf(2.0f,(lfo_val_for_pitch*global_lfo_pitch_depth)/12.0f);}
        float current_note_freq=base_freq*pitch_mod_factor;
        local_voice.phase_increment=(current_note_freq*WAVETABLE_SIZE)/SAMPLE_RATE;
        local_voice.phase_accumulator+=local_voice.phase_increment;
        if(local_voice.phase_accumulator>=WAVETABLE_SIZE)local_voice.phase_accumulator-=WAVETABLE_SIZE;
        osc_output=interpolateWavetable(wavetables[local_voice.current_wave_idx],local_voice.phase_accumulator);
        osc_output*=local_voice.velocity_gain*local_voice.env_level;
      }
    }
    
    osc_output*=PRE_FILTER_DRIVE_ATTENUATION;
    float filtered_output = processGlobalSVF(osc_output);
    filtered_output*=global_master_volume;
    
    if(filtered_output>1.0f)filtered_output=1.0f;else if(filtered_output<-1.0f)filtered_output=-1.0f;
    int16_t sample_out=(int16_t)(filtered_output*MAX_SYSTEM_AMPLITUDE_FLOAT);
    i2s_audio_buffer[frame*NUM_CHANNELS+0]=sample_out;
    i2s_audio_buffer[frame*NUM_CHANNELS+1]=sample_out;
  }

  // --- Kritischer Abschnitt: Voice State zurückschreiben ---
  if(xSemaphoreTake(voiceMutex,(TickType_t)2)==pdTRUE){
    voice.is_playing=local_voice.is_playing;voice.env_stage=local_voice.env_stage;voice.env_level=local_voice.env_level;
    voice.phase_accumulator=local_voice.phase_accumulator;voice.current_wave_idx=local_voice.current_wave_idx;
    // ADSR Raten und Note/Velocity werden nur von MIDI gesetzt, nicht hier zurückschreiben.
    xSemaphoreGive(voiceMutex);
  }
  size_t bytes_written;i2s_channel_write(i2s_tx_chan,i2s_audio_buffer,I2S_WRITE_BUFFER_SIZE_BYTES,&bytes_written,portMAX_DELAY);
}

// -----------------------------------------------------------------------------
// FREERTOS TASKS
// -----------------------------------------------------------------------------
void audioMainTask(void *pvParameters) {
  VoiceState local_voice_init_audio; // Für die einmalige Initialisierung des lokalen Snapshots im Audio Task
  if(xSemaphoreTake(voiceMutex,portMAX_DELAY)==pdTRUE){local_voice_init_audio=voice;xSemaphoreGive(voiceMutex);} else{vTaskDelete(NULL);}
  // Übertrage den initialen Zustand in die lokale Variable des Tasks, falls nötig
  // Hier nicht zwingend, da fillAudioBuffer sowieso den globalen Zustand (via Snapshot) liest
  while (true) { fillAudioBuffer(); }
}
void controlMainTask(void *pvParameters) {
  unsigned long lcprt_ctrl=0; // Korrekte lokale Variable
  while (true) {
    MIDI.read();
    unsigned long ct_ctrl=millis();
    if (ct_ctrl-lcprt_ctrl>=DIRECT_POT_READ_INTERVAL_MS) {
      readAndUpdateAllDirectPotis(); // Korrekter Funktionsname
      lcprt_ctrl=ct_ctrl;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); while(!Serial && millis()<2000);
  Serial.println("ESP32-S3 MONO Synth - Full Featured (Corrected Names V16)");

  pinMode(POT_OSC_WAVE_PIN,INPUT);
  pinMode(POT_ATTACK_PIN,INPUT);pinMode(POT_DECAY_PIN,INPUT);pinMode(POT_SUSTAIN_PIN,INPUT);pinMode(POT_RELEASE_PIN,INPUT);
  pinMode(POT_FILTER_CUTOFF_PIN,INPUT);pinMode(POT_FILTER_RESO_PIN,INPUT);
  pinMode(POT_LFO_RATE_PIN,INPUT);pinMode(POT_LFO_PITCH_DEPTH_PIN,INPUT);
  pinMode(POT_LFO_FILTER_DEPTH_PIN,INPUT);pinMode(POT_MASTER_VOLUME_PIN,INPUT);
  Serial.println("All 11 Poti Pins initialized.");

  generateAllSynthWavetables();
  voiceMutex=xSemaphoreCreateMutex();if(voiceMutex==NULL){Serial.println("Mutex Fail!");while(1);}
  voice.note_on_trigger=false;voice.note_off_trigger=false;voice.is_playing=false;voice.env_stage=ENV_IDLE;voice.env_level=0.0f;voice.phase_accumulator=0.0f;voice.current_wave_idx=0;
  global_filter_lp_state=0.0f;global_filter_bp_state=0.0f;global_filter_integrator1_state=0.0f;global_filter_integrator2_state=0.0f;
  Serial.println("Global resources & Voice initialized.");

  Serial2.begin(MIDI_BAUD_RATE,SERIAL_8N1,MIDI_RX_PIN,MIDI_TX_PIN_FOR_SERIAL2);
  MIDI.setHandleNoteOn(midiNoteOnHandler);MIDI.setHandleNoteOff(midiNoteOffHandler);
  MIDI.begin(MIDI_LISTEN_CHANNEL); Serial.println("MIDI Initialized.");

  i2s_chan_config_t chan_cfg={.id=I2S_PORT_NUM,.role=I2S_ROLE_MASTER,.dma_desc_num=DMA_BUFFER_COUNT,.dma_frame_num=DMA_BUFFER_FRAMES,.auto_clear=true};
  esp_err_t err=i2s_new_channel(&chan_cfg,&i2s_tx_chan,NULL);if(err!=ESP_OK){Serial.printf("I2S:ChCr:%s\n",esp_err_to_name(err));while(1);}
  i2s_std_config_t std_cfg={.clk_cfg={.sample_rate_hz=SAMPLE_RATE,.clk_src=I2S_CLK_SRC_APLL,.mclk_multiple=I2S_MCLK_MULTIPLE_256},
                               .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,I2S_SLOT_MODE_STEREO),
                               .gpio_cfg={.mclk=I2S_MCLK_PIN,.bclk=I2S_BCLK_PIN,.ws=I2S_LRCK_PIN,.dout=I2S_DOUT_PIN,.din=GPIO_NUM_NC,
                                            .invert_flags={.mclk_inv=false,.bclk_inv=false,.ws_inv=false,}}};
  err=i2s_channel_init_std_mode(i2s_tx_chan,&std_cfg);if(err!=ESP_OK){Serial.printf("I2S:StdMd:%s\n",esp_err_to_name(err));while(1);}
  err=i2s_channel_enable(i2s_tx_chan);if(err!=ESP_OK){Serial.printf("I2S:ChEn:%s\n",esp_err_to_name(err));while(1);}
  Serial.println("I2S Initialized.");

  xTaskCreatePinnedToCore(audioMainTask,"AudioMain",16384,NULL,configMAX_PRIORITIES-1,&audioMainTaskHandle,1);
  xTaskCreatePinnedToCore(controlMainTask,"ControlMain",4096,NULL,5,&controlMainTaskHandle,0);
  Serial.println("--- Setup Complete. Tasks Running. ---");
}

void loop() { vTaskDelay(1000 / portTICK_PERIOD_MS); }
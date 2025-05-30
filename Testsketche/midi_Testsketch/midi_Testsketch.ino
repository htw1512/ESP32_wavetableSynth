#include <Arduino.h>
#include <MIDI.h>

// --- MIDI Config ---
// GPIO3 ist der RX-Pin für Serial2, an dem dein MIDI-Shield/Keyboard angeschlossen ist.
#define MIDI_RX_PIN         GPIO_NUM_3
#define MIDI_TX_PIN_SERIAL2 GPIO_NUM_NC // TX wird für reinen Empfang nicht benötigt

#define MIDI_BAUD_RATE      31250

// MIDI-Instanz erstellen, die auf Serial2 lauscht
// Der Name "midiIn" ist hier nur ein Beispiel
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, midiIn);

// --- MIDI Callback Funktionen ---
// Diese Funktionen werden von der MIDI-Bibliothek aufgerufen, wenn entsprechende Nachrichten empfangen werden.

void handleNoteOn(byte channel, byte note, byte velocity) {
  Serial.print("Note On:  CH=");
  Serial.print(channel);
  Serial.print(", Note=");
  Serial.print(note);
  Serial.print(", Vel=");
  Serial.println(velocity);
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  Serial.print("Note Off: CH=");
  Serial.print(channel);
  Serial.print(", Note=");
  Serial.print(note);
  Serial.print(", Vel=");
  Serial.println(velocity);
  // Velocity bei Note Off ist oft 0 oder die tatsächliche Release-Velocity
}

void handleControlChange(byte channel, byte control, byte value) {
  Serial.print("CC:       CH=");
  Serial.print(channel);
  Serial.print(", Ctrl=");
  Serial.print(control);
  Serial.print(", Val=");
  Serial.println(value);
}

void handlePitchBend(byte channel, int bend) {
  // 'bend' ist ein 14-Bit-Wert (0-16383), wobei 8192 die Mitte ist.
  // Die MIDI-Bibliothek gibt ihn oft schon zentriert von -8192 bis +8191 aus.
  Serial.print("PitchBend:CH=");
  Serial.print(channel);
  Serial.print(", Bend=");
  Serial.println(bend); 
}

// Du kannst hier weitere Handler für andere MIDI-Nachrichtentypen hinzufügen:
// void handleProgramChange(byte channel, byte program) { ... }
// void handleAfterTouchPoly(byte channel, byte note, byte pressure) { ... }
// void handleAfterTouchChannel(byte channel, byte pressure) { ... }
// void handleClock() { Serial.println("MIDI Clock"); }
// void handleStart() { Serial.println("MIDI Start"); }
// void handleContinue() { Serial.println("MIDI Continue"); }
// void handleStop() { Serial.println("MIDI Stop"); }
// void handleSystemExclusive(byte *array, unsigned size) { ... }


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Warte auf Serial Monitor
  }
  Serial.println("ESP32 MIDI Input Test on GPIO3 (Serial2 RX)");

  // Initialisiere Serial2 für MIDI-Empfang an GPIO3
  // Wichtig: Der TX-Pin wird hier auf GPIO_NUM_NC (Not Connected) gesetzt,
  // da wir nur empfangen wollen und keinen Konflikt auf einem anderen Pin erzeugen möchten.
  Serial2.begin(MIDI_BAUD_RATE, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN_SERIAL2);
  
  // MIDI-Bibliothek Callbacks zuweisen
  midiIn.setHandleNoteOn(handleNoteOn);
  midiIn.setHandleNoteOff(handleNoteOff);
  midiIn.setHandleControlChange(handleControlChange);
  midiIn.setHandlePitchBend(handlePitchBend);
  // Füge hier weitere setHandleXXX für andere Nachrichtentypen hinzu, die du sehen möchtest

  // Starte das Lauschen auf MIDI-Nachrichten (auf allen Kanälen)
  midiIn.begin(MIDI_CHANNEL_OMNI);

  Serial.println("--- Ready to receive MIDI ---");
}

void loop() {
  // MIDI-Nachrichten lesen und die zugewiesenen Callback-Funktionen aufrufen
  midiIn.read();
  
  // Kein Delay hier, damit MIDI.read() so oft wie möglich aufgerufen wird.
}
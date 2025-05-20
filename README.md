# ESP32-S3 Polyphoner Wavetable-Synthesizer

Dieses Projekt implementiert einen **8-stimmigen polyphonen Wavetable-Synthesizer** auf Basis des ESP32-S3 Mikrocontrollers. Der Synthesizer nutzt eine Zwei-Task-Architektur unter FreeRTOS für separate Audioverarbeitungs- und Steuerungslogik und bietet eine Vielzahl von Klanggestaltungsmöglichkeiten durch direkt angeschlossene Potentiometer und Taster.

## Hauptmerkmale

*   **Polyphonie:** Bis zu **8 gleichzeitig spielbare Stimmen**.
*   **Oszillator pro Stimme:**
    *   Ein Hauptoszillator pro Stimme.
    *   **11 auswählbare Wellenformen:**
        1.  Sine (Sinus)
        2.  Saw Down (Abfallender Sägezahn)
        3.  Square (Rechteck, 50% Puls)
        4.  Triangle (Dreieck)
        5.  Pulse 25% (Schmale Pulswelle)
        6.  Pulse 12% (Sehr schmale Pulswelle)
        7.  Sync Saw (Klangcharakter ähnlich Oszillator-Synchronisation)
        8.  Formant A (Vokalähnlicher Formantklang)
        9.  Ramp Up (Ansteigender Sägezahn)
        10. Hollow Square (Rechteckwelle mit reduzierten Obertönen)
        11. Formant O (Weiterer vokalähnlicher Formantklang)
    *   Wellenformauswahl über einen dedizierten Taster (angeschlossen an GPIO19).
*   **Hüllkurve (ADSR) pro Stimme:**
    *   Attack, Decay, Sustain, Release Parameter, steuerbar über dedizierte Potentiometer.
*   **Spannungsgesteuerter Filter (VCF) (Global):**
    *   State Variable Filter (SVF) mit Tiefpass-Ausgang.
    *   Steuerbare Cutoff-Frequenz und Resonanz über Potentiometer.
    *   Exponentielles Ansprechverhalten für den Cutoff-Regler.
*   **Low-Frequency Oscillator (LFO):**
    *   Ein globaler LFO (Sinuswelle).
    *   Steuerbare Rate über Potentiometer.
    *   Modulationsziele:
        *   **Pitch-Modulation:** Tiefe steuerbar über ein Poti (Basis-Tiefe) und ein weiteres Poti als Modulationsrad-Äquivalent (Intensität).
        *   **Filter-Cutoff-Modulation:** Tiefe steuerbar über ein Potentiometer.
*   **Performance-Kontrollen:**
    *   **Pitchbend:** Steuerbar über ein Potentiometer (z.B. +/- 2 Halbtöne, an GPIO36).
    *   **Modulationsrad:** Steuerbar über ein Potentiometer (an GPIO39), moduliert die Intensität der LFO-zu-Pitch Modulation.
    *   **Master Volume:** Globale Lautstärkeregelung über ein Potentiometer (an GPIO12).
*   **Arpeggiator:**
    *   Integrierter Arpeggiator mit verschiedenen Modi:
        *   Off
        *   Up (Aufsteigend)
        *   Down (Absteigend)
        *   Up/Down (Auf- und Absteigend)
        *   Random (Zufällig)
    *   Arpeggiator An/Aus schaltbar über Taster (an GPIO18).
    *   Arpeggiator-Modus wählbar über Taster (an GPIO17).
    *   Feste Arpeggiator-Rate (im Code definiert auf `ARP_STEP_INTERVAL_MS`).
*   **Audioausgabe:**
    *   Stereo-Audioausgabe über I2S bei einer Samplerate von 22050 Hz.
    *   16-Bit Audioauflösung.
*   **MIDI-Eingang:**
    *   Empfängt MIDI Note On, Note Off und andere Standard-MIDI-Nachrichten über `Serial2` (RX an GPIO3).
    *   Verarbeitet Velocity-Informationen.
*   **Benutzeroberfläche (Hardware):**
    *   Direkte Steuerung vieler Parameter über insgesamt 11 Potentiometer.
    *   Taster für Wellenformauswahl, Arpeggiator An/Aus und Arpeggiator-Modus.
*   **Display:**
    *   Unterstützung für ein SSD1306 I2C OLED-Display (128x64 Pixel).
    *   Zeigt die aktuell ausgewählte Wellenform und den Arpeggiator-Status an.
*   **Software-Architektur:**
    *   FreeRTOS mit zwei dedizierten Tasks für Audioverarbeitung (hohe Priorität) und Steuerung/MIDI/Display (niedrigere Priorität).
    *   Mutex (`xSemaphoreHandle`) zur Synchronisation des Zugriffs auf geteilte Voice-Daten.
    *   Software-Debouncing für Taster.
    *   Deadzone-Implementierung für Potentiometer zur Stabilisierung der Nullstellung und an den Endanschlägen.

## Technische Basis

*   **Mikrocontroller:** ESP32-S3
*   **Entwicklungsumgebung:** Arduino IDE mit ESP32 Core
*   **Hauptbibliotheken:**
    *   `driver/i2s_std.h` (ESP-IDF für I2S)
    *   `MIDI.h` (Arduino MIDI Library)
    *   `Wire.h`, `Adafruit_GFX.h`, `Adafruit_SSD1306.h` (für OLED Display)
    *   FreeRTOS (integriert im ESP-IDF)

## Mögliche zukünftige Erweiterungen

*   Poti-Steuerung oder MIDI-CC-Steuerung für die Arpeggiator-Rate.
*   Speichern und Laden von Presets (z.B. auf SPIFFS oder SD-Karte).
*   Weitere Effekte (z.B. Delay, Chorus – mit Bedacht auf CPU-Last).
*   Dynamische Pulsweitenmodulation (PWM) für Puls-Wellenformen.
*   Mehr Modulationsziele für den LFO oder zusätzliche LFOs.
*   Erweiterte Display-Informationen (z.B. grafische Darstellung der Hüllkurve, Parameterwerte).

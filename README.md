# ESP32-S3 Polyphoner Wavetable-Synthesizer

Dieses Projekt implementiert einen **8-stimmigen polyphonen Wavetable-Synthesizer** auf Basis des ESP32-S3 Mikrocontrollers. Der Synthesizer nutzt eine Zwei-Task-Architektur unter FreeRTOS für separate Audioverarbeitungs- und Steuerungslogik und bietet eine Vielzahl von Klanggestaltungsmöglichkeiten durch direkt angeschlossene Potentiometer und Taster. Die Audioausgabe erfolgt über ein DAC PCM5102. Das Projekt wurde in "enger Zusammenarbeit mit Google Studio AI" entwickelt.

## Hauptmerkmale

*   **Polyphonie:** Bis zu **8 gleichzeitig spielbare Stimmen** in der Variante mit 22050Khz Sample Rate
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
    * ARP_UP`, `ARP_DOWN`, `ARP_UP_DOWN`, `ARP_RANDOM` (bestehend)
    *   ARP_OCT_UP`: Spielt das gewählte Pattern (Basis: Up) über mehrere Oktaven aufsteigend.
    *   ARP_OCT_DOWN`: Spielt das gewählte Pattern (Basis: Up) über mehrere Oktaven absteigend.
    *   ARP_CHORD`: Spielt alle gehaltenen Noten gleichzeitig als Akkord.
    *   ARP_ORDER_PLAYED`: Spielt Noten in der Reihenfolge ihres Anschlags.
    *   Geschwindigkeitsvarianten (`_DOUBLE`, `_HALF`) für Up, Down und Up/Down Pattern.
    *   Arpeggiator An/Aus schaltbar über Taster (an GPIO18).
    *   Arpeggiator-Modus wählbar über Taster (an GPIO17).
    *   Feste Arpeggiator-Rate (im Code definiert auf `ARP_STEP_INTERVAL_MS`).
*   **Audioausgabe:**
    *   Stereo-Audioausgabe über I2S bei einer Samplerate von 22050 Hz an einem DAC PCM5102 Modul.
    *   16-Bit Audioauflösung.
*   **MIDI-Eingang:**
    *   Empfängt MIDI Note On, Note Off und andere Standard-MIDI-Nachrichten über `Serial2` (RX an GPIO3).
    *   Verarbeitet Velocity-Informationen.
    *   Einfaches Midi-In Modul realisiert über eine 5pol. DIN Buchse an einem Optokoppler 6N137. Buchse Pin4 an Pin2 an 220Ohm Widerstand, Buchse Pin 5 an Pin 3.
*   **Benutzeroberfläche (Hardware):**
    *   Direkte Steuerung vieler Parameter über insgesamt 11 Potentiometer (10k).
    *   Taster für Wellenformauswahl, Arpeggiator An/Aus und Arpeggiator-Modus.
*   **Display:**
    *   Unterstützung für ein SSD1306 I2C OLED-Display (128x64 Pixel).
    *   Zeigt die aktuell ausgewählte Wellenform und den Arpeggiator-Status an.
*   **Software-Architektur:**
    *   FreeRTOS mit zwei dedizierten Tasks für Audioverarbeitung (hohe Priorität) und Steuerung/MIDI/Display (niedrigere Priorität).
    *   Mutex (`xSemaphoreHandle`) zur Synchronisation des Zugriffs auf geteilte Voice-Daten.
    *   Software-Debouncing für Taster.
    *   Deadzone-Implementierung für Potentiometer zur Stabilisierung der Nullstellung und an den Endanschlägen.
## Erweiterter Code mit SampleRate 44,1Khz 16bit
   *gleicher Funktionsumfang, aber nur 3 stimmig polyphon

## Technische Basis

*   **Mikrocontroller:** ESP32-S3 (Freenove ESP32 WROOM Board)
*   PCM5102 DAC
*   Midi-In Shield
*   OLED - SDA,SCL Standard Pin 21, Pin22
*   Einschalter und Power Buchse extern 3,3Volt
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

  ## Codegenerierung und Prüfung mit KI
  *Google Studio AI in der Gratisversion setzte die Anforderungen in Code um
  *Letztlich wurde eine spielbare Version erzeugt mit den oben benannten Merkmalen

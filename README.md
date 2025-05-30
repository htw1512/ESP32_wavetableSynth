# ESP32-S3 Polyphoner Wavetable-Synthesizer

Dieser vielseitige polyphone Synthesizer basiert auf dem ESP32-S3 Mikrocontroller und bietet eine breite Palette an Klangerzeugungsmöglichkeiten. Er verfügt über eine umfangreiche Sammlung an Wavetables, eine vollständige ADSR-Hüllkurve, ein State-Variable-Filter, einen modulierbaren LFO und einen komplexen Arpeggiator mit zahlreichen Modi, einschließlich Optionen für synchronisierte Tonika-Noten. Die Steuerung erfolgt über Potentiometer und Taster, mit visueller Rückmeldung über ein OLED-Display. MIDI-Eingabe wird über eine traditionelle 5-Pol-DIN-Buchse realisiert.

## Kernmodule des Synthesizers

*   **Oszillatoren**:
    *   Bis zu 51 verschiedene digital generierte Wellenformen pro Stimme.
    *   Auswahl umfasst klassische Formen (Sinus, Sägezahn, Rechteck, Dreieck), Pulsbreitenvarianten, Sync-Sounds, Formant-ähnliche Wellen und diverse experimentelle sowie perkussive und klavierähnliche Texturen.
    *   Auswahl der Wellenform über einen dedizierten Taster.
*   **Hüllkurve**:
    *   Eine ADSR-Hüllkurve (Attack, Decay, Sustain, Release) pro Stimme zur Formung der Lautstärke.
    *   Parameter steuerbar über dedizierte Potentiometer.
*   **Filter**:
    *   Ein globales State-Variable-Filter (SVF), dessen Tiefpass-Ausgang genutzt wird.
    *   Cutoff-Frequenz und Resonanz sind über Potentiometer regelbar.
*   **LFO (Low-Frequency Oscillator)**:
    *   Ein globaler LFO (Sinuswelle).
    *   Kann die Tonhöhe (Pitch) und/oder die Filter-Cutoff-Frequenz modulieren.
    *   LFO-Rate und die Modulationstiefe für Filter und Pitch (letztere oft in Kombination mit einem Modulationsrad-Poti) sind über Potentiometer einstellbar.
*   **Arpeggiator**:
    *   Umfangreicher Arpeggiator mit diversen Modi und Steuerungen:
        *   **Grund-Pattern**: Up, Down, Up/Down (mit korrekter Notenwiederholung), Random.
        *   **Oktav-Pattern**: Oct Up (spielt Pattern über mehrere Oktaven aufwärts), Oct Down (abwärts).
        *   **Chord-Modus**: Spielt alle gehaltenen Noten gleichzeitig im Arp-Rhythmus.
        *   **Played Order**: Spielt Noten in der Reihenfolge ihres Anschlags.
        *   **Geschwindigkeits-Modi**: Doppel- und Halbgeschwindigkeitsvarianten für Up, Down und Up/Down Pattern.
        *   **Tonika-Modi**: Varianten von Up, Down, Up/Down und Random, bei denen zusätzlich die tiefste gehaltene Note (Tonika) auf jedem Arp-Step mitgespielt wird.
    *   Steuerung über Taster für An/Aus und Moduswahl.
*   **MIDI-Eingang**:
    *   Empfang von MIDI-Daten (Note On, Note Off, Velocity) über eine traditionelle 5-Pol-DIN-Buchse, die über eine Optokoppler-Schaltung (6N137) an einen Hardware-Serial-Port des ESP32-S3 angebunden ist.
*   **Steuerung & Interaktion**:
    *   Analoge Potentiometer für Echtzeitkontrolle der meisten Syntheseparameter.
    *   Dedizierte Taster für Wellenformauswahl, Arp An/Aus und Arp-Moduswahl.
    *   Pitchbend- und Modulationsrad-Funktionalität über Potentiometer.
*   **Display**:
    *   0,96 Zoll OLED-Display (SSD1306-basiert, 128x64 Pixel) zur Anzeige der aktuellen Parameter, angebunden über I2C (SDA, SCL Pins).
*   **Audioausgabe**:
    *   Stereo-Audioausgabe über einen externen PCM5102A Digital-Analog-Wandler (DAC), der über die I2S-Schnittstelle des ESP32-S3 angesteuert wird.

## Hardware-Aufbau

*   **Mikrocontroller-Board**: Freenove ESP32-S3-WROOM-1 Development Board (oder ein vergleichbares ESP32-S3 Board).
*   **Digital-Analog-Wandler (DAC)**: PCM5102A I2S Stereo DAC-Modul für die Audioausgabe.
*   **MIDI-Eingang**:
    *   Standard 5-Pol-DIN-Buchse.
    *   Optokoppler-Schaltung mit einem 6N137 (oder äquivalent) zur galvanischen Trennung und Pegelanpassung des MIDI-Signals für den ESP32-S3 GPIO (typischerweise RX-Pin eines Hardware-UARTs).
*   **Display**:
    *   0,96 Zoll monochromes OLED-Display mit SSD1306 Controller.
    *   Auflösung: 128x64 Pixel.
    *   Anbindung über I2C-Bus (SDA- und SCL-Pins des ESP32-S3).
*   **Bedienelemente**:
    *   Diverse lineare Potentiometer (typischerweise 10k Ohm) für Parametersteuerung.
    *   Taster (Push Buttons) mit Pull-Up-Widerständen (intern oder extern) für Menü- und Funktionsauswahl.
*   **Stromversorgung**:
    *   Typischerweise über den USB-Anschluss des ESP32-S3 Boards oder
    *   Über ein externes Netzteil und einem Schalter ("Power-In Schalter") wird die 3.3V-Versorgung für das gesamte Board bereitgestellt.

## Konfigurationen & Performance

Der Synthesizer kann in zwei primären Konfigurationen betrieben werden, die einen Kompromiss zwischen Audioqualität/Polyphonie und CPU-Last darstellen:

### Version 1: "Lo-Fi / High Polyphony" (Beispielkonfiguration)

*   **Sample Rate**: `22050 Hz`
*   **Aktive Stimmen (Polyphonie)**: `8 Stimmen`
*   **Charakteristik**: Diese Konfiguration ermöglicht eine höhere Anzahl gleichzeitig spielbarer Noten, was besonders für komplexe Akkorde oder dichte Arpeggios von Vorteil ist. Die niedrigere Sample Rate kann zu einem "Lo-Fi"-Charakter mit weniger Brillanz in den höchsten Frequenzen und potenziell mehr Aliasing bei sehr hohen Noten führen, was aber auch als klangästhetisches Merkmal geschätzt werden kann.

### Version 2: "Hi-Fi / Focused Polyphony" (Aktuelle Konfiguration im Code)

*   **Sample Rate**: `44100 Hz`
*   **Aktive Stimmen (Polyphonie)**: `3 Stimmen`
*   **Charakteristik**: Bietet eine höhere Audioqualität mit besserer Abbildung hoher Frequenzen und weniger Aliasing-Artefakten. Die reduzierte Stimmenanzahl stellt sicher, dass die CPU-Last auch bei der höheren Sample Rate im Rahmen bleibt. Dies ist ideal für klarere Lead-Sounds, Bässe oder weniger dichte Arpeggios, bei denen die Klangqualität jeder einzelnen Note im Vordergrund steht. Bei den Arp-Modi mit Tonika (die 2 Stimmen pro Step benötigen) ist die effektive Polyphonie für ausklingende Noten weiter reduziert.

## Technische Basis

*   **Mikrocontroller**: ESP32-S3 (im Freenove Development Board)
*   **Programmierung**: Arduino Framework mit FreeRTOS für Multitasking.
*   **Wellenform-Speicher**: Intern generierte Wavetables im SRAM

  ## Hinweis zum Code Upload über die Arduino IDE
  * Die Auswahl des Boards in der Arduino IDE hängt von der Verwendung des jeweiligen ESP32Board ab.
  * in Abhängigkeit vom verwendeten Boards müssen beim Upload Verbindungen zu bestimmten Pins getrennt werden. im vorliegenden Fall wurde das über Jumper gelöst.

  ## Codegenerierung und Prüfung mit KI
  *Google Studio AI in der Gratisversion setzte die Anforderungen in Code um
  *Letztlich wurde eine spielbare Version erzeugt mit den oben benannten Merkmalen

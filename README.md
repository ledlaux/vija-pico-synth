# VIJA synthesizer

Raspberry PICO digital synthesizer based on **Mutable Instruments Braids** macro oscillator 
in semi-modular format.  

---

## 🚀 Features

* **40+ Oscillator Engines:** Includes VA, FM, Additive, Wavetable, Physical Modeling and Drums.
* **Polyphony:** Per-sample AR (Attack-Release) envelopes.
* **OLED Interface:** Real-time feedback with a menu system and a oscilloscope.
* **Modulation:** 2 pots, CV input and MIDI CC.
* **Internal Filter:** Integrated State Variable Filter (SVF) with Low-Pass and Resonance.
* **MIDI:** Support for both USB MIDI and classic UART MIDI.
* **Arpeggiator** with MIDI sync.

---

## 🕹 Menu System & UI

The synthesizer operates in three primary display modes:

1. **ENGINE SELECT:** Rotate the encoder to scroll through engines  

2. **SETTINGS:** Click the encoder button to cycle through parameters  
   * **VOLUME:** Global gain control  
   * **A/R ENVELOPES:** Adjust the "snappiness" (Attack) or "fade" (Release) of notes  
   * **FILTER:** Enable/Disable the Filter  

   * **ARPEGGIATOR:**  
     - **ARP:** Enable/Disable arpeggiator  
     - **MODE:** UP / DOWN / UPDN / PLAY / RAND  
     - **DIV:** Note division (e.g. 1/4, 1/8, triplet, dotted)  
     - **OCT:** Octave range  
     - **LATCH:** Hold notes after release  

   * **CV:** CV modulation mode  
   * **MIDI:** Enable MIDI CC and disable CV  
   * **MIDI CH:** Set the input channel (1–16)  

   * **OSCILLOSCOPE:** Toggle On / Off  
   * **SAVE SETTINGS:** Long press to save menu settings  
   * **EXIT MENU:**  Double click 

3. **OSCILLOSCOPE:** Automatically engages after 10 seconds to visualize the current waveform  
### Filter Mode (Default)

- Timbre & Color (default)  
- CV1 & CV2 → Filter cutoff & resonance

### CV Modulation Mode1

- Timbre & Color POTS → Modulation depth  
- CV1 & CV2 →  Modulation source

### CV Modulation Mode2

- CV1 → Engine selection
- CV2 → FM MOD
  
### MIDI Modulation Mode

- Timbre & Color (Soft takeover mode)

  Align coresponding MIDI CC with Timbre or Color POT value to release or vice versa (screen indicator)
    
- CV1 & CV2 → Free for future functions
  
### All Modes OFF

- Timbre & Color (default)  

---

## 📟 MIDI Implementation (CC Chart)

VIJA responds to the following Control Change (CC) messages on the selected MIDI Channel:

| CC # | Parameter |
| :--- | :--- |
| **7** | Master Volume |
| **8** | Engine Select |
| **9** | Timbre |
| **10** | Color |
| **11** | Envelope Attack |
| **12** | Envelope Release |
| **15** | FM Modulation |
| **16** | Timbre Modulation Amount |
| **17** | Color Modulation Amount |
| **64** | Sustain (Hold notes) |
| **71** | Filter Resonance |
| **74** | Filter Cutoff |

---

## 💻 Software Setup

1.  **Arduino IDE:** Install the [Earle Philhower Pico Core](https://github.com/earlephilhower/arduino-pico)
2.  **Libraries:**

- arduinoMI project (ported Mutable Instruments libraries)
  - STMLIB  https://github.com/poetaster/STMLIB  
  - BRAIDS  https://github.com/poetaster/BRAIDS  

- I2S
 
- Adafruit TinyUSB

- Adafruit SSD1306 or SH110X display

- LittleFS & ArduinoJson for saving settings

3.  **Compilation Settings:**
        
   * **RP2040:**  
              - Flash size: 2MB (Sketch:1MB, FS:1MB)  
              - Optimize Even More (-O3)       
              - CPU Speed: 240mhz (Overclock)    
              - Sample rate: 32000 (4 voices) / 44100 (3 voices)  
              - USB stack: Adafruit TinyUSB 
     
   * **RP2350:**  
              - Flash size: 4MB (Sketch:1MB, FS:3MB)  
              - Optimize Even More (-O3)  
              - Sample rate: 48000  
              - USB stack: Adafruit TinyUSB 
---

## ⚡ Schematic & Wiring

For this project RP2040 Zero model was used, so adjust GPIO numbers for your board.

### 1. Audio Output (I2S DAC)
Connect your **PCM5102** or similar I2S DAC:
* **VCC/VIN** -> Pico 3.3V (Pin 36)
* **GND** -> Pico GND
* **DIN (DATA)** -> Pico GP9
* **BCK (BCLK)** -> Pico GP10 
* **LCK (LRCK)** -> Pico GP11 

### 2. Control & Display
* **OLED SDA** -> Pico GP4 
* **OLED SCL** -> Pico GP5
* **Encoder SW** -> Pico GP6
* **Encoder DT** -> Pico GP7 
* **Encoder CLK** -> Pico GP8

### 3. Potentiometers (ADC)
Connect the outer pins to 3.3V and GND, and the center wiper to:
* **Pot 1 (Timbre)** -> GP26 
* **Pot 2 (Color)** -> GP27 
* **Pot 3 (CV1)** -> GP28 
* **Pot 4 (CV2)** -> GP29 

### 4. MIDI Input (UART)
Connect your MIDI Jack via a 6N138 optocoupler circuit to **GP1**.

---

## 📜 License
* (c) 2025 Vadims Maksimovs - GPLv3
 
* **Core Libraries:** stmlib/braids - MIT License (Copyright 2020 Emilie Gillet)
* **Ported code:** stmlib/braids - MIT License (Copyright 2025 Mark Washeim)
  
---
##  Version history
* 2026-04-20 - v1.0.3 (+ arpeggiator)
* 2026-02-26 - v1.0.2 
* 2026-02-03 - v1.0.1  
* 2026-02-02 - First release v1.0

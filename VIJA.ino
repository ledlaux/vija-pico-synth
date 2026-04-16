/*
  VIJA (v1.0.3) 

  Copyright (c) 2025 Vadims Maksimovs ledlaux.github.com | GPLv3
  
  Raspberry PICO polyphonic synthesizer based on Mutable Instruments Braids macro oscillator 
  in semi-modular format.

  Features:
  - 40+ digital oscillator engines
  - Polyphonic, per-sample AR envelopes
  - USB or UART MIDI input
  - Filter (SVF)
  - OLED display with menu system & oscilloscope
  - Synth controls via potentiometers or MIDI CC
  - Arpeggiator (midi sync)
  
  Hardware:
  - RP2040 or RP2350 board, I2S PCM5102 DAC, SSD1306 OLED, rotary encoder with button, 2 pots, 2 cv jacks or 2 more pots
  - MIDI via USB or UART

  For this project I use RP2040 Zero model, so adjust GPIO numbers to your board.

  Compilation:

  RP2040: - Optimize: Optimize Even More (-O3)
          - CPU Speed: 200-240mhz (Overclock) depending on the sample rate and needed voice count   
          - Sample rate: 32000 (4 voices) / 44100 (3 voices)  
  RP2350:
         - Optimize: Optimize Even More (-O3)
         - Sample rate: 48000
  
  Software:
 - BRAIDS and STMLIB libraries ported by Mark Washeim:
  https://github.com/poetaster/arduinoMI (MIT License)

  stmlib, braids source libs
  Copyright (c) 2020 (emilie.o.gillet@gmail.com)
  MIT License
  
*/

#include <Arduino.h>
#include <I2S.h>
#include <Adafruit_TinyUSB.h>
#include <STMLIB.h>
#include <BRAIDS.h>
#include <pico/stdlib.h>
#include <Wire.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#define SSD1306 1  //ssd1306 display
// #define SH110X 1// sh110x display

#if SSD1306
#include <Adafruit_SSD1306.h>
#define SCREEN_WHITE SSD1306_WHITE
#endif

#if SH110X
#include <Adafruit_SH110X.h>
#define SCREEN_WHITE SH110X_WHITE
#endif

#define I2S_DATA_PIN 9
#define I2S_BCLK_PIN 10
#define SAMPLE_RATE 48000
#define AUDIO_BLOCK 32
#define MAX_VOICES 4

#define USE_POTS 0
#define POT_TIMBRE A0      // GPIO26
#define POT_COLOR A1       // GPIO27
#define POT_TIMBRE_MOD A2  // GPIO28
#define POT_COLOR_MOD A3   // GPIO29

#define ENCODER_SW 13
#define ENCODER_DT 14
#define ENCODER_CLK 15
#define BUTTON_DEBOUNCE_MS 200
#define LONG_PRESS_MS 1000

#define USE_UART_MIDI 0  // 0 = USB MIDI, 1 = UART MIDI
#define MIDI_UART_RX 1

#define USE_SCREEN 1
#define OLED_SDA 4
#define OLED_SCL 5
#define SCOPE_WIDTH 128

// Splash screen
const unsigned char waveform_bitmap[] PROGMEM = {
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00001100, 0b00110000, 0b00001100, 0b00110000,
  0b00011110, 0b01111000, 0b00011110, 0b01111000,
  0b00111111, 0b11111100, 0b00111111, 0b11111100,
  0b01111111, 0b11111110, 0b01111111, 0b11111110,
  0b11111111, 0b11111111, 0b11111111, 0b11111111,
  0b11111111, 0b11111111, 0b11111111, 0b11111111,
  0b01111111, 0b11111110, 0b01111111, 0b11111110,
  0b00111111, 0b11111100, 0b00111111, 0b11111100,
  0b00011110, 0b01111000, 0b00011110, 0b01111000,
  0b00001100, 0b00110000, 0b00001100, 0b00110000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000
};

const char *SETTINGS_FILE = "/vija_settings.json";

enum DisplayMode { ENGINE_SELECT_MODE,
                   SETTINGS_MODE,
                   OSCILLOSCOPE_MODE };

enum EncoderState { ENGINE_SELECT,
                    VOLUME_ADJUST,
                    ATTACK_ADJUST,
                    RELEASE_ADJUST,
                    ARP_TOGGLE,
                    ARP_MODE,
                    ARP_DIV,
                    ARP_LATCH,
                    FILTER_TOGGLE,
                    MIDI_MOD,
                    CV_MOD1,
                    CV_MOD2,
                    MIDI_CH,
                    SCOPE_TOGGLE };


#define MAX_ARP_NOTES 12

enum ArpDirection { UP,
                    DOWN,
                    UP_DOWN,
                    RANDOM,
                    CHORD,
                    AS_PLAYED };

volatile uint8_t midi_ch = 1;
volatile int engine_idx = 1;
static int last_engine_idx = -1;
volatile float timbre_in = 0.4f;
volatile float color_in = 0.3f;
volatile float fm_mod = 0.0f;
volatile float timb_mod_midi = 0.0f;
volatile float color_mod_midi = 0.0f;
volatile float timb_mod_cv = 0.0f;
volatile float color_mod_cv = 0.0f;
volatile float fm_target = 0.0f;

volatile float master_volume = 0.7f;
volatile float env_attack_s = 0.009f;
volatile float env_release_s = 0.01f;
static float attackCoef = 0.0f;
static float releaseCoef = 0.0f;
volatile bool sustain_enabled = false;

volatile bool engine_updated = true;
volatile bool env_params_changed = true;
volatile unsigned long last_param_change = 0;
unsigned long last_midi_lock_time = 0;

volatile bool midi_mod = false;
volatile bool cv_mod1 = false;
volatile bool cv_mod2 = false;

volatile float timbre_midi_target = 0.0f;
volatile float color_midi_target = 0.0f;
volatile bool timbre_locked = false;
volatile bool color_locked = false;

volatile bool filter_enabled = true;
volatile float filter_mix = 1.0f;
volatile uint8_t filter_cutoff_cc = 64;
volatile uint8_t filter_resonance_cc = 32;

volatile bool save_requested = false;
static bool show_saved_flag = false;
static unsigned long saved_start_time = 0;
static const unsigned long SAVED_DISPLAY_MS = 800;

volatile bool oscilloscope_enabled = true;
volatile float scope_buffer_front[SCOPE_WIDTH];
volatile float scope_buffer_back[SCOPE_WIDTH];

const unsigned long AUTO_REVERT_MS = 4000;
volatile unsigned long last_encoder_activity = 0;
volatile DisplayMode display_mode = ENGINE_SELECT_MODE;
volatile EncoderState enc_state = ENGINE_SELECT;

volatile bool system_ready = false;

volatile bool arp_latch_enabled = false;
volatile int physicalKeysHeld = 0;

// For UI updates
float pot_timbre = 0.5f;
float pot_color = 0.5f;

#if USE_SCREEN
static int last_engine_draw = -1;
static unsigned long last_draw_time = 0;
static int lBtn = HIGH;
#endif

volatile float scope_buffer[SCOPE_WIDTH];
volatile bool scope_ready = false;

struct Voice {
  braids::MacroOscillator osc;
  int pitch;
  float velocity;
  float vel_smoothed;
  bool active;
  bool last_trig;
  float env;
  int16_t buffer[AUDIO_BLOCK];
  uint8_t sync_buffer[AUDIO_BLOCK];
  uint32_t age;
  bool sustained;
};

struct SynthSettings {
  float master_volume;
  float env_attack_s;
  float env_release_s;
  int engine_idx;
  bool filter_enabled;
  bool midi_mod;
  bool cv_mod1;
  bool cv_mod2;
  float timbre_in;
  float color_in;
  float timb_mod_cv;
  float color_mod_cv;
  uint8_t midi_ch;
  EncoderState enc_state;
  bool oscilloscope_enabled;
};

// Default settings for the first run
SynthSettings settings = {
  .master_volume = 0.7f,
  .env_attack_s = 0.009f,
  .env_release_s = 0.01f,
  .engine_idx = 1,
  .filter_enabled = true,
  .midi_mod = false,
  .cv_mod1 = false,
  .cv_mod2 = false,
  .timbre_in = 0.4f,
  .color_in = 0.3f,
  .timb_mod_cv = 0.0f,
  .color_mod_cv = 0.0f,
  .midi_ch = 1,
  .enc_state = ENGINE_SELECT,
  .oscilloscope_enabled = true
};

Voice voices[MAX_VOICES];
uint32_t global_age = 0;
float attack_coeff_cached, release_coeff_cached;

I2S i2s_output(OUTPUT);

braids::Svf global_filter;

Adafruit_USBD_MIDI usb_midi;

SynthSettings lastSavedSettings;  // Settings copy for comparison of changes

#if USE_SCREEN

#if SSD1306
Adafruit_SSD1306 display(128, 64, &Wire, -1);
#endif

#if SH110X
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);
#endif

#endif

const char *const engine_names[] = {
  "CSAW", "/\\-_", "//-_", "FOLD", "uuuu", "SUB-", "SUB/", "SYN-", "SYN/",
  "//x3", "-_x3", "/\\x3", "SIx3", "RING", "////", "//uu", "TOY*", "ZLPF", "ZPKF",
  "ZBPF", "ZHPF", "VOSM", "VOWL", "VFOF", "HARM", "-FM-", "FBFM", "WTFM",
  "PLUK", "BOWD", "BLOW", "FLUT", "BELL", "DRUM", "KICK", "CYMB", "SNAR",
  "WTBL", "WMAP", "WLIN", "WTx4", "NOIS", "TWNQ", "CLKN", "CLOU", "PRTC",
  "QPSK", "????"
};

constexpr int NUM_ENGINES = sizeof(engine_names) / sizeof(engine_names[0]);

struct Arpeggiator {
  volatile bool enabled = false;
  volatile ArpDirection mode = AS_PLAYED;
  volatile uint8_t division = 6;

  int heldNotes[MAX_ARP_NOTES];
  int sortedNotes[MAX_ARP_NOTES];
  volatile int numHeld = 0;

  int currentStep = -1;
  // --- FIX: Add this line below ---
  int lastVoiceIndices[MAX_VOICES];
  int lastPitchCount = 0;
  bool directionUp = true;
  uint32_t tickCounter = 0;

  void addNote(int pitch) {
    if (numHeld < MAX_ARP_NOTES) {
      heldNotes[numHeld++] = pitch;
      updateSort();
    }
  }

  void removeNote(int pitch) {
    for (int i = 0; i < numHeld; i++) {
      if (heldNotes[i] == pitch) {
        for (int j = i; j < numHeld - 1; j++) heldNotes[j] = heldNotes[j + 1];
        numHeld--;
        updateSort();
        break;
      }
    }
    if (numHeld == 0) currentStep = -1;
  }

  void updateSort() {
    for (int i = 0; i < numHeld; i++) sortedNotes[i] = heldNotes[i];
    for (int i = 0; i < numHeld - 1; i++) {
      for (int j = 0; j < numHeld - i - 1; j++) {
        if (sortedNotes[j] > sortedNotes[j + 1]) {
          int temp = sortedNotes[j];
          sortedNotes[j] = sortedNotes[j + 1];
          sortedNotes[j + 1] = temp;
        }
      }
    }
  }

  void clear() {
    numHeld = 0;
    currentStep = -1;
    lastPitchCount = 0;
  }
};

Arpeggiator arp;

int findFreeVoice() {
  int oldest = 0;
  uint32_t old_age = voices[0].age;
  for (int i = 0; i < MAX_VOICES; i++) {
    if (!voices[i].active && voices[i].env == 0.f)
      return i;
    if (voices[i].age < old_age) {
      old_age = voices[i].age;
      oldest = i;
    }
  }
  return oldest;
}


int findVoiceByPitch(int pitch) {
  for (int i = 0; i < MAX_VOICES; i++)
    if (voices[i].active && voices[i].pitch == pitch) return i;
  return -1;
}


void __not_in_flash_func(updateAudio)() {

  if (engine_idx != last_engine_idx) {
    braids::MacroOscillatorShape shape =
      (braids::MacroOscillatorShape)engine_idx;

    for (int v = 0; v < MAX_VOICES; v++)
      voices[v].osc.set_shape(shape);

    last_engine_idx = engine_idx;
  }

  if (env_params_changed) {
    attackCoef = 1.0f - expf(-1.0f / (SAMPLE_RATE * env_attack_s));
    releaseCoef = 1.0f - expf(-1.0f / (SAMPLE_RATE * env_release_s));
    env_params_changed = false;
  }

  float mix[AUDIO_BLOCK] = { 0 };

  static float fm_slew = 0.0f;
  static float timb_slew = 0.0f;
  static float color_slew = 0.0f;

  if (midi_mod) {
    fm_target = fm_mod;
  } else if (cv_mod1) {
    fm_target = 0.0f;
  } else if (filter_enabled) {
    fm_target = 0.0f;
  } else {
    fm_target = fm_mod;
  }

  float timb_target = midi_mod  ? timb_mod_midi
                      : cv_mod1 ? timb_mod_cv
                                : 0.0f;

  float color_target = midi_mod  ? color_mod_midi
                       : cv_mod1 ? color_mod_cv
                                 : 0.0f;

  auto apply_stable_slew = [](float &current, float target, float coefficient) {
    float diff = target - current;
    float abs_diff = fabsf(diff);

    if (abs_diff < 0.005f) {
      if (target == 0.0f && abs_diff < 0.01f) current = 0.0f;
      return;
    }

    if (abs_diff < 0.001f) {
      current = target;
    } else {
      current += diff * coefficient;
    }
  };

  apply_stable_slew(fm_slew, fm_target, 0.05f);
  apply_stable_slew(timb_slew, timb_target, 0.01f);
  apply_stable_slew(color_slew, color_target, 0.01f);

  const float block_gain = master_volume * 0.25f;

  for (int v = 0; v < MAX_VOICES; v++) {
    Voice &voice = voices[v];

    if (!voice.active && !voice.sustained && voice.env < 0.0001f)
      continue;

    voice.vel_smoothed += (voice.velocity - voice.vel_smoothed) * 0.25f;

    float pitch = voice.pitch * 128.0f + fm_slew * 1536.0f;
    voice.osc.set_pitch(pitch);

    float t = constrain(timbre_in + timb_slew, 0.0f, 1.0f);
    float m = constrain(color_in + color_slew, 0.0f, 1.0f);
    voice.osc.set_parameters(t * 32767.0f, m * 32767.0f);

    if (voice.active && !voice.last_trig)
      voice.osc.Strike();

    voice.last_trig = voice.active;
    voice.osc.Render(voice.sync_buffer, voice.buffer, AUDIO_BLOCK);

    float envTarget = (voice.active || voice.sustained) ? 1.0f : 0.0f;
    float coef = envTarget ? attackCoef : releaseCoef;

    for (int i = 0; i < AUDIO_BLOCK; i++) {
      voice.env += (envTarget - voice.env) * coef;
      if (voice.env < 0.0001f) voice.env = 0.0f;

      mix[i] += (voice.buffer[i] * 0.000030517578125f) * (voice.env * voice.vel_smoothed * block_gain);
    }
  }


  static int scope_idx = 0;
  static float scopeSmooth = 0.0f;
  if (oscilloscope_enabled && !scope_ready) {
    for (int i = 0; i < AUDIO_BLOCK; i += 4) {
      scopeSmooth += (mix[i] - scopeSmooth) * 0.25f;
      scope_buffer_front[scope_idx++] = scopeSmooth;
      if (scope_idx >= SCOPE_WIDTH) {
        memcpy((void *)scope_buffer_back,
               (const void *)scope_buffer_front,
               sizeof(scope_buffer_back));
        scope_ready = true;
        scope_idx = 0;
        break;
      }
    }
  }

  static float cut_slew = 0.0f;
  static float res_slew = 0.0f;
  static float mix_slew = 0.0f;

  float cut_t = filter_cutoff_cc * (32767.0f / 127.0f);
  float res_t = filter_resonance_cc * (32767.0f / 127.0f);
  float mix_t = filter_enabled ? 1.0f : 0.0f;

  cut_slew += (cut_t - cut_slew) * 0.05f;
  res_slew += (res_t - res_slew) * 0.05f;
  mix_slew += (mix_t - mix_slew) * 0.01f;

  global_filter.set_frequency((uint16_t)cut_slew);
  global_filter.set_resonance((uint16_t)res_slew);

  const float dry_scale = (1.0f - mix_slew) * 32767.0f;
  const float wet_scale = mix_slew;

  for (int i = 0; i < AUDIO_BLOCK; i++) {
    float dry_f = mix[i];
    int32_t dry_int = (int32_t)(dry_f * 32767.0f);
    float wet_f = global_filter.Process(dry_int);
    float mixed_signal = (dry_f * dry_scale) + (wet_f * wet_scale);
    int16_t s = (int16_t)fmaxf(-32767.0f, fminf(32767.0f, mixed_signal));
    i2s_output.write16(s, s);
  }
}


void drawScope() {
#if USE_SCREEN
  if (!scope_ready) return;

  display.clearDisplay();

  const float midY = 40.0f;
  const float current_gain = 150.0f;

  for (int i = 0; i < SCOPE_WIDTH - 1; i++) {
    int16_t y1 = (int16_t)(midY - (scope_buffer_back[i] * current_gain));
    int16_t y2 = (int16_t)(midY - (scope_buffer_back[i + 1] * current_gain));
    if (y1 < 0) y1 = 0;
    if (y1 > 63) y1 = 63;
    if (y2 < 0) y2 = 0;
    if (y2 > 63) y2 = 63;
    display.drawLine(i, y1, i + 1, y2, SSD1306_WHITE);
  }

  display.display();
  scope_ready = false;
#endif
}


#if USE_SCREEN
void drawEngineUI() {
  if (show_saved_flag) return;  // Don't redraw while saving
  display.clearDisplay();
  const char *name = engine_names[engine_idx];
  display.setTextSize(4);
  display.setTextColor(SSD1306_WHITE);

  char idxBuf[8];
  sprintf(idxBuf, "%d", engine_idx + 1);
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(2);
  display.getTextBounds(idxBuf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(0, 15);
  display.print(idxBuf);

  display.setTextSize(4);
  display.getTextBounds(name, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(128 - w - 2, 15);
  display.println(name);

  display.setTextSize(1);
  char menuBuf[32] = "";
  switch (enc_state) {
    case VOLUME_ADJUST: sprintf(menuBuf, "VOL:%3d", int(master_volume * 100)); break;
    case ATTACK_ADJUST: sprintf(menuBuf, "A:%.2f", env_attack_s); break;
    case RELEASE_ADJUST: sprintf(menuBuf, "R:%.2f", env_release_s); break;

    case ARP_TOGGLE:
      sprintf(menuBuf, "ARP:%s", arp.enabled ? "ON" : "OFF");
      break;

    case ARP_MODE:
      {
        const char *modes[] = { "UP", "DOWN", "UPDN", "RAND", "CHRD", "PLAY" };
        sprintf(menuBuf, "MODE:%s", modes[arp.mode]);
        break;
      }

    case ARP_DIV:
      {
        const char *divStr;
        if (arp.division == 24) divStr = "1/4";
        else if (arp.division == 12) divStr = "1/8";
        else if (arp.division == 6) divStr = "1/16";
        else if (arp.division == 3) divStr = "1/32";
        else divStr = "??";
        sprintf(menuBuf, "DIV:%s", divStr);
        break;
      }

    case ARP_LATCH:
      sprintf(menuBuf, "LATCH:%s", arp_latch_enabled ? "ON" : "OFF");
      break;

    case FILTER_TOGGLE: sprintf(menuBuf, "FLT:%s", filter_enabled ? "ON" : "OFF"); break;
    case CV_MOD1: sprintf(menuBuf, "CV1:%s", cv_mod1 ? "ON" : "OFF"); break;
    case CV_MOD2: sprintf(menuBuf, "CV2:%s", cv_mod2 ? "ON" : "OFF"); break;
    case MIDI_MOD: sprintf(menuBuf, "MIDI:%s", midi_mod ? "ON" : "OFF"); break;
    case MIDI_CH:
      sprintf(menuBuf, "MIDICH:%d", midi_ch);
      engine_updated = true;
      break;
    case SCOPE_TOGGLE: sprintf(menuBuf, "SCOPE:%s", oscilloscope_enabled ? "ON" : "OFF"); break;
    default:
      if (timbre_locked && color_locked) strcpy(menuBuf, "ALL-MIDI");
      else if (timbre_locked) strcpy(menuBuf, "T-MIDI");
      else if (color_locked) strcpy(menuBuf, "C-MIDI");
      else strcpy(menuBuf, "");
      break;
  }
  if (menuBuf[0] != '\0') {
    display.setCursor(0, 55);
    display.print(menuBuf);
  }

  if (!cv_mod1) {
    char buf[16];
    int tVal = int((timbre_locked ? timbre_in : pot_timbre) * 127);
    int mVal = int((color_locked ? color_in : pot_color) * 127);
    sprintf(buf, "T:%3d C:%3d", tVal, mVal);

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(128 - w - 2, 55);
    display.print(buf);
  }
  display.display();
}


void drawSplash() {

  display.clearDisplay();
  display.drawBitmap((128 - 32) / 2, 0, waveform_bitmap, 32, 16, SSD1306_WHITE);
  const char *title = "VIJA";
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 18);
  display.println(title);
  const char *subtitle = "synthesizer";
  display.setTextSize(1);
  display.getTextBounds(subtitle, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 40);
  display.println(subtitle);
  const char *version = "v1.0.3";
  display.getTextBounds(version, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 54);
  display.println(version);
  display.display();
}
#endif


void __not_in_flash_func(triggerArpVoice)(int pitch) {
  int v = findFreeVoice();
  if (v < 0) return;

  voices[v].pitch = pitch;
  voices[v].velocity = 0.8f;
  voices[v].active = true;
  voices[v].last_trig = false;
  voices[v].age = global_age++;

  // Track the voice index
  if (arp.lastPitchCount < MAX_VOICES) {
    arp.lastVoiceIndices[arp.lastPitchCount++] = v;
  }
}

void __not_in_flash_func(advanceArp)() {
  // 1. Release previous voices by index
  for (int i = 0; i < arp.lastPitchCount; i++) {
    int v = arp.lastVoiceIndices[i];
    if (v >= 0 && v < MAX_VOICES) {
      voices[v].active = false;
    }
  }
  arp.lastPitchCount = 0;

  if (arp.numHeld == 0) return;

  // 2. Chord Mode
  if (arp.mode == CHORD) {
    for (int i = 0; i < arp.numHeld && i < MAX_VOICES; i++) {
      triggerArpVoice(arp.heldNotes[i]);
    }
  }
  // 3. Sequential Modes
  else {
    if (arp.currentStep < 0 || arp.currentStep >= arp.numHeld) arp.currentStep = 0;

    switch (arp.mode) {
      case UP:
      case AS_PLAYED:
        arp.currentStep = (arp.currentStep + 1) % arp.numHeld;
        break;
      case DOWN:
        arp.currentStep = (arp.currentStep <= 0) ? arp.numHeld - 1 : arp.currentStep - 1;
        break;
      case RANDOM:
        arp.currentStep = random(0, arp.numHeld);
        break;
      case UP_DOWN:
        if (arp.directionUp) {
          arp.currentStep++;
          if (arp.currentStep >= arp.numHeld - 1) {
            arp.currentStep = arp.numHeld - 1;
            arp.directionUp = false;
          }
        } else {
          arp.currentStep--;
          if (arp.currentStep <= 0) {
            arp.currentStep = 0;
            arp.directionUp = true;
          }
        }
        break;
    }

    // Safety bound
    if (arp.currentStep < 0) arp.currentStep = 0;
    if (arp.currentStep >= arp.numHeld) arp.currentStep = arp.numHeld - 1;

    int pitch = (arp.mode == AS_PLAYED) ? arp.heldNotes[arp.currentStep]
                                        : arp.sortedNotes[arp.currentStep];
    triggerArpVoice(pitch);
  }
}



void __not_in_flash_func(handleMIDI)() {
  static uint8_t running_status = 0;
  static uint8_t data_bytes[2] = { 0 };
  static uint8_t data_idx = 0;

  uint8_t status = 0, d1 = 0, d2 = 0;
  bool has_msg = false;

#if USE_UART_MIDI
  if (Serial1.available() > 0) {
    uint8_t byte = Serial1.read();
    if (byte >= 0xF8) {
      if (byte == 0xF8 && arp.enabled) {  // Clock
        arp.tickCounter++;
        if (arp.tickCounter >= arp.division) {
          arp.tickCounter = 0;
          advanceArp();
        }
      } else if (byte == 0xFA || byte == 0xFB) {  // Start/Continue
        arp.tickCounter = arp.division;
        arp.currentStep = -1;
      }
      return;
    }
    if (byte & 0x80) {
      running_status = byte;
      data_idx = 0;
    } else if (running_status != 0) {
      if (data_idx < 2) data_bytes[data_idx++] = byte;
      uint8_t type = running_status & 0xF0;
      uint8_t len = (type == 0xC0 || type == 0xD0) ? 1 : 2;
      if (data_idx == len) {
        status = running_status;
        d1 = data_bytes[0];
        d2 = (len == 2) ? data_bytes[1] : 0;
        data_idx = 0;
        has_msg = true;
      }
    }
  }
#endif

  uint8_t packet[4];
  if (!has_msg && usb_midi.readPacket(packet)) {
    uint8_t cin = packet[0] & 0x0F;
    if (cin == 0xF) {  // USB Real-time
      if (packet[1] == 0xF8 && arp.enabled) {
        arp.tickCounter++;
        if (arp.tickCounter >= arp.division) {
          arp.tickCounter = 0;
          advanceArp();
        }
      } else if (packet[1] == 0xFA || packet[1] == 0xFB) {
        arp.tickCounter = arp.division;
        arp.currentStep = -1;
      }
      return;
    }
    if (cin >= 0x8 && cin <= 0xE) {
      status = packet[1];
      d1 = packet[2];
      d2 = packet[3];
      has_msg = true;
    }
  }

  if (!has_msg || (status & 0x0F) != (midi_ch - 1)) return;
  uint8_t msgType = status & 0xF0;

  if (msgType == 0xB0) {
    switch (d1) {
      case 64:  // Sustain Pedal
        sustain_enabled = (d2 >= 64);
        if (!sustain_enabled && !arp.enabled) {
          for (int i = 0; i < MAX_VOICES; i++) {
            if (voices[i].sustained) {
              voices[i].active = false;
              voices[i].sustained = false;
            }
          }
        }
        break;

      case 7: master_volume = d2 / 127.f; break;
      case 8: engine_idx = map(d2, 0, 127, 0, NUM_ENGINES - 1); break;

      case 9:  // Timbre Lock
        if (midi_mod) {
          timbre_in = d2 / 127.f;
          timbre_locked = true;
          last_midi_lock_time = millis();
        }
        break;

      case 10:  // Color Lock
        if (midi_mod) {
          color_in = d2 / 127.f;
          color_locked = true;
          last_midi_lock_time = millis();
        }
        break;

      case 11: env_attack_s = 0.01f + (d2 / 127.f) * 2.f; break;
      case 12: env_release_s = 0.01f + (d2 / 127.f) * 3.f; break;
      case 71: filter_resonance_cc = d2; break;
      case 74: filter_cutoff_cc = d2; break;
    }
    engine_updated = true;
    last_param_change = millis();
    return;
  }

  bool isNoteOff = (msgType == 0x80) || (msgType == 0x90 && d2 == 0);

  if (isNoteOff) {
    if (arp.enabled) {
      if (physicalKeysHeld > 0) physicalKeysHeld--;
      // Only remove the note if we aren't latching
      if (!arp_latch_enabled) {
        arp.removeNote(d1);
      }
    } else {
      int i = findVoiceByPitch(d1);
      if (i >= 0) {
        if (sustain_enabled) voices[i].sustained = true;
        voices[i].active = false;
      }
    }
  } else if (msgType == 0x90) {  // Note On
    if (arp.enabled) {
      if (arp_latch_enabled && physicalKeysHeld == 0) {
        arp.clear();
        for (int i = 0; i < MAX_VOICES; i++) voices[i].active = false;
      }
      physicalKeysHeld++;
      arp.addNote(d1);
    } else {
      int v = findFreeVoice();
      if (v >= 0) {
        voices[v].pitch = d1;
        voices[v].velocity = d2 / 127.f;
        voices[v].active = true;
        voices[v].age = global_age++;
      }
    }
  }
}


// Saving settings
void saveSettings() {
  // 1. FAST COMPARISON: Check if memory blocks are identical
  if (memcmp(&settings, &lastSavedSettings, sizeof(SynthSettings)) == 0) {
    return;  // Exit: No changes = no click, no flash wear
  }

  if (!LittleFS.begin()) return;

  JsonDocument doc;
  doc["vol"] = settings.master_volume;
  doc["atk"] = settings.env_attack_s;
  doc["rel"] = settings.env_release_s;
  doc["eng"] = settings.engine_idx;
  doc["filt"] = settings.filter_enabled;
  doc["mod"] = settings.midi_mod;
  doc["cv1"] = settings.cv_mod1;
  doc["cv2"] = settings.cv_mod2;
  doc["timb"] = settings.timbre_in;
  doc["color"] = settings.color_in;
  doc["tcv"] = settings.timb_mod_cv;
  doc["mcv"] = settings.color_mod_cv;
  doc["ch"] = settings.midi_ch;
  doc["enc"] = (int)settings.enc_state;
  doc["osc"] = settings.oscilloscope_enabled;

  File f = LittleFS.open(SETTINGS_FILE, "w");
  if (!f) return;

  if (serializeJson(doc, f) != 0) {
    lastSavedSettings = settings;
    show_saved_flag = true;
    saved_start_time = millis();
  }
  f.close();
}


void loadSettings() {
  if (!LittleFS.begin() || !LittleFS.exists(SETTINGS_FILE)) return;

  File f = LittleFS.open(SETTINGS_FILE, "r");
  if (!f) return;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;

  settings.master_volume = doc["vol"] | 0.7f;
  settings.env_attack_s = doc["atk"] | 0.001f;
  settings.env_release_s = doc["rel"] | 0.03f;
  settings.engine_idx = doc["eng"] | 1;
  settings.filter_enabled = doc["filt"] | true;
  settings.midi_mod = doc["mod"] | false;
  settings.cv_mod1 = doc["cv1"] | false;
  settings.cv_mod2 = doc["cv2"] | false;
  settings.timbre_in = doc["timb"] | 0.4f;
  settings.color_in = doc["color"] | 0.3f;
  settings.timb_mod_cv = doc["tcv"] | 0.0f;
  settings.color_mod_cv = doc["mcv"] | 0.0f;
  settings.midi_ch = doc["ch"] | 1;
  settings.enc_state = (EncoderState)(doc["enc"] | 0);
  settings.oscilloscope_enabled = doc["osc"] | true;

  master_volume = settings.master_volume;
  env_attack_s = settings.env_attack_s;
  env_release_s = settings.env_release_s;
  engine_idx = settings.engine_idx;
  filter_enabled = settings.filter_enabled;
  midi_mod = settings.midi_mod;
  cv_mod1 = settings.cv_mod1;
  cv_mod2 = settings.cv_mod2;
  timbre_in = settings.timbre_in;
  color_in = settings.color_in;
  timb_mod_cv = settings.timb_mod_cv;
  color_mod_cv = settings.color_mod_cv;
  midi_ch = settings.midi_ch;
  enc_state = settings.enc_state;
  oscilloscope_enabled = settings.oscilloscope_enabled;

  lastSavedSettings = settings;
  engine_updated = true;
}


#if USE_SCREEN
void checkSavedFeedback() {
  if (!show_saved_flag) return;

  unsigned long now = millis();
  if (now - saved_start_time < SAVED_DISPLAY_MS) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    const char *msg = "Saved!";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((128 - w) / 2, (64 - h) / 2);
    display.println(msg);
    display.display();
  } else {
    show_saved_flag = false;
    engine_updated = true;
  }
}
#endif


void saveButton() {
  int btn = digitalRead(ENCODER_SW);
  static int last_btn_state = HIGH;
  static unsigned long button_press_start = 0;
  static bool has_saved_this_press = false;

  if (btn == LOW && last_btn_state == HIGH) {
    button_press_start = millis();
    has_saved_this_press = false;
  }

  if (btn == LOW && !has_saved_this_press) {
    if (millis() - button_press_start >= LONG_PRESS_MS) {
      // Update struct with current live values
      settings.master_volume = master_volume;
      settings.env_attack_s = env_attack_s;
      settings.env_release_s = env_release_s;
      settings.engine_idx = engine_idx;
      settings.filter_enabled = filter_enabled;
      settings.midi_mod = midi_mod;
      settings.cv_mod1 = cv_mod1;
      settings.cv_mod2 = cv_mod2;
      settings.timbre_in = timbre_in;
      settings.color_in = color_in;
      settings.timb_mod_cv = timb_mod_cv;
      settings.color_mod_cv = color_mod_cv;
      settings.midi_ch = midi_ch;
      settings.oscilloscope_enabled = oscilloscope_enabled;

      saveSettings();  // This now ONLY clicks if data changed
      has_saved_this_press = true;
    }
  }

  if (btn == HIGH && last_btn_state == LOW) {
    has_saved_this_press = false;
  }
  last_btn_state = btn;
}


void setup() {
  // Serial.begin(115200);
  bool fs_ready = false;
  if (!LittleFS.begin()) {
    //  Serial.println("LittleFS Mount Failed. Attempting to format...");
    LittleFS.format();
    if (LittleFS.begin()) {
      //    Serial.println("LittleFS Formatted and Mounted successfully.");
      fs_ready = true;
    } else {
      //   Serial.println("LittleFS Critical Error: Hardware issue or Flash size not set!");
    }
  } else {
    //  Serial.println("LittleFS Mounted.");
    fs_ready = true;
  }


  //  MIDI Initialization
  if (USE_UART_MIDI == 1) {
    Serial1.setRX(MIDI_UART_RX);
    Serial1.begin(31250);
  } else {
    TinyUSBDevice.setManufacturerDescriptor("LEDLAUX");
    TinyUSBDevice.setProductDescriptor("VIJA");
    TinyUSBDevice.setSerialDescriptor("PICO0");
    usb_midi.begin();
  }

  i2s_output.setFrequency(SAMPLE_RATE);
  i2s_output.setDATA(I2S_DATA_PIN);
  i2s_output.setBCLK(I2S_BCLK_PIN);
  i2s_output.begin();

  for (int v = 0; v < MAX_VOICES; v++) {
    voices[v].osc.Init(SAMPLE_RATE);
    voices[v].active = false;
  }

  global_filter.Init();
  global_filter.set_mode(braids::SVF_MODE_LP);
  uint16_t init_cutoff = 32767 / 4;
  uint16_t init_res = 32767 / 2;

  global_filter.set_frequency(init_cutoff);
  global_filter.set_resonance(init_res);

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();
  Wire.setClock(400000);
  loadSettings();
}


void loop() {

  if (!system_ready) {
    yield();  // Wait for Core 1 to finish the splash
    return;
  }

  if (i2s_output.availableForWrite() >= AUDIO_BLOCK * 4) {
    updateAudio();
  }
  handleMIDI();
}


void setup1() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

#if USE_SCREEN
#if SSD1306
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
#endif
#ifdef SH110X
  display.begin(0x3C, true);
#endif
  drawSplash();
  delay(4000);
  display.clearDisplay();
  display.display();
#endif

  system_ready = true;
}


void loop1() {
  saveButton();
#if USE_SCREEN
  checkSavedFeedback();
#endif

  static float smoothT = 0.5f;
  static float smoothC = 0.5f;
  static float smoothTMod = 0.0f;
  static float smoothCMod = 0.0f;
  static float smoothCut = 0.5f;
  static float smoothRes = 0.25f;
  static unsigned long last_pot_read = 0;

  if (millis() - last_pot_read > 4) {
    last_pot_read = millis();

    float rT = analogRead(POT_TIMBRE) / 1023.0f;
    float rC = analogRead(POT_COLOR) / 1023.0f;
    float srcT = analogRead(POT_TIMBRE_MOD) / 1023.0f;
    float srcC = analogRead(POT_COLOR_MOD) / 1023.0f;

    const float SMOOTH_POT = 0.06f;
    pot_timbre += (rT - pot_timbre) * SMOOTH_POT;
    pot_color += (rC - pot_color) * SMOOTH_POT;

    if (pot_timbre > 0.999f) pot_timbre = 1.0f;
    if (pot_timbre < 0.001f) pot_timbre = 0.0f;

    if (pot_color > 0.999f) pot_color = 1.0f;
    if (pot_color < 0.001f) pot_color = 0.0f;

    int valT = (int)(pot_timbre * 127.0f + 0.5f);
    int valC = (int)(pot_color * 127.0f + 0.5f);


    if (!midi_mod) {
      timbre_locked = false;
      color_locked = false;
      engine_updated = true;
    }


    if (cv_mod1) {

      // --- Smooth the potentiometer inputs (depth controls) ---
      smoothT += (rT - smoothT) * 0.15f;
      smoothC += (rC - smoothC) * 0.15f;

      // --- Smooth the modulation sources ---
      smoothTMod += (srcT - smoothTMod) * 0.1f;  // slower smoothing
      smoothCMod += (srcC - smoothCMod) * 0.1f;

      // --- Apply modulation depth with soft scaling ---
      timb_mod_cv += ((smoothT * smoothTMod) - timb_mod_cv) * 0.05f;
      color_mod_cv += ((smoothC * smoothCMod) - color_mod_cv) * 0.05f;

      // --- Set base values for other modes ---
      timbre_in = 0.5f;
      color_in = 0.5f;
      engine_updated = true;

    }

    else if (midi_mod) {

      // -------- TIMBRE --------
      if (timbre_locked) {
        if (fabsf(rT - timbre_in) < 0.01f) {
          timbre_locked = false;
          smoothT = rT;
        }
      }

      if (!timbre_locked) {
        smoothT += (rT - smoothT) * 0.15f;
        timbre_in = smoothT;
      }

      // -------- COLOR --------
      if (color_locked) {
        if (fabsf(rC - color_in) < 0.01f) {
          color_locked = false;
          smoothC = rC;
        }
      }

      if (!color_locked) {
        smoothC += (rC - smoothC) * 0.15f;
        color_in = smoothC;
      }

      engine_updated = true;
    }

    else if (filter_enabled) {
      // --- Update filter CVs from modulation pots ---
      smoothCut += (srcT - smoothCut) * 0.1f;
      smoothRes += (srcC - smoothRes) * 0.1f;

      filter_cutoff_cc = (uint8_t)(smoothCut * 127.0f);
      filter_resonance_cc = (uint8_t)(smoothRes * 127.0f);

      // --- Keep Timbre and Color pots working as default ---
      smoothT += (rT - smoothT) * 0.08f;
      smoothC += (rC - smoothC) * 0.08f;

      timbre_in = smoothT;
      color_in = smoothC;

      // --- Decay any modulation CV influence smoothly ---
      timb_mod_cv *= 0.9f;
      color_mod_cv *= 0.9f;

      // --- FM is inactive in filter mode ---
      fm_target = 0.0f;
      engine_updated = true;

    }

    else if (cv_mod2) {

      smoothT += (rT - smoothT) * 0.08f;
      smoothC += (rC - smoothC) * 0.08f;
      timbre_in = smoothT;
      color_in = smoothC;

      // --- 2. Rolling Average Filter ---
      static float historyT[16];
      static float historyC[16];
      static int histIdx = 0;

      historyT[histIdx] = srcT;
      historyC[histIdx] = srcC;
      histIdx = (histIdx + 1) % 16;

      float avgT = 0, avgC = 0;
      for (int i = 0; i < 16; i++) {
        avgT += historyT[i];
        avgC += historyC[i];
      }
      avgT /= 16.0f;
      avgC /= 16.0f;

      smoothTMod += (avgT - smoothTMod) * 0.05f;
      smoothCMod += (avgC - smoothCMod) * 0.05f;

      // --- 3. Strict Deadzone ---
      const float CV_DEADZONE = 0.15f;
      bool cvT_active = (smoothTMod > CV_DEADZONE);
      bool cvC_active = (smoothCMod > CV_DEADZONE);

      // --- 4. Large-Band Hysteresis for Engines ---
      static float lockT = -1.0f;
      const float ENG_HYST = 0.10f;

      if (cvT_active) {
        if (fabsf(smoothTMod - lockT) > ENG_HYST) {
          float norm = (smoothTMod - CV_DEADZONE) / (1.0f - CV_DEADZONE);
          int new_idx = (int)(norm * (float)NUM_ENGINES);
          new_idx = constrain(new_idx, 0, NUM_ENGINES - 1);

          if (new_idx != engine_idx) {
            engine_idx = new_idx;
            lockT = smoothTMod;
            engine_updated = true;
          }
        }
      } else {
        lockT = -1.0f;
      }

      // --- 5. Large-Band Hysteresis for FM ---
      static float lockC = 0.0f;
      const float FM_HYST = 0.1f;

      if (cvC_active) {
        if (fabsf(smoothCMod - lockC) > FM_HYST) {
          float target_fm = (smoothCMod - CV_DEADZONE) / (1.0f - CV_DEADZONE);
          fm_mod = constrain(target_fm, 0.0f, 1.0f);
          lockC = smoothCMod;
        }
      } else {
        fm_mod *= 0.5f;
        if (fm_mod < 0.01f) {
          fm_mod = 0.0f;
          lockC = 0.0f;
        }
      }

      timbre_locked = false;
      color_locked = false;
    }


    else {
      smoothT += (rT - smoothT) * 0.08f;
      smoothC += (rC - smoothC) * 0.08f;
      timbre_in = smoothT;
      color_in = smoothC;

      // Zero out all CV-related variables
      timb_mod_cv = 0.0f;
      color_mod_cv = 0.0f;
      fm_mod = 0.0f;

      timbre_locked = false;
      color_locked = false;
      engine_updated = true;
    }
  }

  static int lClk = digitalRead(ENCODER_CLK);
  int clk = digitalRead(ENCODER_CLK);
  static unsigned long last_enc_time = 0;

  if (clk != lClk && millis() - last_enc_time > 8) {
    last_enc_time = millis();
    int step = (digitalRead(ENCODER_DT) != clk) ? 1 : -1;

    switch (display_mode) {
      case ENGINE_SELECT_MODE:
        engine_idx = (engine_idx + step + NUM_ENGINES) % NUM_ENGINES;
        engine_updated = true;
        last_encoder_activity = millis();
        break;

      case SETTINGS_MODE:
        switch (enc_state) {
          case VOLUME_ADJUST: master_volume = constrain(master_volume + step * 0.01f, 0.f, 1.f); break;
          case ATTACK_ADJUST:
            env_attack_s = constrain(env_attack_s + step * 0.01f, 0.001f, 1.f);
            env_params_changed = true;
            break;
          case RELEASE_ADJUST:
            env_release_s = constrain(env_release_s + step * 0.01f, 0.01f, 2.f);
            env_params_changed = true;
            break;

          case ARP_TOGGLE:
            arp.enabled = !arp.enabled;
            break;

          case ARP_MODE:
            {
              int nextMode = (int)arp.mode + 1;
              if (nextMode > 5) nextMode = 0;
              arp.mode = (ArpDirection)nextMode;
              break;
            }

          case ARP_DIV:
            {
              // Toggle through 24 (1/4), 12 (1/8), 6 (1/16), 3 (1/32)
              if (arp.division == 24) arp.division = 12;
              else if (arp.division == 12) arp.division = 6;
              else if (arp.division == 6) arp.division = 3;
              else arp.division = 24;
              break;
            }

          case ARP_LATCH:
            if (step != 0) {
              arp_latch_enabled = !arp_latch_enabled;

              if (!arp_latch_enabled) {
                // 1. Clear the Arp internal buffer
                arp.clear();

                // 2. IMPORTANT: Hard-kill any voices currently tied to the Arp
                for (int i = 0; i < MAX_VOICES; i++) {
                  // We set active to false to trigger the envelope release
                  voices[i].active = false;
                  // If your engine is 'hanging', force velocity to 0 or re-init
                  voices[i].sustained = false;
                }

                // 3. Reset the tracking array
                arp.lastPitchCount = 0;
              }
              engine_updated = true;
            }
            break;


          case FILTER_TOGGLE:
            filter_enabled = !filter_enabled;
            midi_mod = false;
            cv_mod1 = false;
            cv_mod2 = false;
            break;
          case MIDI_MOD:
            midi_mod = !midi_mod;

            if (midi_mod) {
              cv_mod1 = false;
              cv_mod2 = false;
            }
            break;
          case CV_MOD1:
            cv_mod1 = !cv_mod1;
            filter_enabled = false;
            cv_mod2 = false;
            if (cv_mod1) midi_mod = false;
            break;
          case CV_MOD2:
            cv_mod2 = !cv_mod2;
            cv_mod1 = false;
            filter_enabled = false;
            if (cv_mod2) midi_mod = false;
            break;
            if (cv_mod1) midi_mod = false;
            break;
          case MIDI_CH:
            midi_ch = constrain(midi_ch + step, 1, 16);
            break;
          case SCOPE_TOGGLE:
            oscilloscope_enabled = !oscilloscope_enabled;
            if (!oscilloscope_enabled && display_mode == OSCILLOSCOPE_MODE) {
              display_mode = ENGINE_SELECT_MODE;
              scope_ready = false;
            }
            break;

          default:
            display_mode = ENGINE_SELECT_MODE;
            midi_mod = false;
            cv_mod1 = false;
            cv_mod2 = false;
            filter_enabled = false;
            engine_updated = true;

            break;
        }
        last_encoder_activity = millis();
        engine_updated = true;
        break;

      case OSCILLOSCOPE_MODE:
        display_mode = ENGINE_SELECT_MODE;
        engine_updated = true;
        last_encoder_activity = millis();
        break;
    }
  }
  lClk = clk;

  static int lBtn = HIGH;
  static unsigned long last_btn_time = 0;

  int btn = digitalRead(ENCODER_SW);

  if (lBtn == HIGH && btn == LOW && millis() - last_btn_time > BUTTON_DEBOUNCE_MS) {

    last_btn_time = millis();
    last_encoder_activity = millis();

    switch (display_mode) {
      case ENGINE_SELECT_MODE:
        display_mode = SETTINGS_MODE;
        enc_state = ENGINE_SELECT;
        engine_updated = true;
        break;

      case SETTINGS_MODE:
        enc_state = (EncoderState)((enc_state + 1) % 14);
        engine_updated = true;
        break;

      case OSCILLOSCOPE_MODE:
        display_mode = SETTINGS_MODE;
        enc_state = (EncoderState)((enc_state + 1) % 14);
        engine_updated = true;
        break;
    }
  }

  lBtn = btn;

#if USE_SCREEN
  static int last_engine_draw = -1;
  static unsigned long last_draw_time = 0;

  if (millis() - last_draw_time > 60) {
    last_draw_time = millis();
    unsigned long idle = millis() - last_encoder_activity;

    if (display_mode == SETTINGS_MODE && idle > 5000) {
      display_mode = ENGINE_SELECT_MODE;
      enc_state = ENGINE_SELECT;
      engine_updated = true;
      last_engine_draw = -1;
    } else if (display_mode == ENGINE_SELECT_MODE && idle > 10000 && oscilloscope_enabled) {
      display_mode = OSCILLOSCOPE_MODE;
      engine_updated = true;
      last_engine_draw = -1;
    }
    switch (display_mode) {
      case OSCILLOSCOPE_MODE:
        drawScope();
        break;
      case ENGINE_SELECT_MODE:
      case SETTINGS_MODE:
        if (engine_updated || engine_idx != last_engine_draw) {
          drawEngineUI();
          last_engine_draw = engine_idx;
          engine_updated = false;
        }
        break;
    }
  }
#endif

  yield();
}

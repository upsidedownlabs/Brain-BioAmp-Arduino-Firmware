#include <Arduino.h>
#include "FspTimer.h"

#define SAMP_RATE 256           // Hz
#define BAUD_RATE 115200
#define SAMPLE_PIN A0

FspTimer unoTimer;

// control flags
volatile bool sample_flag = false;  // set by ISR
volatile uint32_t last_sample = 0;  // last sample value (if you want it)
bool timerStatus = false;

// tiny callback matching library signature
void timerCallback(timer_callback_args_t __attribute((unused)) * p_args) {
  // keep it minimal
  sample_flag = true;
  (void)p_args;
}

// Try to initialize a timer and report result
bool timerBegin(float sampling_rate) {
  uint8_t timer_type = GPT_TIMER; // as in your FspTimer.h
  int8_t timer_channel = FspTimer::get_available_timer(timer_type);


  if (timer_channel != -1) {
    bool ok = unoTimer.begin(TIMER_MODE_PERIODIC, timer_type, timer_channel, sampling_rate, 0.0f, timerCallback);
    if (!ok) return false;

    // Optional helper calls shown in examples you used previously
    unoTimer.setup_overflow_irq(); // ok if available; harmless otherwise
    unoTimer.open();               // prepare internal structures

    return true;
  } else {
    return false;
  }
}

bool timerStart() {
  timerStatus = true;
  //digitalWrite(LED_BUILTIN, HIGH);
  bool ok = unoTimer.start();
  return ok;
}

bool timerStop() {
  timerStatus = false;
  //digitalWrite(LED_BUILTIN, LOW);
  bool ok = unoTimer.stop();
  return ok;
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) { /* wait */ }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  analogReadResolution(14); // as you used earlier

  // Attempt to initialize timer
  bool begin_ok = timerBegin(SAMP_RATE);

  if (!begin_ok) {
    Serial.println("Timer begin failed. Falling back to software-timer test.");
    // Optionally you can start a software fallback here or just return
    // We'll return and the sketch will still respond to serial commands below
  } else {
    // Start immediately so you can see samples without sending START command
    bool started = timerStart();
    if (!started) {
      Serial.println("Timer start failed after begin. You may need a different type/channel.");
    } else {
      Serial.println("Timer started successfully. You should see samples soon.");
    }
  }
}

void loop() {
  // If flag set by callback, sample_flag will be true.
  if (sample_flag) {
    // Clear flag atomically
    noInterrupts();
    sample_flag = false;
    interrupts();

    // Read ADC in main context
    float value = analogRead(SAMPLE_PIN);
    float filteredvalue= EEGFilter(value);

    // Blink LED very briefly to show activity (not in ISR)
    //digitalWrite(LED_BUILTIN, HIGH);
    // Print timestamp + value in CSV: micros(),value

    Serial.println(filteredvalue);

    digitalWrite(LED_BUILTIN, LOW);
  }

}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500 Hz, frequency: [0.5, 29.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html

float EEGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.41361551*z1 - 0.51060681*z2;
    output = 0.00071374*x + 0.00142749*z1 + 0.00071374*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.64480745*z1 - 0.76352964*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.98813960*z1 - 0.98818120*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.99527805*z1 - 0.99531780*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
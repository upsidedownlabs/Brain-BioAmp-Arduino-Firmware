// This code is designed to work specifically on:
// 1. Arduino UNO R4 Minima
// 2. Arduino UNO R4 WiFi

// BCI FFT - BioAmp EXG Pill
// [https://github.com/upsidedownlabs/BioAmp-EXG-Pill](https://github.com/upsidedownlabs/BioAmp-EXG-Pill)

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2024 - 2025 Krishnanshu Mittal - [krishnanshu@upsidedownlabs.tech](mailto:krishnanshu@upsidedownlabs.tech)
// Copyright (c) 2024 - 2025 Upside Down Labs - [contact@upsidedownlabs.tech](mailto:contact@upsidedownlabs.tech)

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "arm_math.h"
#include <math.h>
#include "Keyboard.h"

#define SAMPLE_RATE 500 // 500 samples per second
#define FFT_SIZE 256    // FFT resolution: 256 samples
#define BAUD_RATE 115200
#define INPUT_PIN A0
#define BETA_THRESHOLD 10.0 // Beta threshold for focus detection

// EEG Frequency Bands
#define DELTA_LOW 0.5
#define DELTA_HIGH 4.0
#define THETA_LOW 4.0
#define THETA_HIGH 8.0
#define ALPHA_LOW 8.0
#define ALPHA_HIGH 13.0
#define BETA_LOW 13.0
#define BETA_HIGH 30.0
#define GAMMA_LOW 30.0
#define GAMMA_HIGH 45.0

// Smoothing factor (0.0 to 1.0) - Lower values = more smoothing
#define SMOOTHING_FACTOR 0.63
const float EPS = 1e-6f; // small guard value against divide-by-zero

// Global variables for key state tracking
bool wKeyPressed = false;

// Structure to hold bandpower results
typedef struct
{
  float delta;
  float theta;
  float alpha;
  float beta;
  float gamma;
  float total;
} BandpowerResults;

// Structure for smoothed bandpower values
typedef struct
{
  float delta;
  float theta;
  float alpha;
  float beta;
  float gamma;
  float total;
} SmoothedBandpower;

SmoothedBandpower smoothedPowers = {0}; // Global smoothed values

// --- Filter Functions ---
// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: [https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html)
float Notch(float input)
{
  float output = input;
  {
    static float z1 = 0, z2 = 0;
    float x = output - (-1.56858163f * z1) - (0.96424138f * z2);
    output = 0.96508099f * x + (-1.56202714f * z1) + (0.96508099f * z2);
    z2 = z1;
    z1 = x;
  }
  {
    static float z1 = 0, z2 = 0;
    float x = output - (-1.61100358f * z1) - (0.96592171f * z2);
    output = 1.00000000f * x + (-1.61854514f * z1) + (1.00000000f * z2);
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Low-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: 45.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: [https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html)
float EEGFilter(float input)
{
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.22465158 * z1 - 0.45044543 * z2;
    output = 0.05644846 * x + 0.11289692 * z1 + 0.05644846 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// --- FFT Setup ---
float inputBuffer[FFT_SIZE];
float fftOutputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];

arm_rfft_fast_instance_f32 S;
volatile uint16_t sampleIndex = 0;
volatile bool bufferReady = false;

// Apply exponential moving average smoothing
void smoothBandpower(BandpowerResults *raw, SmoothedBandpower *smoothed)
{
  smoothed->delta = SMOOTHING_FACTOR * raw->delta + (1 - SMOOTHING_FACTOR) * smoothed->delta;
  smoothed->theta = SMOOTHING_FACTOR * raw->theta + (1 - SMOOTHING_FACTOR) * smoothed->theta;
  smoothed->alpha = SMOOTHING_FACTOR * raw->alpha + (1 - SMOOTHING_FACTOR) * smoothed->alpha;
  smoothed->beta = SMOOTHING_FACTOR * raw->beta + (1 - SMOOTHING_FACTOR) * smoothed->beta;
  smoothed->gamma = SMOOTHING_FACTOR * raw->gamma + (1 - SMOOTHING_FACTOR) * smoothed->gamma;
  smoothed->total = SMOOTHING_FACTOR * raw->total + (1 - SMOOTHING_FACTOR) * smoothed->total;
}

// Calculate bandpower for different frequency bands
BandpowerResults calculateBandpower(float *powerSpectrum, float binResolution, uint16_t halfSize)
{
  BandpowerResults results = {0};

  for (uint16_t i = 1; i < halfSize; i++)
  {
    float freq = i * binResolution;
    float power = powerSpectrum[i];
    results.total += power;

    if (freq >= DELTA_LOW && freq < DELTA_HIGH)
    {
      results.delta += power;
    }
    else if (freq >= THETA_LOW && freq < THETA_HIGH)
    {
      results.theta += power;
    }
    else if (freq >= ALPHA_LOW && freq < ALPHA_HIGH)
    {
      results.alpha += power;
    }
    else if (freq >= BETA_LOW && freq < BETA_HIGH)
    {
      results.beta += power;
    }
    else if (freq >= GAMMA_LOW && freq < GAMMA_HIGH)
    {
      results.gamma += power;
    }
  }

  return results;
}

// Process FFT and calculate bandpower
void processFFT()
{
  // Compute FFT
  arm_rfft_fast_f32(&S, inputBuffer, fftOutputBuffer, 0);

  // Compute magnitudes and power spectrum
  uint16_t halfSize = FFT_SIZE / 2;
  for (uint16_t i = 0; i < halfSize; i++)
  {
    float real = fftOutputBuffer[2 * i];
    float imag = fftOutputBuffer[2 * i + 1];
    powerSpectrum[i] = real * real + imag * imag;
  }

  // Frequency resolution
  float binResolution = (float)SAMPLE_RATE / FFT_SIZE;

  // Calculate raw bandpower
  BandpowerResults rawBandpower = calculateBandpower(powerSpectrum, binResolution, halfSize);

  // Apply smoothing
  smoothBandpower(&rawBandpower, &smoothedPowers);

  // Calculate beta percentage
  float betaPercentage = (smoothedPowers.beta / (smoothedPowers.total + EPS)) * 100;

  // Handle W key press/release based on beta threshold
  if (betaPercentage > BETA_THRESHOLD && !wKeyPressed)
  {
    Keyboard.press('w');
    wKeyPressed = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Focused - W pressed");
  }
  else if (betaPercentage <= BETA_THRESHOLD && wKeyPressed)
  {
    Keyboard.release('w');
    wKeyPressed = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("W released");
  }
}

void setup()
{
  Serial.begin(BAUD_RATE);
  while (!Serial)
    ;

  // Initialize Keyboard HID functionality
  Keyboard.begin();

  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  arm_rfft_fast_init_f32(&S, FFT_SIZE);
}

void loop()
{
  static unsigned long lastMicros = 0;
  unsigned long currentMicros = micros();
  unsigned long interval = currentMicros - lastMicros;
  lastMicros = currentMicros;

  static long timer = 0;
  timer -= interval;
  if (timer < 0)
  {
    timer += 1000000 / SAMPLE_RATE;

    int rawSample = analogRead(INPUT_PIN);
    float filteredSample = EEGFilter(Notch(rawSample));

    if (sampleIndex < FFT_SIZE)
    {
      inputBuffer[sampleIndex++] = filteredSample;
    }
    if (sampleIndex >= FFT_SIZE)
    {
      bufferReady = true;
    }
  }

  if (bufferReady)
  {
    processFFT();
    sampleIndex = 0;
    bufferReady = false;
  }
}

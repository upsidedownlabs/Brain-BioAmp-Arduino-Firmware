
// This code is designed to work specifically on:
// 1.Arduino UNO R4 WiFi

// BCI Spiral - BioAmp EXG Pill
// https://github.com/upsidedownlabs/BioAmp-EXG-Pill

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2024 - 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech

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
#include "spiralAnimation.h"

#define SAMPLE_RATE   512         // 500 samples per second
#define FFT_SIZE      256         // FFT resolution: 256 samples
#define BAUD_RATE     115200
#define INPUT_PIN     A0
#define WARMUP_PERIOD_MS 3000     // Ignore first 5 seconds of data

// EEG Frequency Bands
#define DELTA_LOW     0.5
#define DELTA_HIGH    4.0
#define THETA_LOW     4.0
#define THETA_HIGH    8.0
#define ALPHA_LOW     8.0
#define ALPHA_HIGH    13.0
#define BETA_LOW      13.0
#define BETA_HIGH     30.0
#define GAMMA_LOW     30.0
#define GAMMA_HIGH    45.0

// Smoothing factor (0.0 to 1.0) - Lower values = more smoothing
#define SMOOTHING_FACTOR 0.63
const float EPS = 1e-6f;          // small guard value against divide-by-zero

// Structure to hold bandpower results
typedef struct {
  float delta;
  float theta;
  float alpha;
  float beta;
  float gamma;
  float total;
} BandpowerResults;

// Structure for smoothed bandpower values
typedef struct {
  float delta;
  float theta;
  float alpha;
  float beta;
  float gamma;
  float total;
} SmoothedBandpower;

SmoothedBandpower smoothedPowers = {0}; // Global smoothed values

// --- Filter Functions ---
// Notch filter to remove 48-52 Hz interference
float Notch(float input) {
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

// EEG low-pass filter with 45 Hz cutoff
float EEGFilter(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.22465158*z1 - 0.45044543*z2;
    output = 0.05644846*x + 0.11289692*z1 + 0.05644846*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// --- FFT Setup ---
float inputBuffer[FFT_SIZE];
float fftOutputBuffer[FFT_SIZE];
float fftMagnitudes[FFT_SIZE / 2];
float powerSpectrum[FFT_SIZE / 2];

arm_rfft_fast_instance_f32 S;
volatile uint16_t sampleIndex = 0;
volatile bool bufferReady = false;

// Apply exponential moving average smoothing
void smoothBandpower(BandpowerResults* raw, SmoothedBandpower* smoothed) {
  smoothed->delta = SMOOTHING_FACTOR * raw->delta + (1 - SMOOTHING_FACTOR) * smoothed->delta;
  smoothed->theta = SMOOTHING_FACTOR * raw->theta + (1 - SMOOTHING_FACTOR) * smoothed->theta;
  smoothed->alpha = SMOOTHING_FACTOR * raw->alpha + (1 - SMOOTHING_FACTOR) * smoothed->alpha;
  smoothed->beta  = SMOOTHING_FACTOR * raw->beta  + (1 - SMOOTHING_FACTOR) * smoothed->beta;
  smoothed->gamma = SMOOTHING_FACTOR * raw->gamma + (1 - SMOOTHING_FACTOR) * smoothed->gamma;
  smoothed->total = SMOOTHING_FACTOR * raw->total + (1 - SMOOTHING_FACTOR) * smoothed->total;
}

// Calculate bandpower for different frequency bands
BandpowerResults calculateBandpower(float* powerSpectrum, float binResolution, uint16_t halfSize) {
  BandpowerResults results = {0};
  
  for (uint16_t i = 1; i < halfSize; i++) {
    float freq = i * binResolution;
    float power = powerSpectrum[i];
    results.total += power;
    
    if (freq >= DELTA_LOW && freq < DELTA_HIGH) {
      results.delta += power;
    } else if (freq >= THETA_LOW && freq < THETA_HIGH) {
      results.theta += power;
    } else if (freq >= ALPHA_LOW && freq < ALPHA_HIGH) {
      results.alpha += power;
    } else if (freq >= BETA_LOW && freq < BETA_HIGH) {
      results.beta += power;
    } else if (freq >= GAMMA_LOW && freq < GAMMA_HIGH) {
      results.gamma += power;
    }
  }
  
  return results;
}

enum SpiralDir { NONE=0, spiralFORWARD, spiralBACKWARD };
volatile SpiralDir spiralDir = NONE;

// remember the last time we saw beta > threshold
static unsigned long lastBetaDetected = 0;
static bool warmupComplete = false;
static unsigned long startTime = 0;

// Process FFT and calculate bandpower
void processFFT() {
  // Compute FFT
  arm_rfft_fast_f32(&S, inputBuffer, fftOutputBuffer, 0);

  // Compute magnitudes and power spectrum
  uint16_t halfSize = FFT_SIZE / 2;
  for (uint16_t i = 0; i < halfSize; i++) {
    float real = fftOutputBuffer[2 * i];
    float imag = fftOutputBuffer[2 * i + 1];
    fftMagnitudes[i] = sqrtf(real * real + imag * imag);
    powerSpectrum[i] = real * real + imag * imag;
  }

  // Frequency resolution
  float binResolution = (float)SAMPLE_RATE / FFT_SIZE;

  // Calculate raw bandpower
  BandpowerResults rawBandpower = calculateBandpower(powerSpectrum, binResolution, halfSize);

  // Apply smoothing
  smoothBandpower(&rawBandpower, &smoothedPowers);

  // Only check for beta waves after warmup period
  if (warmupComplete) {
    bool betaAbove = ((smoothedPowers.beta / (smoothedPowers.total + EPS)) * 100) > 20;
    if (betaAbove) {
      lastBetaDetected = millis();
    }
  }

  // Print results
  Serial.print("Delta");
  Serial.print(" (");
  Serial.print((smoothedPowers.delta / (smoothedPowers.total + EPS)) * 100);
  Serial.println("%)");
  
  Serial.print("Theta");
  Serial.print(" (");
  Serial.print((smoothedPowers.theta / (smoothedPowers.total + EPS)) * 100);
  Serial.println("%)");
  
  Serial.print("Alpha");
  Serial.print(" (");
  Serial.print((smoothedPowers.alpha / (smoothedPowers.total + EPS)) * 100);
  Serial.println("%)");
  
  Serial.print("Beta");
  Serial.print(" (");
  Serial.print((smoothedPowers.beta / (smoothedPowers.total + EPS)) * 100);
  Serial.println("%)");
  
  Serial.print("Gamma");
  Serial.print(" (");
  Serial.print((smoothedPowers.gamma / (smoothedPowers.total + EPS)) * 100);
  Serial.println("%)");
  
  Serial.println();
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial);

  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  SpiralAnimation::init();

  arm_rfft_fast_init_f32(&S, FFT_SIZE);
  
  // Initialize timing variables
  startTime = millis();
  lastBetaDetected = millis();
}

void loop() {
  static unsigned long lastMicros = 0;
  unsigned long currentMicros = micros();
  unsigned long interval = currentMicros - lastMicros;
  lastMicros = currentMicros;

  // Check if warmup period is complete
  if (!warmupComplete && (millis() - startTime > WARMUP_PERIOD_MS)) {
    warmupComplete = true;
    Serial.println("Warmup complete - now detecting beta waves");
  }

  static long timer = 0;
  timer -= interval;
  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    
    int rawSample = analogRead(INPUT_PIN);
    float filteredSample = EEGFilter(Notch(rawSample));
    
    if (sampleIndex < FFT_SIZE) {
      inputBuffer[sampleIndex++] = filteredSample;
    }
    if (sampleIndex >= FFT_SIZE) {
      bufferReady = true;
    }
  }

  if (bufferReady) {
    processFFT();
    sampleIndex = 0;
    bufferReady = false;
  }

  // drive the spiral animation forward/backward
  static unsigned long lastStep = 0;
  const unsigned long FORWARD_INTERVAL  = 100;   // Fast forward
  const unsigned long BACKWARD_INTERVAL = 200;   // Slower backward

  // decide direction
  if (!warmupComplete) {
    spiralDir = spiralBACKWARD;  // Force backward during warmup
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    unsigned long since = millis() - lastBetaDetected;
    spiralDir = (since < 1000UL) ? spiralFORWARD : spiralBACKWARD;
  }

  if (millis() - lastStep >= (spiralDir == spiralFORWARD ? FORWARD_INTERVAL : BACKWARD_INTERVAL)) {
    if (spiralDir == spiralFORWARD) {
      SpiralAnimation::stepForward();
    }
    else {
      SpiralAnimation::stepBackward();
    }
    lastStep = millis();
  }
}
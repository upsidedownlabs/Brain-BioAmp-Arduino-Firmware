# BCI Gaming

Brain-controlled gaming using EEG signals to control 'W' key input for forward movement in games.

## How it Works

This sketch processes EEG signals from the BioAmp EXG Pill and automatically presses the 'W' key when beta waves (focus/concentration) exceed the threshold.

## Upload Instructions

1. Open Arduino IDE
2. Load this sketch
3. Select Arduino UNO R4 board (Install R4 boards from boards manager if not installed)
4. Install CMSIS_DSP library from the library manager
5. Upload to your Arduino

## Configuration

If focus detection isn't working properly, modify the beta threshold:

``
#define BETA_THRESHOLD 10.0 // Change this value
``


- **Increase** (15-20) if 'W' key triggers too easily
- **Decrease** (5-8) if focus not detected properly

## Gaming

Test with **Evo F4 Game**: [https://gamaverse.com/evo-f4-game/](https://gamaverse.com/evo-f4-game/)

Or use any game that uses 'W' key to go forward. Focus your mind to move forward!

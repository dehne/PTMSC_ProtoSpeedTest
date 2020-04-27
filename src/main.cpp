/****
 * PTMSC Proto Cals Speed Test V0.1, April 2020
 * 
 * Run the core calculation needed to drive the diver along a straight line 
 * path to see how long it takes because I'm worried that a little Arduino 
 * Uno isn't going to hack it. I think I need to step the steppers at at
 * least 200 Hz and to do the core calculation at at least 10Hz. Plus I need 
 * to have some capacity left over to communicate with the main computer.
 * 
 * Copyright (C) 2020 D.L. Ehnebuske
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 * 
 ****/

#include <Arduino.h>

#define BANNER      F("Calc Speed Test v0.10")
#define PULSE_PIN   (3)           // Pin to pulse for stepping motors to account for time. Really there are 4.
#define DIR_PIN     (4)           // Ditto but for direction pins.

#define X_START     (20.0)        // Starting x value (mm)
#define Y_START     (20.0)        // Starting y (mm)
#define Z_START     (103.0)       // Starting z (mm)
#define X_END       (925.0)       // Ending x value (mm)
#define Y_END       (545.0)       // Ending y (mm)
#define Z_END       (650.0)       // Ending z  (mm)

#define SIZE_X      (950.0)       // Width of display, pulley to pulley (mm)
#define SIZE_Y      (565.0)       // Depth (mm)
#define SIZE_Z      (753.0)       // Height (mm)

#define DS_FREQ     (800)               // Max frequency (Hz) at which we step outputting
#define DS_INTERVAL (1000000 / DS_FREQ) // Min interval in us at which we do the step outputting
#define MM_PER_REV       (31.51)        // Millimeters of wire travel per revolution
#define STEPS_PER_REV    (400.0)        // What it sounds like
#define STEPS_PER_MM     (STEPS_PER_REV / MM_PER_REV)
#define mmToSteps(mm)    ((mm) * STEPS_PER_MM)
#define stepsToMm(steps) ((steps) / STEPS_PER_MM)


unsigned long startMicros;        // micros() at start of run
unsigned long dsMicros[4] = {0};  // micros() at start of last step outputting for given motor
unsigned long dsInterval[4] = {0}; // Interval (us) between steps for given motor
float my, by, mz, bz;             // slope in x-y, y-intercept, slope in x-z, z-intercept
float tScale;                     // Move length in steps
long nBatches = 0;                // Number of DT_INTERVAL batches done
float dt;                         // Fraction of the move a batch is
float t = 0;                      // How far along we are in the move [0..1]
long calcTime = 0;                // Accumulator for time used by dt calculation

long cableSteps[4];               // Current lengths of cables in steps
bool shortening[4] = {0};         // Whether the cable is shortening or lengthening
long pendingSteps[4] = {0};       // Number of remaining steps by which to change each cable

void setup() {
  Serial.begin(9600);
  pinMode(PULSE_PIN, OUTPUT);

  // Set current position
  float xs, ys, zs, xe, ye,  ze, xSize, ySize, zSize;
  xs = mmToSteps(X_START);
  ys = mmToSteps(Y_START);
  zs = mmToSteps(Z_START);
  xe = mmToSteps(X_END);
  ye = mmToSteps(Y_END);
  ze = mmToSteps(Z_END);
  xSize = mmToSteps(SIZE_X);
  ySize = mmToSteps(SIZE_Y);
  zSize = mmToSteps(SIZE_Z);
  cableSteps[0] = sqrt(xs * xs + ys * ys + (zSize - zs) * (zSize - zs));
  cableSteps[1] = sqrt((xSize - xs) * (xSize - xs) + ys * ys + (zSize - zs) * (zSize - zs));
  cableSteps[2] = sqrt(xs * xs + (ySize - ys) * (ySize - ys) + (zSize - zs) * (zSize - zs));
  cableSteps[3] = sqrt((xSize - xs) * (xSize - xs) + (ySize - ys) * (ySize - ys) + (zSize - zs) * (zSize - zs));
  
  // Calculate slopes and intercepts
  my = (ye - ys) / (xe - xs);
  mz = (ze - zs) / (xe - xs);
  by = ys - my * xs;
  bz = zs - mz * xs;
  tScale = sqrt((xe - xs) * (xe - xs) + (ye - ys) * (ye - ys) + (ze - zs) * (ze - zs));
  dt = 1.0 / (stepsToMm(tScale) / 10);    // A batch every cm

  Serial.println(BANNER);
  Serial.print(F("Moving from ("));
  Serial.print(X_START);
  Serial.print(F(", "));
  Serial.print(Y_START);
  Serial.print(F(", "));
  Serial.print(Z_START);
  Serial.print(F(") to ("));
  Serial.print(X_END);
  Serial.print(F(", "));
  Serial.print(Y_END);
  Serial.print(F(", "));
  Serial.print(Z_END);
  Serial.print(F(") all in mm.\ny(t) = "));
  Serial.print(my);
  Serial.print(F(" * x(t) + "));
  Serial.print(by);
  Serial.print(F(", z(t) = "));
  Serial.print(mz);
  Serial.print(F(" * x(t) + "));
  Serial.print(bz);
  Serial.print(F(" all in steps.\nLength of move: "));
  Serial.print(tScale);
  Serial.print(F(" steps.\nEach batch is "));
  Serial.print(dt, 6);
  Serial.print(F(" of the whole.\nInitial cable lengths: "));
  for (byte i = 0; i < 4; i++) {
    Serial.print(cableSteps[i]);
    Serial.print(i == 3 ? F(" steps.\n") : F(", "));
  }

  startMicros = micros();
}

void loop() {
  byte nFinished;

  unsigned long curMicros = micros();

  // Take any pending steps for which the time is ripe
  nFinished = 0;
  for (byte i = 0; i < 4; i++) {
    if (curMicros - dsMicros[i] >= dsInterval[i] && pendingSteps[i] > 0) {
      digitalWrite(PULSE_PIN, HIGH);    // Move stepper[i] one step
      digitalWrite(PULSE_PIN, LOW);
      pendingSteps[i]--;
      cableSteps[i] += shortening[i] ? -1 : 1;
      dsMicros[i] = curMicros;
    }
    if (pendingSteps[i] == 0) {
      nFinished++;
    }
  }

  // If finished the move
  if (t == 1 && nFinished == 4) {
    Serial.print(F("Finished in "));
    Serial.print((curMicros - startMicros) / 1000000.0, 4);
    Serial.print(F(" seconds. Core calculation used "));
    Serial.print(calcTime / 1000000.0, 4);
    Serial.print(F(" seconds of that.\nFinal cable lengths: "));
    for (byte i = 0; i < 4; i++) {
      Serial.print(cableSteps[i]);
      Serial.print(i == 3 ? F(" steps.\n") : F(", "));
    }
    while (true) {
      // Stuck here
    }
  }

  // If time to do dt calculation
  if (nFinished == 4 && t < 1.0) {
    unsigned long startCalc = micros();   // For compute timing data
    float xsts, sxxsts;                   // x(t)**2 and (SIZE_X - x(t))**2
    float ysts, syysts;                   // y(t)**2 and (SIZE_Y - y(t))**2
    float szzsts;                         // (SIZE_Z - z(t))**2
    float newCableSteps[4];               // Cable lengths after this batch of steps
    t = min(1.0, t + dt);                 // t for this batch
    xsts = (1 - t) * mmToSteps(X_START) + t * mmToSteps(X_END); // x(t)
    sxxsts = mmToSteps(SIZE_X) - xsts;    // SIZE_X - x(t)
    ysts = my * xsts + by;                // y(t)
    syysts = mmToSteps(SIZE_Y) - ysts;    // SIZE_Y - y(t)
    szzsts = mmToSteps(SIZE_Z) - mz * xsts - bz;  // SIZE_Z - z(t)
    xsts *= xsts;                         // x(t)**2
    sxxsts *= sxxsts;                     // (SIZE_X - x(t))**2
    ysts *= ysts;                         // y(t)**2
    syysts *= syysts;                     // (SIZE_Y - y(t))**2
    szzsts *= szzsts;                     // (SIZE_Z - z(t))**2
    newCableSteps[0] = sqrt(xsts + ysts + szzsts);
    newCableSteps[1] = sqrt(sxxsts + ysts + szzsts);
    newCableSteps[2] = sqrt(xsts + syysts + szzsts);
    newCableSteps[3] = sqrt(sxxsts + syysts + szzsts);
    float longest = 0.0;
    for (byte i = 0; i < 4; i++) {
      pendingSteps[i] = newCableSteps[i] - cableSteps[i];
      if (pendingSteps[i] < 0) {
        pendingSteps[i] = -pendingSteps[i];
        digitalWrite(DIR_PIN, LOW);             // Set direction anticlockwise
        shortening[i] = true;                   //   to shorten the cable
      } else {
        digitalWrite(DIR_PIN, HIGH);            // Set direction clockwise
        shortening[i] = false;                  //   to lengthen the cable
      }
      if (pendingSteps[i] > longest) {
        longest = pendingSteps[i];
      }
    }
    for (byte i = 0; i < 4; i++) {              // Set interval between steps
      dsInterval[i] = DS_INTERVAL * (longest / max(pendingSteps[i], 1));
    }

    calcTime += micros() - startCalc;
    Serial.print(F("Dispatched batch "));
    Serial.print(nBatches);
    Serial.print(F("; t: "));
    Serial.print(t, 6);
    Serial.print(F("; pendingsteps: "));
    for (byte i = 0; i < 4; i++) {
      Serial.print(shortening[i] ? -pendingSteps[i] : pendingSteps[i]);
      Serial.print(i == 3 ? F(". dsIntervals: ") : F(", "));
    }
    for (byte i = 0; i < 4; i++) {
      Serial.print(dsInterval[i]);
      Serial.print(i == 3 ? F(" us.\n") : F(", "));
    }
    nBatches++;
  }
}
### Interrupt-Driven Display & Real-Time Graphing on BBC micro\:bit

This repo contains a **single C source file `CW3.cpp`** that turns a stock BBC micro\:bit into a dual-display, real-time data logger—*without* busy-wait loops. Everything is written **from first principles** (no high-level CODAL helpers except where explicitly permitted) and respects the coursework’s automated-marking constraints.&#x20;

---

#### 1 · Micro\:bit 5 × 5 LED driver (Subtask 1 · 35 %)

* **Timer-based ISR** refreshes the matrix at **100 Hz** to eliminate flicker.
* Lean **frame-buffer API**

  ```c
  void initMicroBitDisplay(void);
  void setMicroBitPixel(uint8_t x, uint8_t y);
  void clearMicroBitPixel(uint8_t x, uint8_t y);
  void clearMicroBitDisplay(void);
  ```
* Only the bits that matter are written—minimising RAM, CPU time and power.&#x20;

---

#### 2 · SSD1306 128 × 64 OLED driver (Subtask 2 · 35 %)

* Runs the external I²C bus at **400 kHz** for snappy updates.
* Compact **`oledDisplayFrameBuffer`** plus pixel & Bresenham-style line primitives:

  ```c
  void initOledDisplay(void);
  void setOledPixel(uint8_t x, uint8_t y);
  void clearOledPixel(uint8_t x, uint8_t y);
  void drawOledLine(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);
  void clearOledDisplay(void);
  ```
* Pure C; touches the controller only when something actually changed.&#x20;

---

#### 3 · Real-time accelerometer & jerk grapher (Subtask 3 · 30 %)

* `void graphData(uint8_t refreshRate)` runs forever:

  * **Mode A:** plots raw X-axis acceleration (-512 … +511) as vertical bars from the bottom.
  * **Mode B:** plots **jerk** (Δacceleration, -1023 … +1023) around the mid-line.
* **Horizontal scrolling** after 128 samples—old data slides off screen.
* **Button interrupts** (GPIOTE) toggle modes; optional LED columns indicate the active mode.
* Written for speed: the only I²C traffic per tick is a 128-byte page transfer.&#x20;

---

### How to build & flash

```bash
# clone coursework scaffold (CODAL + NRF SDK assumed)
yotta build
microbit-daplink-copy ./build/bbc-microbit-classic-gcc/source/cw3-combined.hex
```

Hardware wiring: connect the 128×64 OLED to `I2C_EXT_SCL`, `I2C_EXT_SDA`, 3 V & GND.
All other peripherals are on-board.

---

### Why this matters

* Shows how to **graduate from busy-waiting to true interrupt-driven design** on Cortex-M0.
* Demonstrates tight, test-friendly APIs that could drop into other micro\:bit projects.
* Emphasises **code clarity, commenting and energy efficiency**, as required by the brief.&#x20;

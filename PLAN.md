# `simpleye` Development Plan
This document outlines the plan and tracks the development of the project. It is chronological in order, but does not set target dates for completion. The plan is subject to, and likely will, change as the project progresses.

## Main Application
### Development Environment
- [ ] VSCode + working IntelliSense
- [ ] Headers and libraries targetting the nRF52840
- [ ] Make (or CMake?) build configuration
- [ ] Working blinky program
- [ ] Configure debugging (openOCD + GDB)
- [ ] Working hello world (print via debug port)
### Camera Driver
- [ ] Configuration (I2C) Interface
- [ ] Data (DVP, 8-bit parallel) Interface
- [ ] Get single frame and pass over debug port
- [ ] Configure stream of data from camera to memory
  - Ideally through DMA
### Main Application
- [ ] TBD

## Neural Network
- [ ] TBD
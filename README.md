# simpleye
A TensorFlow Micro project that aims to implement a "text vision" camera (image in, classifier out).

## The Hardware
The project uses the Arduino [Tiny Machine Learning Kit](https://store-usa.arduino.cc/products/arduino-tiny-machine-learning-kit) which is comprised of:
- Arduino [Nano 33 BLE Sense](https://store.arduino.cc/products/arduino-nano-33-ble-sense)
- [OV7675 Camera Board](https://www.arducam.com/products/camera-breakout-board/0-3mp-ov7675/)
- Tiny Machine Learning Shield (carrier board for the nano 33 and camera)

### Nano 33 BLE Sense
This Arduino board is closer to a true "dev board" than most Arduino boards and hosts a reasonably specc'ed [nRF52840](https://content.arduino.cc/assets/Nano_BLE_MCU-nRF52840_PS_v1.1.pdf) ARM Cortex-M4 @ 64MHz with 1MB Flash and 256KB SRAM. As its name implies, the nano 33 BLE (or really, the nRF52840) includes Nordic's Bluetooth 5 transceiver and (if desired) a BLE soft-device.

The MCU and BLE transceiver are actually populated on the board within a u-blox [NINA-B306 mobule](https://www.u-blox.com/en/product/nina-b30-series-open-cpu-0) with PCB antenna.

In addition to the MCU and radio, the nano 33 BLE includes a good variety of sensors:
- [LSM9DS1](https://content.arduino.cc/assets/Nano_BLE_Sense_lsm9ds1.pdf) Inertial Measurement Unit (accelerometer, gyroscope, magnetometer)
- [MP34DT05](https://content.arduino.cc/assets/Nano_BLE_Sense_mp34dt05-a.pdf) Microphone
- [APDS9960](https://content.arduino.cc/assets/Nano_BLE_Sense_av02-4191en_ds_apds-9960.pdf) Gesture, light & proximity
- [LPS22HB](https://content.arduino.cc/assets/Nano_BLE_Sense_lps22hb.pdf) Barometric pressure
- [HTS221](https://content.arduino.cc/assets/Nano_BLE_Sense_HTS221.pdf) Temperature & humidity

### OV7675 Camera
The camera breakout board included with the ML kit hosts an Omnivision [OV07675](https://www.ovt.com/products/ov07675-a23a/) VGA (640x480) image sensor. The sensor has a parallel output interface (DVP, 8-pin) for image data and a SCCB (I2C) serial interface for control and configuration.

## The Software
The `functional goal` is a machine vision application that can identify a wide range of objects in the video frame and "compress" the image data into object metadata in near-realtime (1fps or better?) and reasolable (80%?) accuracy.

The `project goal` is to refresh and increase my knowledge of embedded C++ development while learning about practical implementations of neural networks in heavily resource-constrained systems.

### Embedded code
The embedded software implementation will be a bare-metal C++ application that samples the camera, runs inference using a pre-compiled Tensorflow Micro neural network, and outputs text data that is descriprive of the scene (exact output format TBD). A stretch goal is to output the data over BLE to a mobile app that then resconstructs the scene using only the received input.

Embedded development will take place in VSCode using Make & arm-gcc for compilation and open-OCD & GDB for debugging. The goal is to avoid proprietary IDE and debugging tools at all costs short of not completing the project. While this is an Arduino kit, the goal is also to avoid Arduino tools (see [functional goal](#the-software) above).

The application code will attempt to follow [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

### Neural network
Building and training the NN model will be done in Python and generally follow the documentation and examples provided by [TensorFlow](https://www.tensorflow.org/lite/microcontrollers/get_started_low_level). This section will be updated (training data, model architecture, compromises, etc...) as more is learned during the development process.

## The Plan
The development plan and progress are documented in [PLAN.md](PLAN.md).
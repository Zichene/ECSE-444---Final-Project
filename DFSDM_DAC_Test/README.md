# DOCUMENTATION

This folder contains **all the files** for the STM32CubeIDE project that implements Wi-Fi, DFSDM & DAC compatibility at the same time. 
See `Getting DFSDM in WiFi.pdf` for rough steps on how DFSDM & DAC was added to the example Wi-Fi project without using MXCube (IOC file).


## SETUP

1. Clone this folder onto your machine.
2. Run the `.project` file in the `WiFi_HTTP_Server\STM32CubeIDE` folder with STM32Cube IDE.
3. See **SETUP** of the `Project_Main` folder to see how to change Wi-Fi & board configurations.

## TODO
- DFSDM & DAC functionality currently is only available on `receiving_main.c`. Need to change  `sending_main.c` to match this.
- Currently more testing needs to be done on Microphone data acquisition and DAC output

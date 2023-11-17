# DOCUMENTATION

This folder contains **all the files** for the STM32CubeIDE project that implements Wi-Fi, DFSDM & DAC compatibility at the same time. 
See `Getting DFSDM in WiFi.pdf` for rough steps on how DFSDM & DAC was added to the example Wi-Fi project without using MXCube (IOC file).


## SETUP

1. Clone this folder onto your machine into the `C:\Users\Administrator\STM32Cube\Repository\STM32Cube_FW_L4_V1.18.0\Projects\B-L4S5I-IOT01A\Applications` folder. You may want to (or not) replace the WiFi folder with this folder (it is the essentially the same folder).
2. Run the `.project` file in the `WiFi_HTTP_Server\STM32CubeIDE` folder with STM32Cube IDE.
3. See **SETUP** of the `Project_Main` folder to see how to change Wi-Fi & board configurations.

## TODO
- DFSDM & DAC functionality currently is only available on `receiving_main.c`. Need to change  `sending_main.c` to match this.
- Currently more testing needs to be done on Microphone data acquisition and DAC output

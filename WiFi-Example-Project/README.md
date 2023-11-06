# Wifi example project
Based on example project for Wi-Fi module given in STMCube32 Repository code.
Using a B-L4s5i-iot01a board (the one with more memory):
- Connects to an existing Wi-Fi network
- Starts a server
- Sends HTML webpage
- Waits for request from client
- POST and GET requests are printed to UART, as well as server info messages

## How to make it work
- Go to the directory where STM32Cube was downloaded, usually looks like C:\Users<name>\STM32Cube\Repository\STM32Cube_FW_L4_V1.18.0\Projects\B-L4S5I-IOT01A\Applications\WiFi
- Replace existing main.c with this main.c
- In `wifi_test` function, replace SSID, Password, Security settings with your network settings.
- If you want to change the HTML displayed, you can use the python script (change the file.html and run main.py to produce a txt file which you can add to the function `SendCustomPage()`).
- Run and hopefully it works


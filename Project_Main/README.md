# DOCUMENTATION
This project folder contains the files to be placed in the `...\Application\User` folder of the STM32Cube repository Wi-Fi example project for the b-l4s5i-iot01a board.

## SETUP
1. Copy-paste all of the files in this folder into the `...\Application\User` folder, and remove the existing main.c file, if it exists.
2. Modify the 'boards.cfg' file, this file contains the configuration data for the boards, its structure looks like:

```python
board_receiving = true
ssid = example_ssid
password = example_pwd
# either WPA_PSK, WPA2_PSK, OPEN
security = WPA2_PSK
host_ip = 10.0.0.215
```

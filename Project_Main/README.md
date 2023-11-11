# DOCUMENTATION
This project folder contains the files to be placed in the `...\Application\User` folder of the STM32Cube repository Wi-Fi example project for the b-l4s5i-iot01a board.

## SETUP
1. Copy-paste all of the files in this folder into the `...\Application\User` folder, and remove the existing main.c file, if it exists. Note: it doesn't matter if you are using the sending or receiving board, you may copy paste both `receiving_main.c` and `sending_main.c` into the folder. The config script will take care of making sure that only one .c file is runnable at a time in STM32CubeIDE.
2. Modify the 'boards.cfg' file, this file contains the configuration data for the boards, its structure looks like:

```python
board_receiving = true     # this field should be true if you're currently working with the receiving board, and false otherwise
ssid = example_ssid        
password = example_pwd
# either WPA_PSK, WPA2_PSK, OPEN
security = WPA2_PSK 
host_ip = 10.0.0.215      # this field is only relevant for the sending board, and you might need to connect with the receiving board first to see what is the IP address
```
3. After you found the IP address of the receiving board (by connecting to Wi-Fi with it, and looking at the UART) and modifying all of the config params in the `boards.cfg` file, you can run either: run the `config.py` script or simply run the 'config.exe' executable. These scripts take care of generating the header files (.h) containing the config data.

4. Running the config.py script in terminal has the advantage that you will be able to see error messages relating to the script, however the .exe works just fine otherwise.
   
5. You can now run the project in STM32CubeIDE!

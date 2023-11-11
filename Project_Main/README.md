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
**NOTE**: You can directly modify the .cfg file in STMCube32IDE, and in fact if you have Git installed you can also use the Git terminal to run the python script inside of the IDE!

3. After you found the IP address of the receiving board (by connecting to Wi-Fi with it, and looking at the UART) and modifying all of the config params in the `boards.cfg` file, you can run either: run the `config.py` script or simply run the `config.exe` executable. These scripts take care of generating the header files (.h) containing the config data.

4. Running the config.py script in terminal has the advantage that you will be able to see error messages relating to the script, however the .exe works just fine otherwise.
   
5. You can now run the project in STM32CubeIDE!

## SENDING BOARD FUNCTIONALITY
- On startup, the sending board connects to a network with credentials `ssid, password, ecn` stored in code. Modify them if your network changes.
- A server is created using `SOCKET = 1, PORT = 10`; these values are also stored in the main file. These settings SHOULD NOT change during the course of the project.
- The sending board now attempts to connect to the receiving board after being provided with the right IP address of the receiving board (you need to manually enter this). The parameters of the function `wifi_connect_to_board()` are values for the IP address of the receiving board.
- Now, we can send data to the receiving board using the function `wifi_send_data_to_board(char* data)`. Every data request sent to the receiver board will be responded to: we can see this response in the UART of the sending board.
### TODO
- Import DFSDM module into workspace.
- Implement all the other things we need to do (audio compression, etc.)
  
## RECEIVING BOARD FUNCTIONALITY
- On startup, the receiving board connects to a network with credentials `ssid, password, ecn` stored in code. Modify them if your network changes.
- A server is created using `SOCKET = 0, PORT = 80`; these values are also stored in the main file. These settings SHOULD NOT change during the course of the project.
- We wait for a client to connect (the SENDING BOARD).
- Once connected, we wait for some data to be sent. When the data arrives, we display the entire http request (for now, can be changed later to just display the data).
- We close the connection and wait for the client to connect again.
### TODO
- Remove alot of useless code that was left over from the example project template.
- Implement all of the desired functionality of the project.

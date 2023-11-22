# DOCUMENTATION

This folder contains **all the files** for the STM32CubeIDE project that implements Wi-Fi, DFSDM & DAC compatibility at the same time. 
See `Getting DFSDM in WiFi.pdf` for rough steps on how DFSDM & DAC was added to the example Wi-Fi project without using MXCube (IOC file).


## SETUP

1. Clone this folder onto your machine into the `C:\Users\Administrator\STM32Cube\Repository\STM32Cube_FW_L4_V1.18.0\Projects\B-L4S5I-IOT01A\Applications` folder. You may want to (or not) replace the WiFi folder with this folder (it is the essentially the same folder).
2. Run the `.project` file in the `WiFi_HTTP_Server\STM32CubeIDE` folder with STM32Cube IDE.
3. The `...\Application\User` folder contains the main files. Note that there are **two** main files: `sending_main.c` and `receiving_main.c`. There is also a `boards.cfg` file and a python script called `config.py`. **The config script will take care of making sure that only one .c file is runnable at a time in STM32CubeIDE.**
   
4. Modify the `boards.cfg` file, this file contains the configuration data for the boards, its structure looks like:

```python
board_receiving = true     # this field should be true if you're currently working with the receiving board, and false otherwise
ssid = example_ssid        
password = example_pwd
# either WPA_PSK, WPA2_PSK, OPEN
security = WPA2_PSK 
host_ip = 10.0.0.215      # this field is only relevant for the sending board, and you might need to connect with the receiving board first to see what is the IP address
```
**NOTE**: You can directly modify the .cfg file in STMCube32IDE, and in fact if you have Git installed you can also use the Git terminal to run the python script inside of the IDE!

5. After you found the IP address of the receiving board (by connecting to Wi-Fi with it, and looking at the UART) and modifying all of the config params in the `boards.cfg` file, you can either: run the `config.py` script or simply run the `config.exe` executable. These scripts take care of generating the header files (.h) containing the config data.

6. Running the config.py script in terminal has the advantage that you will be able to see error messages relating to the script, however the .exe works just fine otherwise.
   
7. You can now run the project in STM32CubeIDE!

## TODO
- Make the sound received actually sound decent
- Audio compression
- Optimization

## SENDING BOARD FUNCTIONALITY
- On startup, the sending board connects to a network with credentials `ssid, password, ecn` stored in code. Modify them if your network changes.
- A server is created using `SOCKET = 1, PORT = 10`; these values are also stored in the main file. These settings **SHOULD NOT** change during the course of the project.
- The sending board now attempts to connect to the receiving board after being provided with the right IP address of the receiving board (you need to manually enter this). The parameters of the function `wifi_connect_to_board()` are values for the IP address of the receiving board.
- Now, we can send data to the receiving board using the function `wifi_send_data_to_board(char* data)`. Every data request sent to the receiver board will be responded to: we can see this response in the UART of the sending board.
- The board will wait for the blue button (user button) to be pressed, after which it will start to collect microphone data and put it in the buffer. Then, the values will be converted to 8-bit (char) values which are sent over Wi-Fi to the other board (using `wifi_connect_to_board()`).
- This process loops until the button is pressed again.

  
## RECEIVING BOARD FUNCTIONALITY
- On startup, the receiving board connects to a network with credentials `ssid, password, ecn` stored in code. Modify them if your network changes.
- A server is created using `SOCKET = 0, PORT = 80`; these values are also stored in the main file. These settings **SHOULD NOT** change during the course of the project.
- We wait for a client to connect (the SENDING BOARD).
- Once connected, we wait for audio data to be sent. When the audio data arrives, it is converted to 32-bit values so that it can be played via the DAC speaker.
- We close the connection and wait for the client to connect again.


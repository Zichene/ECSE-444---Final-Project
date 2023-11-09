# DOCUMENTATION

This is the main file to be placed in the STMCube32IDE project on the SENDING BOARD (B-L4S5I-IOT01A).

## FUNCTIONALITY
- On startup, the sending board connects to a network with credentials `ssid, password, ecn` stored in code. Modify them if your network changes.
- A server is created using `SOCKET = 1, PORT = 10`; these values are also stored in the main file. These settings SHOULD NOT change during the course of the project.
- The sending board now attempts to connect to the receiving board after being provided with the right IP address of the receiving board (you need to manually enter this). The parameters of the function `wifi_connect_to_board()` are
- values for the IP address of the receiving board.
- Now, we can send data to the receiving board using the function `wifi_send_data_to_board(char* data)`. Every data request sent to the receiver board will be responded to: we can see this response in the UART of the sending board.
## TODO
- Implement all of the desired functionality of the project.

# DOCUMENTATION

This is the main file to be placed in the STMCube32IDE example Wi-Fi project on the RECEIVING BOARD (B-L4S5I-IOT01A).

## FUNCTIONALITY
- On startup, the receiving board connects to a network with credentials `ssid, password, ecn` stored in code. Modify them if your network changes.
- A server is created using `SOCKET = 0, PORT = 80`; these values are also stored in the main file. These settings SHOULD NOT change during the course of the project.
- We wait for a client to connect (the SENDING BOARD).
- Once connected, we wait for some data to be sent. When the data arrives, we display the entire http request (for now, can be changed later to just display the data).
- We close the connection and wait for the client to connect again.

## TODO
- Remove alot of useless code that was left over from the example project template.
- Implement all of the desired functionality of the project.

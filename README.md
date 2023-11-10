# ECSE-444---Final-Project
Repo for the Final Project for ECSE 444
## IMPORTANT
- For now this repository only contains the main.c files, which are to be copied into (and replace the existing main files) the WiFi example folder in the STM32CubeIDE repository folder (see DOCUMENTATION below).
- Currently gitignore is set to ignore all files except .c, .h, .s, .py, .ioc, .html, .txt, .tex, .pdf, .png files. If you wish to change this, feel free to modify the .gitignore to include other files. This is so that we only focus on the important files of the project.

### TODO
- Test latency for sending/receiving data between boards
- Import DFSDM, DAC modules into the project workspaces (make these modules work on our boards, without using an IOC file & CubeMX)
- Implement rest of features

## DOCUMENTATION

Two STM32 B-L4S5I-IOT01A boards will send compressed audio data over Wi-Fi. One board will act as the sending board, this board will interface with the DFSDM microphones on board to capture audio data. The other board will be receiving this data via an HTTP request, decompressing it and will use the on-board DAC module to generate a sound via a connected speaker. 


Boards used can be found at: https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html


Note: The wifi component of this project was based from an example project downloaded from the STM32CubeIDE repository folder, specifically the file path should look like this (after downloading STM32CubeIDE): `C:\Users<name>\STM32Cube\Repository\STM32Cube_FW_L4_V1.18.0\Projects\B-L4S5I-IOT01A\Applications\WiFi`

## Links
- Proposal Google Docs: https://docs.google.com/document/d/1_Vil0bpvTTfFVEeQeM1tbq3TkEXOZylY-xVfDCFg2Fo/edit?usp=sharing
- Proposal Official (Overleaf): https://www.overleaf.com/7336791323fcbdjyqzjfsm#9a69bd


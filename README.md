# ECSE-444---Final-Project 
Repo for the Final Project for ECSE 444 @ McGill University F2023. Video demo can be found [here](https://www.youtube.com/watch?v=PtOjv1w0G0o&ab_channel=ChristopherGhosn).

## DOCUMENTATION

Two STM32 B-L4S5I-IOT01A boards will send compressed audio data over Wi-Fi. One board will act as the sending board, this board will interface with the DFSDM microphones on board to capture audio data. The other board will be receiving this data via an HTTP request, decompressing it and will use the on-board DAC module to generate a sound via a connected speaker. 


Boards used can be found at: https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html


Note: The wifi component of this project was based from an example project downloaded from the STM32CubeIDE repository folder, specifically the file path should look like this (after downloading STM32CubeIDE): `C:\Users<name>\STM32Cube\Repository\STM32Cube_FW_L4_V1.18.0\Projects\B-L4S5I-IOT01A\Applications\WiFi`




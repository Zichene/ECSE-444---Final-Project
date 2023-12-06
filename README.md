# ECSE-444---Final-Project 
Repo for the Final Project for ECSE 444 @ McGill University F2023. Video demo can be found [here](https://www.youtube.com/watch?v=PtOjv1w0G0o&ab_channel=ChristopherGhosn).

## DOCUMENTATION

Two STM32 B-L4S5I-IOT01A boards will send compressed audio data over Wi-Fi. One board will act as the sending board, this board will interface with the DFSDM microphones on board to capture audio data. The audio data will be compressed using the [CMSIS-DSP library](https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html). There are two implementations: one where the data is compressed and decompressed using FFT functions, and one where the data is filtered. The code for each can be found in the `FFT_Compression` branch and `Testing-rig-for-filter` branch, respectively.


Boards used can be found at: https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html


Note: The wifi component of this project was based from an example project downloaded from the STM32CubeIDE repository folder, specifically the file path should look like this (after downloading STM32CubeIDE): `C:\Users<name>\STM32Cube\Repository\STM32Cube_FW_L4_V1.18.0\Projects\B-L4S5I-IOT01A\Applications\WiFi`

## COMPRESSION / DECOMPRESSION
See [here](https://github.com/Zichene/ECSE-444---Final-Project/tree/FFT_Compression).

## FILTERING
See [here](https://github.com/Zichene/ECSE-444---Final-Project/tree/Testing-rig-for-filter).



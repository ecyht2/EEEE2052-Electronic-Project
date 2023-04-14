# EEEE2052-Electronic-Project
STM32 code for Electronic Project of EEEE2052 module

## Requirements
1. An [STM32 L476 rg](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html)
2. Arduino LCD Keypad Shield
3. [STM32 cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html)
4. Doppler Radar Unit HB100

### STM32 Pins

### Custom Circuits

## Usage

### Cloning Repository

### Building and Running

### CPLD

A Digilent xxx CPLD board is used to display the speed on 7-segment displays. A logisim circuit is provided in the [CPLD](CPLD/) folder. This contains the gates used to upload to the board. In order to upload to the CPLD board, Xilinx Web Pack is used.

1. Open [CPLD.circ] in logisim.
2. Export to VHDL code `FPGA` → `Synthesize & Download`.
3. Hit annotate and Execute. (The target board does not matter)
4. Don't map any components just hit done. (logisim-evolution would give a warning just hit ok)
5. The exported VHDL code should be in the user's home directory `logisim_evolution_workspace/CPLD/main/vhdl`.
6. Import all the VHDL file into your Xilinx except for the ones in `toplevel` directory.
7. Write an UCF file.
8. Program your CPLD using Xilinx.

For a more detailed tutorial you can see this youtube [video](https://youtu.be/hVAeMMDGdbc?t=1170).

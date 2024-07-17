# ECD322 Open Souce FPGA

This open source project uses the iCE5LP4K iCE40 chip. All files for this project are open source, including the gerber files for the PCB, and are freely modifiable.

Project features include:

- A test socket PCB for the FPGA, which includes the bypass capacitors for the FPGA. The separate PCB to mount the FPGA allows for a more minimalistic design, and increases ease of soldering.
- A two layer and four layer PCB design
- Test software and constraint files to test the FPGA
- Features all components that are easily solderable, using a reflow oven or solder iron

The PCB design includes: 
- A switch and an LED to provide input and output to the FPGA
- A PMOD compatible GPIO header with 8 I/Os to the FPGA
- An external connection to connect your own external clock, and an on-board oscillator

The PCB will require a 5V DC input to power the board. The board requires purchase of Lattice's HW-USBN-2B programming cable, which can be found [here](https://www.latticestore.com/searchresults/tabid/463/searchid/1/searchvalue/hw-usbn-2b/default.aspx). To program the FPGA, you will need to generate a bitmap using the [Lattice iCECube2](https://www.latticesemi.com/iCEcube2) software, and the programming cable is compatible with [Lattice Diamond Programmer](https://www.latticesemi.com/programmer). A free license is needed to use the iCECube2 software.

The PCB was created using EasyEDA, which is a PCB software that is easy to use and modify. The gerber files are not compatible with Atrium. 

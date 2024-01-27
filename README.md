# sdram_controller
SystemVerilog SDRAM Controller specifically for IS42S16400 (on the DE0 Dev Board) 

Made for efficiency, done by keeping rows open during reads, and only precharging when neccesary. The controller intelligently keeps track of what rows are currently open in each bank.
Adjustable burst length of 1, 2, 4, or 8 words.
Adjustable CAS latency of 2 or 3 cycles.
Common SDRAM timing requirements, e.g. tRCD, are parameters, and easily modifiable for different SDRAMs.

I've found that, with 50 random read and write requests (to differing banks, rows, and columns), that the controller can reach around **~50 MT/s** or **~90 MB/s** (at 100 MHz). This is also with a burst length of 8 words. Obviously, if all of these operations were done on the same row in the same bank, this could _much_ higher. E.g., with 20 writes to consequtive columns in the same row in the same bank, and 20 reads to the same, the controller achieves **~100 MT/s** or **~200 MB/s** speed. 

Currently, the DRAM FIFOs are not used, but they will be implemented soon, for handling request queueing. 

## More Technical ##
All banks are closed during refresh, as an all-bank precharge is executed after the refresh.

## What each file is ##
- `/test` contains functional verificationt testbenches (and maybe formal too if I figure that out!)
- `/rtl` obviously contains the rtl
    - `sdram_ctrl.sv` is the SDRAM controller, the heart of this repo
    - `sdram_refresh.sv` is the SDRAM refresh timer
    - `dram_fifo_*.sv` are the files for the SDRAM controller's uni-directional async FIFO, which is yet to be implemented
    - `fifo_*.sv` are the files for the UART's transceivers FIFO
    - `fpga_top_level.sv` is the testbench that's ran on the FPGA, allowing for read and write commands to be issued to test the SDRAM
    - `uart.sv`, `uart_tx.sv`, `uart_rx.sv` is the UART transciever needed for the `fpga_top_level.sv` (also made by me!)

So the only files needed for an implementation of this controller into your project is just `sdram_ctrl.sv`, and `sdram_refresh.sv`. The controller expects the clock frequency it's given to be the clock frequency given to the SDRAM, but phase shifted by 180 degrees. This is to meet setup and hold timing requirements.

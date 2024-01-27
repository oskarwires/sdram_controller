# sdram_controller
SystemVerilog SDRAM Controller specifically for IS42S16400 (on the DE0 Dev Board) 

Made for efficiency, done by keeping rows open during reads, and only precharging when neccesary. The controller intelligently keeps track of what rows are currently open in each bank.
Adjustable burst length of 1, 2, 4, or 8 words.
Adjustable CAS latency of 2 or 3 cycles.
Common SDRAM timing requirements, e.g. tRCD, are parameters, and easily modifiable for different SDRAMs.

I've found that, with 50 random read and write requests (to differing banks, rows, and columns), that the controller can reach around 50 MT/s (at 100 MHz). This is also with a burst length of 8 words. Obviously, if all of these operations were done on the same row in the same bank, this could _much_ higher.

Currently, the DRAM FIFOs are not used, but they will be implemented soon, for handling request queueing. 

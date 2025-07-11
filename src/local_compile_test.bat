@echo off
REM Basic command structure:
REM
REM Compile the Verilog for simulation.
REM    iverilog -s (module to run) -o (output object file) (input file 1) (input file 2) (...)
REM Run the simulation, which may auto-generate a VCD file.
REM    vvp (object file)
REM Run GTKWave on the results
REM    gtkwave (VCD file)

REM Get starting time
echo Starting time: %time%

REM Run compilation tests with iVerilog
echo Running compilation tests...
C:\iverilog\bin\iverilog.exe -Wall -s tt_um_nitelich_conway -o project.o project.v
echo:

REM Clean up all unnecessary files (just object and VCD files)
del .\*.o 2>nul
del .\*.vcd 2>nul

REM All tests done!
echo End time: %time%
echo All tests complete
pause
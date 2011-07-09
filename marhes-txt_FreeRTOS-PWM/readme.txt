Here are instructions on how to program the LPC2378 TXT board.

1.  Installation:
    a.  Install the free Codesourcery toolchain for ARM
    b.  Install openocd -> sudo apt-get install openocd
    c.  Install ddd -> sudo apt-get install ddd
2.  Compile the sources
    a.  cd marhes-txt
    b.  make
3.  Start openocd
    a.  Make sure the ARM-USB-OCD JTAG debugger is plugged into usb and JTAG is 
        plugged into the board.  Also power the board.
    b.  Start openocd -> ./StartOpenOCD.sh
4.  Program and run the board
    a.  ./ddd.sh -> Programs and then runs the program.
    b.  ./dddRun.sh -> Only runs the program.

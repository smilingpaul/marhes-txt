target remote localhost:3333
monitor sleep 500
monitor arm7_9 dcc_downloads enable
monitor arm7_9 fast_memory_access enable
monitor reset halt
monitor gdb_breakpoint_override hard
monitor poll
monitor flash probe 0
set mem inaccessible-by-default off
set remotetimeout 5000
monitor flash erase_sector 0 0 last
monitor flash write_image main.bin 0x0
monitor reset init
monitor sleep 500
monitor soft_reset_halt
symbol-file main.out
break main.c:108
continue

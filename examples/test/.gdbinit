set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4

#
# Use openocd as the remote gdbserver
#
target remote | openocd -f ./openocd.cfg -c "gdb_port pipe; log_output openocd.log"

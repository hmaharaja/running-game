# set the working dir, where all compiled verilog goes
vlib work

# compile all verilog modules in mux.v to working dir
# could also have multiple verilog files
vlog score.v
#load simulation using mux as the top level simulation module
#vsim -L altera_mf_ver running_game -t ns
vsim scoring
#log all signals and add some signals to waveform window
log {/*}
# add wave {/*} would add all items in top level simulation module
add wave {/*}

force {CLOCK_50} 0 0ms, 1 {10ms} -r 20 ms


#reset 
force {KEY[0]} 1 
run 10 ms

force {KEY[0]} 0 
run 100 ms

#data%3 = 0
force {KEY[0]} 1
force {SW[0]} 1

run 50000 ms

#data%3 = 1
force {KEY[0]} 1
force {SW[0]} 1
force SW{SW[3]} 1 
run 10000 ms

#data%3 = 2
force {KEY[0]} 1
force {SW[0]} 1  
run 10000 ms 
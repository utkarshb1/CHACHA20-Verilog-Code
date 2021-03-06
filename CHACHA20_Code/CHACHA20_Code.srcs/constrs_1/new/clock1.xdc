create_clock -period 15.000 -name clk -waveform {0.000 1.000} [get_ports clk]
#set_output_delay -clock [get_clocks *] -add_delay -10.0 [get_ports -filter { NAME =~  "*" && DIRECTION == "OUT" }]
#set_input_delay -clock [get_clocks *] -add_delay 2.9 [get_ports {{addr[0]} {addr[1]} {addr[2]} {addr[3]} {addr[4]} {addr[5]} {addr[6]} {addr[7]} {write_data[0]} {write_data[1]} {write_data[2]} {write_data[3]} {write_data[4]} {write_data[5]} {write_data[6]} {write_data[7]} {write_data[8]} {write_data[9]} {write_data[10]} {write_data[11]} {write_data[12]} {write_data[13]} {write_data[14]} {write_data[15]} {write_data[16]} {write_data[17]} {write_data[18]} {write_data[19]} {write_data[20]} {write_data[21]} {write_data[22]} {write_data[23]} {write_data[24]} {write_data[25]} {write_data[26]} {write_data[27]} {write_data[28]} {write_data[29]} {write_data[30]} {write_data[31]}}]
#set_output_delay -clock [get_clocks *] -add_delay -2.8 [get_ports -filter { NAME =~  "*" && DIRECTION == "OUT" }]

set_property PACKAGE_PIN N18 [get_ports {read_data[27]}]
set_property PACKAGE_PIN R16 [get_ports {addr[1]}]
set_property PACKAGE_PIN T19 [get_ports {addr[5]}]
set_property PACKAGE_PIN T18 [get_ports {addr[3]}]
set_property PACKAGE_PIN R15 [get_ports {addr[0]}]
set_property PACKAGE_PIN M17 [get_ports {read_data[29]}]
set_property PACKAGE_PIN L22 [get_ports {read_data[17]}]
set_property PACKAGE_PIN K20 [get_ports {read_data[16]}]
set_property PACKAGE_PIN N19 [get_ports {read_data[11]}]
set_property PACKAGE_PIN AB17 [get_ports {write_data[15]}]
set_property PACKAGE_PIN AB14 [get_ports {write_data[2]}]
set_property PACKAGE_PIN U14 [get_ports {write_data[0]}]
set_property PACKAGE_PIN Y13 [get_ports {write_data[4]}]
set_property PACKAGE_PIN U15 [get_ports {write_data[20]}]
set_property PACKAGE_PIN V14 [get_ports {write_data[12]}]
set_property PACKAGE_PIN P21 [get_ports {read_data[2]}]
set_property PACKAGE_PIN Y18 [get_ports {write_data[26]}]
set_property PACKAGE_PIN Y14 [get_ports {write_data[6]}]
set_property PACKAGE_PIN U16 [get_ports {write_data[19]}]
set_property PACKAGE_PIN Y19 [get_ports {write_data[28]}]
set_property PACKAGE_PIN N22 [get_ports {read_data[7]}]
set_property PACKAGE_PIN W15 [get_ports {write_data[8]}]
set_property PACKAGE_PIN AA17 [get_ports {write_data[16]}]
set_property PACKAGE_PIN Y15 [get_ports {write_data[7]}]
set_property PACKAGE_PIN P15 [get_ports {read_data[0]}]
set_property PACKAGE_PIN Y21 [get_ports {write_data[31]}]
set_property PACKAGE_PIN R21 [get_ports {read_data[4]}]
set_property PACKAGE_PIN N20 [get_ports {read_data[10]}]
set_property PACKAGE_PIN AA13 [get_ports {write_data[3]}]
set_property PACKAGE_PIN M22 [get_ports {read_data[8]}]
set_property PACKAGE_PIN M20 [get_ports {read_data[12]}]
set_property PACKAGE_PIN M15 [get_ports {read_data[26]}]
set_property PACKAGE_PIN J21 [get_ports {read_data[22]}]
set_property PACKAGE_PIN J22 [get_ports {read_data[21]}]
set_property PACKAGE_PIN P17 [get_ports we]
set_property PACKAGE_PIN T16 [get_ports cs]
set_property PACKAGE_PIN R19 [get_ports {addr[6]}]
set_property PACKAGE_PIN J18 [get_ports {read_data[24]}]
set_property PACKAGE_PIN T17 [get_ports {addr[7]}]
set_property PACKAGE_PIN R18 [get_ports {addr[4]}]
set_property PACKAGE_PIN P16 [get_ports {addr[2]}]
set_property PACKAGE_PIN L16 [get_ports {read_data[31]}]
set_property PACKAGE_PIN AA19 [get_ports {write_data[27]}]
set_property PACKAGE_PIN AA18 [get_ports {write_data[25]}]
set_property PACKAGE_PIN M19 [get_ports {read_data[13]}]
set_property PACKAGE_PIN P22 [get_ports {read_data[6]}]
set_property PACKAGE_PIN AA16 [get_ports {write_data[14]}]
set_property PACKAGE_PIN R20 [get_ports {read_data[5]}]
set_property PACKAGE_PIN W17 [get_ports {write_data[24]}]
set_property PACKAGE_PIN W13 [get_ports {write_data[9]}]
set_property PACKAGE_PIN AB20 [get_ports {write_data[29]}]
set_property PACKAGE_PIN V17 [get_ports {write_data[17]}]
set_property PACKAGE_PIN AA14 [get_ports {write_data[5]}]
set_property PACKAGE_PIN W16 [get_ports {write_data[22]}]
set_property PACKAGE_PIN V15 [get_ports {write_data[11]}]
set_property PACKAGE_PIN AB15 [get_ports {write_data[1]}]
set_property PACKAGE_PIN AB19 [get_ports {write_data[30]}]
set_property PACKAGE_PIN L18 [get_ports {read_data[15]}]
set_property PACKAGE_PIN L19 [get_ports {read_data[14]}]
set_property PACKAGE_PIN P20 [get_ports {read_data[3]}]
set_property PACKAGE_PIN V13 [get_ports {write_data[10]}]
set_property PACKAGE_PIN AB16 [get_ports {write_data[13]}]
set_property PACKAGE_PIN M21 [get_ports {read_data[9]}]
set_property PACKAGE_PIN W18 [get_ports {write_data[23]}]
set_property PACKAGE_PIN Y16 [get_ports {write_data[21]}]
set_property PACKAGE_PIN N15 [get_ports {read_data[1]}]
set_property PACKAGE_PIN U17 [get_ports {write_data[18]}]
set_property PACKAGE_PIN L17 [get_ports {read_data[30]}]
set_property PACKAGE_PIN N17 [get_ports {read_data[28]}]
set_property PACKAGE_PIN J20 [get_ports {read_data[20]}]
set_property PACKAGE_PIN L21 [get_ports {read_data[18]}]
set_property PACKAGE_PIN K21 [get_ports {read_data[19]}]
set_property PACKAGE_PIN M16 [get_ports {read_data[25]}]
set_property PACKAGE_PIN K18 [get_ports {read_data[23]}]
set_property PACKAGE_PIN P18 [get_ports reset_n]
set_property PACKAGE_PIN K19 [get_ports clk]

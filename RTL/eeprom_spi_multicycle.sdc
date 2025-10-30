
create_clock -name x2osc_clk  -period 15.038 [get_nets x2osc_clk]
create_clock -name eif_clk_x2 -period 10.025 [get_nets eif_clk_x2]

create_generated_clock -name EEP_SCLK -source [get_clocks eif_clk_x2] -divide_by 2 [get_ports eep_sclk]

set_clock_groups -asynchronous -group {x2osc_clk} -group {eif_clk_x2}

set_multicycle_path 2 -setup -from [get_cells -hier *u_eep/spi_master_i/*] -to [get_ports {eep_dq0 eep_dq1 eep_dq2 eep_dq3}]
set_multicycle_path 1 -hold  -from [get_cells -hier *u_eep/spi_master_i/*] -to [get_ports {eep_dq0 eep_dq1 eep_dq2 eep_dq3}]

set regs_samples [get_registers *u_eep/spi_master_i/samples_left*]
set_multicycle_path 2 -setup -from $regs_samples -to $regs_samples
set_multicycle_path 1 -hold  -from $regs_samples -to $regs_samples

set_max_delay 20.05 -from $regs_samples -to $regs_samples

# set_output_delay -clock [get_clocks EEP_SCLK] -max 3.0 [get_ports {eep_dq0 eep_dq1 eep_dq2 eep_dq3}]
# set_output_delay -clock [get_clocks EEP_SCLK] -min -1.0 [get_ports {eep_dq0 eep_dq1 eep_dq2 eep_dq3}]

# Define the source and destination registers involved in the timing violation
# set data_cntr_reg [get_registers *u_eep/data_cntr*]
# set spi_tx_bits_reg [get_registers *u_eep/spi_tx_bits*]

# # Apply Multicycle Path for setup: allow 2 cycles for data to arrive
# set_multicycle_path 2 -setup -from $data_cntr_reg -to $spi_tx_bits_reg

# # Apply Multicycle Path for hold: set to N-1 (2-1=1) cycle
# set_multicycle_path 1 -hold  -from $data_cntr_reg -to $spi_tx_bits_reg

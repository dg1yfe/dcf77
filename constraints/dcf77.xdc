## ============================================================================
## dcf77.xdc -- timing constraints for the DCF77 receiver core
##
## The CIC cascade runs on a single 125 MHz clock with clock-enable-style
## valid strobes (every R input samples for the comb/decimation section of
## each stage).  Without help, Vivado would size every adder to the full
## 125 MHz period.  The multicycle constraints below relax the inner stages
## to their actual logical rate.
##
## NOTE: replace `clk` in the get_clocks lookup with the name of the system
## clock at the integration point.  The wildcards under the CIC entities
## should be reviewed once the design is integrated (hierarchical paths
## depend on the parent instance name of dcf77_core).
## ============================================================================

## ---------------------------------------------------------------------------
## CIC1 (R=16): inner integrators and combs only advance every 2 input
## samples (the cic0_v strobe rate).  Relax setup by 2.
## ---------------------------------------------------------------------------
set_multicycle_path -setup 2 \
    -from [get_cells -hier -filter {NAME =~ *u_cic1*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic1*integ_reg*}]
set_multicycle_path -hold 1 \
    -from [get_cells -hier -filter {NAME =~ *u_cic1*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic1*integ_reg*}]

set_multicycle_path -setup 2 \
    -from [get_cells -hier -filter {NAME =~ *u_cic1*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic1*comb_prev*}]
set_multicycle_path -hold 1 \
    -from [get_cells -hier -filter {NAME =~ *u_cic1*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic1*comb_prev*}]

## ---------------------------------------------------------------------------
## CIC2 (R=16): only advances every 32 input samples (cic1_v strobe rate),
## i.e. every 32 system clocks.  Relax setup by 32.
## ---------------------------------------------------------------------------
set_multicycle_path -setup 32 \
    -from [get_cells -hier -filter {NAME =~ *u_cic2*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic2*integ_reg*}]
set_multicycle_path -hold 31 \
    -from [get_cells -hier -filter {NAME =~ *u_cic2*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic2*integ_reg*}]

set_multicycle_path -setup 32 \
    -from [get_cells -hier -filter {NAME =~ *u_cic2*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic2*comb_prev*}]
set_multicycle_path -hold 31 \
    -from [get_cells -hier -filter {NAME =~ *u_cic2*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic2*comb_prev*}]

## ---------------------------------------------------------------------------
## Adaptive threshold / RSSI slow IIRs in dcf77_bit_det only update at the
## 244 kS/s env_valid tick.  False-path them; they have no functional timing
## requirement beyond eventually-consistent.
## ---------------------------------------------------------------------------
set_false_path -from [get_cells -hier -filter {NAME =~ *u_bit_det*env_max_reg*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *u_bit_det*threshold_reg*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *u_bit_det*hyst_reg*}]

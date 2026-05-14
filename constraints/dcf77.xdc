## ============================================================================
## dcf77.xdc -- timing constraints for the DCF77 receiver core
##
## The CIC cascade runs on a single 125 MHz clock with clock-enable-style
## valid strobes (every R input samples for the comb/decimation section of
## each stage).  Without help, Vivado would size every adder to the full
## 125 MHz period.  The multicycle constraints below relax the inner stages
## to their actual logical rate.
##
## Cascade structure (post the 250 kHz pre-mixer / 1 kHz post-mixer refactor):
##
##   adc -> CIC0(R=2)  -- single-cycle at 125 MHz
##       -> CIC1(R=10) -- multicycle 2   (effective 62.5 MS/s)
##       -> CIC2(R=25) -- multicycle 20  (effective 6.25 MS/s)
##       -> mixer @ 250 kS/s
##       -> CIC3_I, CIC3_Q (R=10)  -- multicycle 500   (effective 25 kS/s)
##       -> CIC4_I, CIC4_Q (R=25)  -- multicycle 5000  (effective 1 kS/s)
##
## NOTE: the wildcards under the CIC entities should be reviewed once the
## design is integrated; hierarchical paths depend on the parent instance
## name of dcf77_core.
## ============================================================================

## ---------------------------------------------------------------------------
## CIC1 (R=10): inner integrators / combs advance every 2 input samples
## (the cic0_v strobe rate).  Relax setup by 2.
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
## CIC2 (R=25): inner registers advance every 20 input samples
## (= 2 * 10, the cic1_v strobe rate).
## ---------------------------------------------------------------------------
set_multicycle_path -setup 20 \
    -from [get_cells -hier -filter {NAME =~ *u_cic2*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic2*integ_reg*}]
set_multicycle_path -hold 19 \
    -from [get_cells -hier -filter {NAME =~ *u_cic2*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic2*integ_reg*}]

set_multicycle_path -setup 20 \
    -from [get_cells -hier -filter {NAME =~ *u_cic2*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic2*comb_prev*}]
set_multicycle_path -hold 19 \
    -from [get_cells -hier -filter {NAME =~ *u_cic2*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic2*comb_prev*}]

## ---------------------------------------------------------------------------
## CIC3_I / CIC3_Q (R=10, post-mixer): advance every 500 clocks (= 2*10*25,
## the mix_v rate at the 250 kS/s post-CIC2 sample point).
## ---------------------------------------------------------------------------
set_multicycle_path -setup 500 \
    -from [get_cells -hier -filter {NAME =~ *u_cic3_*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic3_*integ_reg*}]
set_multicycle_path -hold 499 \
    -from [get_cells -hier -filter {NAME =~ *u_cic3_*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic3_*integ_reg*}]

set_multicycle_path -setup 500 \
    -from [get_cells -hier -filter {NAME =~ *u_cic3_*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic3_*comb_prev*}]
set_multicycle_path -hold 499 \
    -from [get_cells -hier -filter {NAME =~ *u_cic3_*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic3_*comb_prev*}]

## ---------------------------------------------------------------------------
## CIC4_I / CIC4_Q (R=25, post-mixer last stage): advance every 5000 clocks
## (= 2 * 10 * 25 * 10, the cic3_*_v rate at the 25 kS/s point).
## ---------------------------------------------------------------------------
set_multicycle_path -setup 5000 \
    -from [get_cells -hier -filter {NAME =~ *u_cic4_*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic4_*integ_reg*}]
set_multicycle_path -hold 4999 \
    -from [get_cells -hier -filter {NAME =~ *u_cic4_*integ_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic4_*integ_reg*}]

set_multicycle_path -setup 5000 \
    -from [get_cells -hier -filter {NAME =~ *u_cic4_*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic4_*comb_prev*}]
set_multicycle_path -hold 4999 \
    -from [get_cells -hier -filter {NAME =~ *u_cic4_*comb_prev*}] \
    -to   [get_cells -hier -filter {NAME =~ *u_cic4_*comb_prev*}]

## ---------------------------------------------------------------------------
## Adaptive threshold / RSSI / env_min / noise slow IIRs in dcf77_bit_det
## only update at the 1 kS/s env_valid tick.  False-path them; they have no
## functional timing requirement beyond eventually-consistent.
## ---------------------------------------------------------------------------
set_false_path -from [get_cells -hier -filter {NAME =~ *u_bit_det*env_max_reg*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *u_bit_det*env_min_reg*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *u_bit_det*noise_iir_reg*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *u_bit_det*threshold_reg*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *u_bit_det*hyst_reg*}]

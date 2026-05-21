#!/usr/bin/env bash
set -euo pipefail
rm -rf work-obj08.cf *.vcd
GHDL=${GHDL:-ghdl}
STD=${STD:---std=08}
$GHDL -a $STD parse_q_pkg.vhd
$GHDL -a $STD cdc_sync.vhd
$GHDL -a $STD level1_adder_tree.vhd
$GHDL -a $STD input_pre_shifter.vhd
$GHDL -a $STD parse_q_lane.vhd
$GHDL -a $STD parse_q_controller.vhd
$GHDL -a $STD parse_q_top.vhd
$GHDL -a $STD parse_q_tb.vhd
$GHDL -e $STD parse_q_tb
$GHDL -r $STD parse_q_tb --vcd=parse_q_partA.vcd
$GHDL -a $STD parse_q_tb_partB.vhd
$GHDL -e $STD parse_q_tb_partB
$GHDL -r $STD parse_q_tb_partB --vcd=parse_q_partB.vcd

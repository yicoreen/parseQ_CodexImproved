# ParseQ_Dev_260514_fix1 notes

This package contains a first-pass static fix set for GHDL/VHDL-2008 simulation.

## Fixes applied

1. `parse_q_pkg.vhd`
   - Updated `fsm_state_t` names to match `parse_q_controller.vhd`:
     `S_CHK_ALIGN`, `S_PROT_ALIGN`, `S_CHK_ADD`, `S_PROT_ADD`.

2. `parse_q_tb.vhd`
   - Moved `calc_partial_s()` and `calc_golden_result()` from the architecture statement region into the architecture declarative region.
   - Replaced the nested `random_block : declare ... begin ... end` block with process-local variables and ordinary sequential statements.
   - Updated `proc_send_pim_group()` to wait until `stall='0'` before sending a new group.

3. `parse_q_tb_partB.vhd`
   - Removed a duplicated `end loop;` in `calc_partial_s()`.
   - Replaced the nested `random_run : declare ... begin ... end` block with process-local variables and ordinary sequential statements.
   - Updated `proc_send_pim_group()` to wait until `stall='0'` before sending a new group.

4. `level1_adder_tree.vhd`
   - Rewrote the sign-bit term `w(7)` to avoid relying on 11-bit signed overflow when `ps_in(7)=8`.

## Run

```bash
./run_ghdl.sh
```

The current container did not have GHDL installed, so this fix set was not executed here. It is prepared for a local GHDL/ModelSim run.

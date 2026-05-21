# AGENTS.md

## Project context

This repository implements a VHDL RTL prototype of a Parse-Q PIM accelerator accumulator path.

Parse-Q replaces a conventional wide periphery accumulator for bit-serial weight-stationary SRAM-PIM with a compact bidirectional shift-register accumulator plus a shared exponent n_e.

The represented value is:

true_value = acc_phys * 2^(n_e)

The current target is not algorithm exploration. The immediate goal is to make the provided RTL and testbenches compile, elaborate, and pass simulation with GHDL or ModelSim while preserving the intended microarchitecture.

## Core architecture assumptions

The PIM array processes sparse bit-serial input activations.

Input activation q_in is 8-bit signed two's complement and is processed from bit 7 down to bit 0.

Bit 7 is the sign bit and must be handled as a subtract operation.

Each q_in bit-strip has length N = 1024.

For a given input bit-strip, the PIM selects only positions whose bit value is 1.

The PIM processes 8 selected positions per sparse group.

For each output channel and each sparse group, the PIM outputs 8 popcounts, one per weight bit position.

The PIM output type is conceptually:

F output channels x 8 weight bits x 4-bit popcount

The current default output-channel count is F = 64.

## Timing and CDC assumptions

The PIM array clock is 25 MHz, corresponding to 40 ns per cycle.

The Parse-Q core clock is 100 MHz, corresponding to 10 ns per cycle.

The PIM holds partial-sum output data stable long enough for the core to capture it.

Single-bit CDC signals such as pim_valid and pim_bit_done are synchronized with 2-FF synchronizers.

The multi-bit PIM data bus must not be synchronized with independent 2-FF synchronizers.

The multi-bit PIM data bus is captured by latch-enable or register-enable style logic when the synchronized valid event is observed.

## Computation hierarchy

Level 0 is the PIM sparse-group output:

F x 8 x 4-bit popcounts

Level 1 is a combinational adder tree:

8 popcounts -> 11-bit signed partial_s

The signed partial sum is:

partial_s = -ps[7] * 128 + ps[6] * 64 + ps[5] * 32 + ps[4] * 16 + ps[3] * 8 + ps[2] * 4 + ps[1] * 2 + ps[0]

Level 2 N-axis accumulation is not implemented here as a separate PIM-internal accumulation stage.

Parse-Q is applied starting from Level 1 partial_s.

Within the same q_in bit position, multiple sparse-group partial_s values are accumulated.

Across q_in bit positions, the accumulator follows MSB-first Horner-style accumulation:

acc = ((((-s7) * 2 + s6) * 2 + s5) * 2 + ... + s0)

At entry to a new q_in bit position, the accumulator performs ALIGN by left-shifting by 1, then performs a post-align overflow check.

For each sparse group, the datapath performs adder tree, pre-shift by n_e, accumulate, and post-add overflow check.

## Current numeric design choices

M_SAFE is 4.

ACC_WIDTH is Q + 1 + M_SAFE.

For the current Q=8 setting, ACC_WIDTH is 13.

PARTIAL_S_W is 11.

Do not revert M_SAFE back to 2.

The M_SAFE=4 choice is intentional to avoid overflow around worst-case additions such as an aligned accumulator value plus an 11-bit partial_s.

Protect shift loops intentionally have no fixed hardware iteration bound in the architecture.

If convergence or infinite stall is suspected, add simulation watchdogs or bounded testbench checks first.

Do not add arbitrary RTL loop bounds unless explicitly requested.

## Sign handling

q_in bit 7 is the two's-complement sign bit.

When processing q_in bit 7, the accumulator must subtract the corresponding partial_s contribution.

The chosen RTL approach is an add/sub shared adder using sub_mode.

Use conditional inversion plus carry-in where appropriate.

Do not add a separate negation datapath unless there is a verified functional or timing reason.

## Reset and start behavior

The reset strategy is not fully finalized.

The summary says that computation-unit reset should be controlled by start, but the current RTL may only clear lane accumulators on rst.

Do not silently change reset semantics.

If a simulation failure is related to back-to-back runs or stale accumulator state, first document whether the RTL currently requires rst between operations.

Then propose either:

1. reset-before-each-operation policy in the testbench, or
2. explicit lane clear on start in the RTL.

Ask for confirmation before making a large semantic reset change.

## Testbench policy

There are two testbenches:

parse_q_tb.vhd covers Part A tests TC1 through TC9.

parse_q_tb_partB.vhd covers Part B tests TC10 through TC18.

The testbench must respect stall/back-pressure.

If stall is asserted, the testbench must not send a new PIM group until stall is deasserted.

If there is a mismatch, first determine whether the golden model or the RTL is wrong.

Do not modify RTL only to match an incorrect golden model.

Do not modify the golden model only to hide an RTL error.

For each functional fix, report:

1. failing test name,
2. observed value,
3. expected value,
4. root cause,
5. files changed,
6. exact command used to verify the fix.

## Preferred command sequence

Use VHDL-2008.

Run:

./run_ghdl.sh

If debugging manually, use:

ghdl -a --std=08 parse_q_pkg.vhd
ghdl -a --std=08 level1_adder_tree.vhd
ghdl -a --std=08 input_pre_shifter.vhd
ghdl -a --std=08 parse_q_lane.vhd
ghdl -a --std=08 parse_q_controller.vhd
ghdl -a --std=08 cdc_sync.vhd
ghdl -a --std=08 parse_q_top.vhd
ghdl -a --std=08 parse_q_tb.vhd
ghdl -a --std=08 parse_q_tb_partB.vhd

Then elaborate and run the actual testbench entity names found in the files.

## Coding constraints

Keep changes minimal.

Preserve synthesizability unless the change is inside a testbench.

Do not rewrite the architecture broadly.

Do not introduce non-standard VHDL unless necessary.

Prefer explicit numeric_std conversions.

Avoid relying on signed overflow wrap-around behavior.

When changing bit widths, explain the numeric range before patching.

Do not add vendor-specific primitives.

## Done criteria

A task is done only when:

1. the relevant GHDL commands compile and elaborate,
2. the target testbench runs,
3. the final answer includes the exact command output summary,
4. the diff is minimal and reviewable,
5. no unrelated style rewrite was performed.

## Review guidelines

Flag any change that alters:

1. q_in bit order,
2. sign-bit subtract behavior,
3. M_SAFE or ACC_WIDTH,
4. n_e update semantics,
5. stall/back-pressure behavior,
6. reset/start semantics,
7. CDC treatment of the multi-bit PIM bus.

Treat these as high-risk changes requiring explicit explanation.

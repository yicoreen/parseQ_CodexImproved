library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

package parse_q_pkg is

  -- =========================================================
  -- Configurable Parameters
  -- =========================================================
  constant F           : positive := 64;
  constant Q           : positive := 8;
  constant Q_IN        : positive := 8;
  constant N           : positive := 1024;
  constant SPARSE_GRP  : positive := 8;
  constant W_BIT       : positive := 8;

  -- =========================================================
  -- Derived Parameters
  -- =========================================================
  constant PARTIAL_S_W : positive := integer(ceil(log2(real(SPARSE_GRP)))) + W_BIT;  -- 11
  constant M_SAFE_MIN  : positive := integer(ceil(real(PARTIAL_S_W - Q)));            -- 3
  constant M_SAFE      : positive := M_SAFE_MIN + 1;                                 -- 4
  constant ACC_WIDTH   : positive := Q + 1 + M_SAFE;                                 -- 13
  constant PAYLOAD_W   : positive := Q + 1;                                           -- 9
  constant PS_BIT_W    : positive := integer(ceil(log2(real(SPARSE_GRP)))) + 1;       -- 4
  constant NE_WIDTH    : positive := 5;

  -- =========================================================
  -- Types
  -- =========================================================
  type acc_array_t       is array (0 to F-1) of signed(ACC_WIDTH-1 downto 0);
  type payload_array_t   is array (0 to F-1) of signed(PAYLOAD_W-1 downto 0);
  type partial_s_array_t is array (0 to F-1) of signed(PARTIAL_S_W-1 downto 0);
  type ps_bit_array_t    is array (0 to W_BIT-1) of unsigned(PS_BIT_W-1 downto 0);
  type pim_output_t      is array (0 to F-1) of ps_bit_array_t;

  -- =========================================================
  -- FSM States
  -- =========================================================
  type fsm_state_t is (
    S_IDLE,
    S_ALIGN,
    S_CHK_ALIGN,
    S_PROT_ALIGN,
    S_WAIT_PIM,
    S_ADD,
    S_CHK_ADD,
    S_PROT_ADD,
    S_DONE
  );

end package parse_q_pkg;

package body parse_q_pkg is
end package body parse_q_pkg;
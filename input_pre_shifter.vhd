-- =============================================================
-- input_pre_shifter.vhd
--
-- [기능]
--   Level 1 adder tree에서 나온 partial_s (11-bit signed)를
--   shared exponent n_e만큼 arithmetic right shift (>>>) 합니다.
--
-- [배경]
--   Parse-Q에서 accumulator에 값을 더하기 전에,
--   현재까지의 protective shift 횟수(n_e)만큼 입력을 줄여야
--   accumulator의 스케일과 맞출 수 있습니다.
--
--     acc_phys ≈ true_value / 2^n_e
--     → 새 값도 2^n_e로 나눠서 더해야 스케일 일치
--     → partial_s_shifted = partial_s >>> n_e (arithmetic right shift)
--
-- [Arithmetic Right Shift란?]
--   sign bit를 유지하면서 우측 이동.
--   예: -100 (signed) >>> 2 = -25
--   양수는 floor division, 음수는 약간의 truncation error 발생
--   이 truncation error는 Parse-Q의 guard bit(Q+1)이 흡수함.
--
-- [구현]
--   이 모듈이 전체 Parse-Q 설계에서 유일한 barrel shifter입니다.
--   n_e는 보통 0~4 정도이므로, 5-stage mux (NE_WIDTH=5)로 구현.
--   각 stage는 n_e의 해당 bit가 1이면 shift, 0이면 bypass.
--
--   Stage 0: n_e[0]=1이면 >>>1,  아니면 bypass
--   Stage 1: n_e[1]=1이면 >>>2,  아니면 bypass
--   Stage 2: n_e[2]=1이면 >>>4,  아니면 bypass
--   Stage 3: n_e[3]=1이면 >>>8,  아니면 bypass
--   Stage 4: n_e[4]=1이면 >>>16, 아니면 bypass
--
--   이 logarithmic barrel shifter는 0~31 범위의 shift를
--   5단의 2:1 mux로 처리합니다. (combinational, 1 cycle)
--
-- [주의]
--   n_e가 PARTIAL_S_W(=11) 이상이면 결과는 0 또는 -1(음수).
--   VHDL의 shift_right(signed, ...) 가 이를 올바르게 처리합니다.
--
-- [인스턴스화]
--   parse_q_top에서 F개 채널에 대해 generate로 병렬 인스턴스화.
--   n_e는 모든 채널이 공유하는 단일 값입니다.
-- =============================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.parse_q_pkg.ALL;

entity input_pre_shifter is
  port (
    -- 입력: Level 1 adder tree 출력 (11-bit signed)
    partial_s       : in  signed(PARTIAL_S_W-1 downto 0);

    -- Shift amount: shared exponent (unsigned, 0~31)
    n_e             : in  unsigned(NE_WIDTH-1 downto 0);

    -- 출력: arithmetic right shifted 결과
    --   ACC_WIDTH(=13) 비트로 sign-extend하여 출력
    --   → 이후 adder에서 바로 acc_phys와 더할 수 있도록
    partial_s_shift : out signed(ACC_WIDTH-1 downto 0)
  );
end entity input_pre_shifter;

architecture rtl of input_pre_shifter is

  -- =============================================================
  -- 내부 signal 설명
  --
  -- extended: partial_s를 ACC_WIDTH(13bit)로 sign-extend
  --   → shift 전에 확장해야 상위 비트가 올바르게 채워짐
  --
  -- stg(0~4): 각 barrel shifter stage의 출력
  --   stg(0) = n_e[0] 적용 후
  --   stg(1) = n_e[1] 적용 후
  --   ...
  --   stg(4) = n_e[4] 적용 후 = 최종 결과
  -- =============================================================

  signal extended : signed(ACC_WIDTH-1 downto 0);

  -- Barrel shifter intermediate stages
  type stg_array_t is array (0 to NE_WIDTH-1) of signed(ACC_WIDTH-1 downto 0);
  signal stg : stg_array_t;

begin

  -- =============================================================
  -- Step 1: Sign Extension (11bit → 13bit)
  --
  -- resize(signed, n)는 MSB(sign)를 복제하여 확장합니다.
  -- 예: 11'b1_0000000000 → 13'b111_0000000000
  -- =============================================================

  extended <= resize(partial_s, ACC_WIDTH);

  -- =============================================================
  -- Step 2: Logarithmic Barrel Shifter (5 stages)
  --
  -- 각 stage i에서:
  --   n_e(i)=1 → shift_right by 2^i
  --   n_e(i)=0 → pass through
  --
  -- arithmetic shift: sign bit가 빈 자리를 채움
  -- =============================================================

  -- Stage 0: conditional >>>1
  stg(0) <= shift_right(extended, 1) when n_e(0) = '1'
            else extended;

  -- Stage 1~4: 각각 >>>2, >>>4, >>>8, >>>16
  gen_stages: for i in 1 to NE_WIDTH-1 generate
    stg(i) <= shift_right(stg(i-1), 2**i) when n_e(i) = '1'
              else stg(i-1);
  end generate gen_stages;

  -- =============================================================
  -- 출력: 마지막 stage가 최종 shifted 값
  -- =============================================================

  partial_s_shift <= stg(NE_WIDTH-1);

end architecture rtl;
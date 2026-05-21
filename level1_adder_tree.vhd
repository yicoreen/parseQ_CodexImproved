-- =============================================================
-- level1_adder_tree.vhd
--
-- [기능]
--   PIM에서 나온 8개의 popcount 값(ps[0]~ps[7])을 받아서
--   하나의 signed partial sum으로 합산합니다.
--
-- [수학적 배경]
--   2's complement weight의 N개 합에서, bit-position별로 분해하면:
--     Σ weight_i = -ps[7]*2^7 + ps[6]*2^6 + ... + ps[0]*2^0
--   여기서 ps[j] = 해당 bit position j에서 1인 weight의 개수 (popcount)
--   ps[j]의 범위: 0 ~ SPARSE_GRP(=8), 즉 4-bit unsigned
--
-- [구조]
--   3-stage combinational adder tree:
--     Stage 1: 인접 pair끼리 weighted add (bit7은 subtract)
--     Stage 2: Stage 1 결과 pair끼리 add
--     Stage 3: 최종 합산 → 11-bit signed 출력
--
-- [출력 범위]
--   최소: -(8 * 128) = -1024
--   최대: +(8 * 64 + 8 * 32 + ... + 8 * 1) = +1016
--   → 11-bit signed [-1024, +1016] 충분
--
-- [인스턴스화]
--   parse_q_top에서 F개 채널에 대해 generate로 병렬 인스턴스화
-- =============================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.parse_q_pkg.ALL;

entity level1_adder_tree is
  port (
    -- 입력: PIM에서 온 8개 popcount (각 4-bit unsigned, 0~8)
    --   ps_in(0) = bit0(LSB) popcount, ..., ps_in(7) = bit7(MSB/sign) popcount
    ps_in     : in  ps_bit_array_t;

    -- 출력: weighted sum, 11-bit signed
    partial_s : out signed(PARTIAL_S_W-1 downto 0)
  );
end entity level1_adder_tree;

architecture rtl of level1_adder_tree is

  -- =============================================================
  -- [Internal Signal 설명]
  --
  -- weighted(j): ps_in(j)에 2^j 가중치를 적용한 값
  --   - j=0..6: positive 가중치 → unsigned → signed 확장
  --   - j=7:    negative 가중치 (-2^7) → negate 처리
  --
  -- 각 weighted 값의 비트폭:
  --   weighted(0): ps(0)*1   → 4bit unsigned → 5bit signed  [0, +8]
  --   weighted(1): ps(1)*2   → 5bit unsigned → 6bit signed  [0, +16]
  --   weighted(2): ps(2)*4   → 6bit unsigned → 7bit signed  [0, +32]
  --   weighted(3): ps(3)*8   → 7bit unsigned → 8bit signed  [0, +64]
  --   weighted(4): ps(4)*16  → 8bit unsigned → 9bit signed  [0, +128]
  --   weighted(5): ps(5)*32  → 9bit unsigned → 10bit signed [0, +256]
  --   weighted(6): ps(6)*64  → 10bit unsigned→ 11bit signed [0, +512]
  --   weighted(7): -ps(7)*128→ 11bit signed                 [-1024, 0]
  --
  -- 최종 합산을 위해 모든 값을 PARTIAL_S_W(=11) 비트로 sign-extend
  -- =============================================================

  -- 각 bit position의 weighted value (모두 11-bit signed로 통일)
  type weighted_array_t is array (0 to W_BIT-1) of signed(PARTIAL_S_W-1 downto 0);
  signal w : weighted_array_t;

  -- Stage 1: 인접 pair 합산 결과 (4개)
  -- w01 = w(0) + w(1),  w23 = w(2) + w(3)
  -- w45 = w(4) + w(5),  w67 = w(6) + w(7)
  signal s1_01 : signed(PARTIAL_S_W-1 downto 0);
  signal s1_23 : signed(PARTIAL_S_W-1 downto 0);
  signal s1_45 : signed(PARTIAL_S_W-1 downto 0);
  signal s1_67 : signed(PARTIAL_S_W-1 downto 0);

  -- Stage 2: Stage 1 pair 합산 결과 (2개)
  -- s2_lo = s1_01 + s1_23,  s2_hi = s1_45 + s1_67
  signal s2_lo : signed(PARTIAL_S_W-1 downto 0);
  signal s2_hi : signed(PARTIAL_S_W-1 downto 0);

begin

  -- =============================================================
  -- Weighted Value 생성
  --
  -- ps_in(j)는 4-bit unsigned (0~8).
  -- shift_left로 2^j 가중치 적용 후, 11-bit signed로 확장.
  --
  -- bit 7 (sign bit)만 특별: -ps(7)*128
  --   구현: ps(7)를 shift_left 후 negate (2's complement)
  --   negate = NOT + 1 = 비트반전 후 +1
  -- =============================================================

  -- j=0..6: positive weighted values
  gen_positive: for j in 0 to W_BIT-2 generate
    -- unsigned 확장 → shift → signed 변환
    -- resize to PARTIAL_S_W unsigned, shift left by j, then cast to signed
    w(j) <= signed(resize(
               shift_left(resize(ps_in(j), PARTIAL_S_W), j),
               PARTIAL_S_W));
  end generate gen_positive;

  -- j=7: negative weighted value (-ps[7] * 2^7)
  -- 11-bit signed에서는 +1024를 표현할 수 없으므로,
  -- 먼저 12-bit에서 +ps(7)*128을 만든 뒤 negate하고 11-bit로 resize합니다.
  -- 이렇게 해야 ps(7)=8일 때도 -1024가 overflow 관성에 의존하지 않고 표현됩니다.
  w(7) <= resize(
              -signed(shift_left(resize(ps_in(7), PARTIAL_S_W + 1), 7)),
              PARTIAL_S_W);

  -- =============================================================
  -- Stage 1: 인접 pair 합산 (4 pairs → 4 sums)
  --
  -- 비트폭: 두 PARTIAL_S_W 값의 합 → 최대 PARTIAL_S_W+1 비트 필요
  -- 하지만 실제 범위가 PARTIAL_S_W 안에 들어오므로 유지
  --   예: w(0)+w(1) 최대 = 8+16 = 24, 11bit signed 충분
  --   예: w(6)+w(7) 범위 = [0+(-1024), 512+0] = [-1024, 512], 11bit ✓
  -- =============================================================

  s1_01 <= w(0) + w(1);
  s1_23 <= w(2) + w(3);
  s1_45 <= w(4) + w(5);
  s1_67 <= w(6) + w(7);

  -- =============================================================
  -- Stage 2: Stage 1 결과 pair 합산 (4 → 2)
  --
  -- s2_lo = s1_01 + s1_23: 범위 [0, 24+96] = [0, 120], 11bit ✓
  -- s2_hi = s1_45 + s1_67: 범위 [-1024, 128+256+512] = [-1024, 896], 11bit ✓
  -- =============================================================

  s2_lo <= s1_01 + s1_23;
  s2_hi <= s1_45 + s1_67;

  -- =============================================================
  -- Stage 3: 최종 합산 (2 → 1)
  --
  -- 범위: [-1024, 120+896] = [-1024, +1016], 11bit signed ✓
  -- =============================================================

  partial_s <= s2_lo + s2_hi;

end architecture rtl;
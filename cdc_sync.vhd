-- =============================================================
-- cdc_sync.vhd
--
-- [기능]
--   2-FF (Flip-Flop) Clock Domain Crossing Synchronizer.
--   PIM clock domain (25MHz) 에서 생성된 신호를
--   Parse-Q core clock domain (100MHz) 으로 안전하게 전달합니다.
--
-- [배경: CDC 문제]
--   서로 다른 clock domain의 신호를 직접 사용하면
--   metastability(준안정 상태)가 발생할 수 있습니다.
--   → FF 출력이 0도 1도 아닌 불확정 상태로 진동
--   → downstream 로직 오동작
--
--   2-FF synchronizer는 첫 번째 FF에서 metastability가
--   발생하더라도, 두 번째 FF의 setup time 전까지
--   안정화될 확률을 극도로 높여줍니다.
--   (MTBF > 수백 년, 일반적인 28nm 공정 기준)
--
-- [타이밍 특성]
--   PIM(25MHz) → core(100MHz) 전달 시:
--   - 최소 latency: 2 core cycles = 20ns
--   - 최대 latency: 3 core cycles = 30ns
--   - PIM 신호가 40ns 동안 유지되므로 충분히 캡처 가능
--
-- [사용처]
--   parse_q_top에서 아래 신호들에 대해 인스턴스화:
--   (1) pim_valid     → pim_valid_sync
--   (2) pim_bit_done  → pim_bit_done_sync
--
-- [주의]
--   이 synchronizer는 single-bit 신호 전용입니다.
--   multi-bit bus (예: partial sum 데이터)는 이 방식으로
--   동기화하면 안 됩니다. (bit별 latency 차이 → 잘못된 값)
--   → 데이터 bus는 pim_valid_sync의 latch enable로 캡처
-- =============================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity cdc_sync is
  port (
    -- 목적지 clock domain의 clock
    clk_dst  : in  std_logic;

    -- 목적지 clock domain의 reset (synchronous, active high)
    rst_dst  : in  std_logic;

    -- 입력: 출발지 clock domain의 신호 (비동기)
    sig_in   : in  std_logic;

    -- 출력: 목적지 clock domain에 동기화된 신호
    sig_out  : out std_logic
  );
end entity cdc_sync;

architecture rtl of cdc_sync is

  -- =============================================================
  -- 2-stage FF chain
  --
  -- ff1: 첫 번째 FF. metastability가 여기서 발생할 수 있음.
  --      이 출력은 직접 사용하지 않음.
  -- ff2: 두 번째 FF. ff1이 안정화된 값을 캡처.
  --      이 출력이 최종 동기화된 신호.
  --
  -- ASYNC_REG attribute:
  --   합성 도구에게 이 FF들이 CDC synchronizer임을 알림.
  --   → FF들을 물리적으로 가까이 배치 (routing delay 최소화)
  --   → metastability 해소 시간 확보에 유리
  -- =============================================================

  signal ff1 : std_logic;
  signal ff2 : std_logic;

  -- Synthesis attribute: Xilinx / Synopsys 공통
  attribute ASYNC_REG : string;
  attribute ASYNC_REG of ff1 : signal is "TRUE";
  attribute ASYNC_REG of ff2 : signal is "TRUE";

begin

  process(clk_dst)
  begin
    if rising_edge(clk_dst) then
      if rst_dst = '1' then
        ff1 <= '0';
        ff2 <= '0';
      else
        ff1 <= sig_in;   -- 1st stage: 비동기 입력 캡처
        ff2 <= ff1;      -- 2nd stage: 안정화된 값 캡처
      end if;
    end if;
  end process;

  sig_out <= ff2;

end architecture rtl;
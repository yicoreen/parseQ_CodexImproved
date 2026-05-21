-- =============================================================
-- parse_q_top.vhd
--
-- [기능]
--   Parse-Q 시스템의 최상위 모듈입니다.
--   PIM 배열로부터 popcount 데이터를 받아,
--   bit-serial accumulation + Parse-Q 압축을 수행하고,
--   최종 (payload, n_e) 쌍을 출력합니다.
--
-- [내부 구성]
--   ┌─────────────────────────────────────────────────────┐
--   │  parse_q_top                                        │
--   │                                                     │
--   │  ┌──────────┐  ┌──────────┐                        │
--   │  │cdc_sync  │  │cdc_sync  │  (PIM 신호 동기화)      │
--   │  │pim_valid │  │pim_done  │                         │
--   │  └────┬─────┘  └────┬─────┘                        │
--   │       │              │                              │
--   │  ┌────▼──────────────▼───┐                         │
--   │  │  parse_q_controller   │  (FSM + n_e counter)     │
--   │  │  cmd, sub_mode, n_e   │                         │
--   │  └──┬────┬────┬──────────┘                         │
--   │     │    │    │                                     │
--   │     │    │    │  ┌──── × F lanes (generate) ────┐  │
--   │     │    │    │  │                               │  │
--   │     │    │    │  │  ┌──────────────────┐        │  │
--   │     │    │    │  │  │ level1_adder_tree│ (comb) │  │
--   │     │    │    │  │  │ 8×4b → 11b       │        │  │
--   │     │    │    │  │  └────────┬─────────┘        │  │
--   │     │    │    │  │           │                   │  │
--   │     │    │    │  │  ┌───────▼──────────┐        │  │
--   │     │    │    │  │  │input_pre_shifter │ (comb) │  │
--   │     │    │    │  │  │ 11b >>> n_e → 13b│        │  │
--   │     │    │    │  │  └────────┬─────────┘        │  │
--   │     │    │    │  │           │                   │  │
--   │     │    │    │  │  ┌───────▼──────────┐        │  │
--   │     │    │    │  │  │  parse_q_lane    │ (seq)  │  │
--   │     │    │    │  │  │  shift reg+detect│        │  │
--   │     │    │    │  │  │  +add/sub        │        │  │
--   │     │    │    │  │  └────────┬─────────┘        │  │
--   │     │    │    │  │      fire_k, payload_k       │  │
--   │     │    │    │  └──────────────────────────────┘  │
--   │     │    │    │                                     │
--   │     │    │  ┌─▼──────────────────┐                 │
--   │     │    │  │ Group-OR: fire_any │ (comb)          │
--   │     │    │  └────────────────────┘                 │
--   │     │    │                                          │
--   └─────┴────┴──────────────────────────────────────────┘
--
-- [클럭 도메인]
--   clk_pim  (25MHz):  PIM 배열 동작 클럭
--   clk_core (100MHz): Parse-Q 내부 로직 클럭
--   CDC synchronizer를 통해 pim_valid, pim_bit_done을 동기화
--   PIM 데이터 bus는 pim_valid 동기화 후 latch로 캡처
--
-- [데이터 흐름 타이밍 (1회 sparse group 처리)]
--
--   Core cycle 0: pim_valid rising edge 감지
--                 → latch_pim=1 → PIM 데이터 레지스터 캡처
--                 → FSM: S_WAIT_PIM → S_ADD
--
--   Core cycle 1: S_ADD → cmd=LOADADD
--                 (adder tree + pre-shifter + adder: combinational)
--                 → 다음 rising edge에서 acc_phys 갱신
--                 → FSM: S_ADD → S_CHK_ADD
--
--   Core cycle 2: S_CHK_ADD → fire_any 확인
--                 fire=0: 다음 sparse group 대기 또는 bit pos 전환
--                 fire=1: S_PROT_ADD → n_e++, 다시 확인 (반복)
--
--   ∴ 최소 2 core cycles per sparse group (fire 없을 때)
--     PIM은 40ns(=4 core cycles)마다 데이터 공급
--     → 충분한 마진 확보 ✓
-- =============================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.parse_q_pkg.ALL;

entity parse_q_top is
  port (
    -- ─── Clocks ───
    clk_core          : in  std_logic;   -- 100MHz, Parse-Q 내부
    clk_pim           : in  std_logic;   -- 25MHz, PIM 배열 (CDC 참조용, 직접 사용 안 함)

    -- ─── Reset ───
    rst               : in  std_logic;   -- synchronous to clk_core, active high

    -- ─── Control ───
    start             : in  std_logic;   -- 1-cycle pulse: 연산 시작
    done              : out std_logic;   -- 연산 완료

    -- ─── PIM Interface (clk_pim domain) ───
    -- pim_valid: PIM이 유효한 popcount 데이터를 출력 중 (clk_pim 동기)
    pim_valid         : in  std_logic;
    -- pim_bit_done: 현재 bit position의 마지막 sparse group (clk_pim 동기)
    pim_bit_done      : in  std_logic;
    -- pim_data: F채널 × 8개 weight bit × 4-bit popcount
    -- PIM latch에서 ~40ns 유지 → clk_core에서 안전하게 캡처 가능
    pim_data          : in  pim_output_t;

    -- ─── Output ───
    -- payload: F개 채널의 최종 quantized 결과 (각 Q+1=9 bits signed)
    payload_out       : out payload_array_t;
    -- n_e: shared exponent (unsigned, protective shift 횟수)
    n_e_out           : out unsigned(NE_WIDTH-1 downto 0);

    -- ─── Back-pressure ───
    stall             : out std_logic
  );
end entity parse_q_top;

architecture rtl of parse_q_top is

  -- =============================================================
  -- CDC 동기화된 PIM 제어 신호
  -- =============================================================
  signal pim_valid_sync    : std_logic;
  signal pim_bit_done_sync : std_logic;

  -- =============================================================
  -- PIM 데이터 레지스터
  -- pim_valid rising edge 시 캡처하여 core domain에서 안정적으로 사용
  -- =============================================================
  signal pim_data_reg : pim_output_t;
  signal latch_pim    : std_logic;

  -- =============================================================
  -- Controller ↔ Lane 인터페이스
  -- =============================================================
  signal cmd      : std_logic_vector(1 downto 0);
  signal sub_mode : std_logic;
  signal n_e_int  : unsigned(NE_WIDTH-1 downto 0);
  signal fire_any : std_logic;

  -- =============================================================
  -- Per-lane 중간 신호
  -- =============================================================

  -- Level 1 adder tree 출력 (각 채널별 11-bit signed)
  signal partial_s     : partial_s_array_t;

  -- Pre-shifter 출력 (각 채널별 13-bit signed, n_e만큼 right-shifted)
  signal partial_s_shifted : acc_array_t;

  -- 각 lane의 fire 신호 (margin 침범 감지)
  signal fire_vec      : std_logic_vector(F-1 downto 0);

  -- 각 lane의 payload 출력
  signal payload_int   : payload_array_t;

  -- Debug
  signal bit_pos_dbg   : unsigned(3 downto 0);

begin

  -- =============================================================
  -- (1) CDC Synchronizers
  --
  -- PIM domain (25MHz) → Core domain (100MHz) 신호 동기화
  -- 각각 2-FF synchronizer 사용
  -- =============================================================

  u_cdc_valid : entity work.cdc_sync
    port map (
      clk_dst => clk_core,
      rst_dst => rst,
      sig_in  => pim_valid,
      sig_out => pim_valid_sync
    );

  u_cdc_bit_done : entity work.cdc_sync
    port map (
      clk_dst => clk_core,
      rst_dst => rst,
      sig_in  => pim_bit_done,
      sig_out => pim_bit_done_sync
    );

  -- =============================================================
  -- (2) PIM Data Latch
  --
  -- latch_pim 펄스 시 PIM 데이터를 core domain 레지스터에 캡처.
  --
  -- PIM 데이터(pim_data)는 multi-bit bus이므로 2-FF CDC 불가.
  -- 대신 pim_valid_sync (동기화 완료) 기반 latch enable 방식 사용.
  -- PIM 데이터가 ~40ns 유지되고, latch는 pim_valid rising edge
  -- 후 수 core cycle 내에 발생하므로 데이터가 안정된 상태.
  --
  -- 구조: F채널 × 8 weight bits × 4-bit popcount
  -- =============================================================

  process(clk_core)
  begin
    if rising_edge(clk_core) then
      if rst = '1' then
        -- 초기화: 모든 popcount를 0으로
        for ch in 0 to F-1 loop
          for b in 0 to W_BIT-1 loop
            pim_data_reg(ch)(b) <= (others => '0');
          end loop;
        end loop;
      elsif latch_pim = '1' then
        -- PIM 출력 캡처
        pim_data_reg <= pim_data;
      end if;
    end if;
  end process;

  -- =============================================================
  -- (3) Controller
  --
  -- FSM + n_e counter + stall/done 관리
  -- 모든 lane에 동일한 cmd, sub_mode, n_e를 broadcast
  -- =============================================================

  u_controller : entity work.parse_q_controller
    port map (
      clk               => clk_core,
      rst               => rst,
      start             => start,
      fire_any          => fire_any,
      pim_valid_sync    => pim_valid_sync,
      pim_bit_done_sync => pim_bit_done_sync,
      cmd               => cmd,
      sub_mode          => sub_mode,
      n_e               => n_e_int,
      latch_pim         => latch_pim,
      stall             => stall,
      done              => done,
      bit_pos_out       => bit_pos_dbg
    );

  -- =============================================================
  -- (4) F-Lane Datapath (Generate)
  --
  -- 각 채널(k = 0 .. F-1)에 대해 동일한 구조를 병렬 생성:
  --
  --   pim_data_reg(k) → level1_adder_tree → partial_s(k)
  --                                              │
  --                     input_pre_shifter ← n_e  │
  --                          │                    │
  --                          ▼                    ▼
  --                     partial_s_shifted(k)
  --                          │
  --                     parse_q_lane ← cmd, sub_mode
  --                          │
  --                     fire_vec(k), payload_int(k)
  -- =============================================================

  gen_lanes : for k in 0 to F-1 generate

    -- ─── Level 1 Adder Tree (Combinational) ───
    -- 8개 popcount → 1개 signed weighted sum
    u_adder_tree : entity work.level1_adder_tree
      port map (
        ps_in     => pim_data_reg(k),
        partial_s => partial_s(k)
      );

    -- ─── Input Pre-Shifter (Combinational) ───
    -- partial_s >>> n_e, ACC_WIDTH로 sign-extend
    u_pre_shifter : entity work.input_pre_shifter
      port map (
        partial_s       => partial_s(k),
        n_e             => n_e_int,
        partial_s_shift => partial_s_shifted(k)
      );

    -- ─── Parse-Q Lane (Sequential) ───
    -- Bidirectional shift register + XOR detector + adder
    u_lane : entity work.parse_q_lane
      port map (
        clk               => clk_core,
        rst               => rst,
        cmd               => cmd,
        sub_mode          => sub_mode,
        partial_s_shifted => partial_s_shifted(k),
        fire              => fire_vec(k),
        payload           => payload_int(k)
      );

  end generate gen_lanes;

  -- =============================================================
  -- (5) Group-OR: 모든 lane의 fire 신호를 하나로 합침
  --
  -- 어느 한 lane이라도 margin 침범 → fire_any = 1
  -- → controller가 모든 lane에 protective shift 명령
  --
  -- 이것이 "shared exponent" 방식의 핵심:
  --   한 lane의 overflow가 전체 lane을 shift시킴
  --   → 일부 lane은 불필요하게 shift될 수 있음
  --   → 이 오차를 guard bit (Q+1번째 비트)가 흡수
  --
  -- 구현: OR reduction, 순수 combinational
  -- 비용: F-1개의 OR gate = 63개 (무시할 수준)
  -- =============================================================

  fire_any <= '1' when unsigned(fire_vec) /= 0
              else '0';

  -- =============================================================
  -- (6) Output Assignment
  -- =============================================================

  -- 최종 payload: 각 lane의 하위 Q+1 비트
  payload_out <= payload_int;

  -- 공유 지수
  n_e_out <= n_e_int;

end architecture rtl;
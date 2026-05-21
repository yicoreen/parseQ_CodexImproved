-- 패키지에서 FSM 상태를 아래로 교체해주세요:
--   type fsm_state_t is (
--     S_IDLE,
--     S_ALIGN,
--     S_CHK_ALIGN,
--     S_PROT_ALIGN,
--     S_WAIT_PIM,
--     S_ADD,
--     S_CHK_ADD,
--     S_PROT_ADD,
--     S_DONE
--   );

-- =============================================================
-- parse_q_controller.vhd
--
-- [기능]
--   Parse-Q 전체 연산을 제어하는 FSM + 공유 n_e 카운터.
--
-- [FSM 상태 전이 요약]
--
--   S_IDLE ──(start)──▶ S_WAIT_PIM  (첫 bit position은 align 불필요)
--
--   S_WAIT_PIM ──(pim_valid↑)──▶ S_ADD ──▶ S_CHK_ADD
--       │                                      │
--       │                          fire=1 ──▶ S_PROT_ADD ──▶ S_CHK_ADD (반복)
--       │                          fire=0, not last_group ──▶ S_WAIT_PIM
--       │                          fire=0, last_group, bit_pos>0 ──▶ S_ALIGN
--       │                          fire=0, last_group, bit_pos=0 ──▶ S_DONE
--       │
--   S_ALIGN ──▶ S_CHK_ALIGN
--                    │
--        fire=1 ──▶ S_PROT_ALIGN ──▶ S_CHK_ALIGN (반복)
--        fire=0 ──▶ S_WAIT_PIM
--
-- [클럭 도메인]
--   본 모듈은 core clock (100MHz) 도메인에서 동작합니다.
--   pim_valid_sync, pim_bit_done_sync는 이미 2-FF CDC를
--   거친 신호라고 가정합니다 (top에서 동기화).
--
-- [pim_valid 엣지 검출]
--   PIM은 25MHz로 동작하므로 pim_valid_sync가 ~4 core cycle 동안
--   high를 유지합니다. 같은 데이터를 중복 처리하지 않기 위해
--   rising edge를 검출하여 1회만 처리합니다.
--
-- [sub_mode]
--   q_in의 MSB(bit 7)는 sign bit이므로, 이 자리를 처리할 때
--   accumulator에서 빼기(subtract)를 수행해야 합니다.
--   bit_pos == Q_IN-1 일 때 sub_mode=1.
--
-- [stall]
--   Parse-Q가 PIM 데이터를 받을 준비가 안 되었을 때 assert.
--   PIM은 stall=1이면 다음 sparse group 전송을 보류합니다.
--   S_WAIT_PIM, S_IDLE, S_DONE 에서만 stall=0.
-- =============================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.parse_q_pkg.ALL;

entity parse_q_controller is
  port (
    -- ─── Clock & Reset ───
    clk               : in  std_logic;
    rst               : in  std_logic;   -- synchronous, active high

    -- ─── Computation Start ───
    -- 1-cycle pulse로 연산 시작. acc, n_e 등 초기화 후 동작 시작.
    start             : in  std_logic;

    -- ─── Fire Signal (from Group-OR of all lanes) ───
    -- combinational: 어느 한 lane이라도 margin 침범 시 '1'
    fire_any          : in  std_logic;

    -- ─── PIM Interface (core clock domain으로 동기화 완료) ───
    pim_valid_sync    : in  std_logic;   -- PIM이 유효한 데이터를 출력 중
    pim_bit_done_sync : in  std_logic;   -- 현재 bit position의 마지막 sparse group

    -- ─── Lane Control Outputs ───
    -- cmd: 모든 F개 lane에 동일하게 broadcast
    --   "00"=HOLD, "01"=ALIGN, "10"=PROTECT, "11"=LOADADD
    cmd               : out std_logic_vector(1 downto 0);

    -- sub_mode: 1이면 lane adder가 subtract 수행 (q_in sign bit 처리)
    sub_mode          : out std_logic;

    -- ─── Shared Exponent ───
    -- 모든 lane의 pre-shifter에 공급. protective shift 횟수 누적값.
    n_e               : out unsigned(NE_WIDTH-1 downto 0);

    -- ─── PIM Data Latch Enable ───
    -- 1-cycle pulse: top에서 PIM 출력 데이터를 레지스터에 캡처하는 타이밍
    latch_pim         : out std_logic;

    -- ─── Back-pressure to PIM ───
    stall             : out std_logic;

    -- ─── Computation Complete ───
    done              : out std_logic;

    -- ─── Debug: 현재 처리 중인 q_in bit position ───
    bit_pos_out       : out unsigned(3 downto 0)
  );
end entity parse_q_controller;

architecture rtl of parse_q_controller is

  -- =============================================================
  -- Command Encoding (parse_q_lane과 동일)
  -- =============================================================
  constant CMD_HOLD    : std_logic_vector(1 downto 0) := "00";
  constant CMD_ALIGN   : std_logic_vector(1 downto 0) := "01";
  constant CMD_PROTECT : std_logic_vector(1 downto 0) := "10";
  constant CMD_LOADADD : std_logic_vector(1 downto 0) := "11";

  -- =============================================================
  -- Internal Registers
  -- =============================================================

  -- FSM 현재 상태
  signal state : fsm_state_t;

  -- q_in bit position 카운터
  -- Q_IN-1(=7, MSB/sign) 부터 0(LSB)까지 감소
  -- 4비트면 0~15 표현 가능 (Q_IN ≤ 16 지원)
  signal bit_pos : unsigned(3 downto 0);

  -- 공유 지수 카운터 (protective shift 횟수 누적)
  signal n_e_reg : unsigned(NE_WIDTH-1 downto 0);

  -- "마지막 sparse group" 플래그
  -- pim_valid 수신 시 pim_bit_done_sync를 래치하여,
  -- ADD→CHK_ADD 이후 다음 동작 결정에 사용
  signal last_group : std_logic;

  -- =============================================================
  -- PIM Valid Rising Edge Detection
  --
  -- pim_valid_sync는 PIM clock (25MHz) 기준 1 cycle 동안 high
  -- → core clock (100MHz)에서 ~4 cycle 동안 high로 관측됨
  -- → rising edge만 검출하여 1회만 처리
  -- =============================================================
  signal pim_valid_d    : std_logic;   -- 1-cycle delayed version
  signal pim_valid_rise : std_logic;   -- rising edge pulse

begin

  -- =============================================================
  -- Rising Edge Detector for pim_valid_sync
  -- =============================================================
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        pim_valid_d <= '0';
      else
        pim_valid_d <= pim_valid_sync;
      end if;
    end if;
  end process;

  -- pim_valid_rise: 이전 cycle 0, 현재 cycle 1 → rising edge
  pim_valid_rise <= pim_valid_sync and not pim_valid_d;

  -- =============================================================
  -- Main FSM (Synchronous Process)
  --
  -- [타이밍 관계 설명]
  --
  -- Cycle N: FSM이 S_ADD에 진입
  --   → cmd = CMD_LOADADD (combinational output)
  --   → lane의 add_result가 combinational으로 계산됨
  --   → rising edge of cycle N+1: acc_phys <= add_result
  --
  -- Cycle N+1: FSM이 S_CHK_ADD에 진입
  --   → fire_any는 새 acc_phys에서 combinational으로 생성
  --   → FSM이 fire_any를 보고 다음 상태 결정
  --
  -- 즉, cmd를 발행한 다음 cycle에 결과를 확인하는 구조.
  -- PROTECT도 동일: 발행 → 다음 cycle에 fire 재확인.
  -- =============================================================
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        state      <= S_IDLE;
        bit_pos    <= (others => '0');
        n_e_reg    <= (others => '0');
        last_group <= '0';

      else
        case state is

          -- ─── IDLE: 연산 시작 대기 ───
          when S_IDLE =>
            if start = '1' then
              -- 초기화: MSB(sign bit)부터 시작, n_e=0
              bit_pos    <= to_unsigned(Q_IN - 1, 4);
              n_e_reg    <= (others => '0');
              last_group <= '0';
              -- 첫 bit position은 align 불필요 → 바로 PIM 대기
              state <= S_WAIT_PIM;
            end if;

          -- ─── ALIGN: acc_phys <<= 1 (새 bit position 진입) ───
          -- 이 cycle에 cmd=ALIGN 발행 → 다음 edge에 shift 적용
          when S_ALIGN =>
            state <= S_CHK_ALIGN;

          -- ─── CHK_ALIGN: align 후 overflow 확인 ───
          -- fire_any는 이미 갱신된 acc_phys에서 combinational
          when S_CHK_ALIGN =>
            if fire_any = '1' then
              -- margin 침범 → protective shift 필요
              state <= S_PROT_ALIGN;
            else
              -- safe → PIM 데이터 대기
              state <= S_WAIT_PIM;
            end if;

          -- ─── PROT_ALIGN: post-align protective shift ───
          -- acc_phys >>>= 1 (all lanes), n_e += 1
          when S_PROT_ALIGN =>
            n_e_reg <= n_e_reg + 1;
            -- 다음 cycle에 다시 fire 확인
            state <= S_CHK_ALIGN;

          -- ─── WAIT_PIM: PIM의 다음 sparse group 데이터 대기 ───
          when S_WAIT_PIM =>
            if pim_valid_rise = '1' then
              -- PIM 데이터 도착
              -- 마지막 group인지 래치 (ADD 후 방향 결정에 사용)
              last_group <= pim_bit_done_sync;
              -- 다음 state에서 adder 결과 로드
              state <= S_ADD;
            end if;

          -- ─── ADD: acc_phys += (or -=) partial_s_shifted ───
          -- cmd=LOADADD 발행 → 다음 edge에 add_result 로드
          when S_ADD =>
            state <= S_CHK_ADD;

          -- ─── CHK_ADD: add 후 overflow 확인 + 다음 동작 결정 ───
          when S_CHK_ADD =>
            if fire_any = '1' then
              -- margin 침범 → protective shift
              state <= S_PROT_ADD;
            else
              -- safe → 다음 동작 결정
              if last_group = '1' then
                -- 현재 bit position의 모든 sparse group 처리 완료
                if bit_pos = 0 then
                  -- 모든 bit position 완료 → 연산 종료
                  state <= S_DONE;
                else
                  -- 다음 bit position으로 이동 (하위 자리)
                  bit_pos <= bit_pos - 1;
                  state   <= S_ALIGN;
                end if;
              else
                -- 같은 bit position 내 다음 sparse group 대기
                state <= S_WAIT_PIM;
              end if;
            end if;

          -- ─── PROT_ADD: post-add protective shift ───
          when S_PROT_ADD =>
            n_e_reg <= n_e_reg + 1;
            state   <= S_CHK_ADD;

          -- ─── DONE: 연산 완료, 결과 유효 ───
          -- rst가 올 때까지 유지
          when S_DONE =>
            null;

          when others =>
            state <= S_IDLE;

        end case;
      end if;
    end if;
  end process;

  -- =============================================================
  -- Output: cmd (Combinational)
  --
  -- 각 FSM 상태에서 lane에 발행하는 명령:
  --   S_ALIGN      → <<1 (자리수 정렬)
  --   S_PROT_ALIGN → >>>1 (post-align 보호)
  --   S_ADD        → adder 결과 로드
  --   S_PROT_ADD   → >>>1 (post-add 보호)
  --   그 외         → 유지 (변화 없음)
  -- =============================================================
  process(state)
  begin
    case state is
      when S_ALIGN      => cmd <= CMD_ALIGN;
      when S_PROT_ALIGN => cmd <= CMD_PROTECT;
      when S_ADD        => cmd <= CMD_LOADADD;
      when S_PROT_ADD   => cmd <= CMD_PROTECT;
      when others       => cmd <= CMD_HOLD;
    end case;
  end process;

  -- =============================================================
  -- Output: sub_mode
  --
  -- q_in의 MSB (bit position = Q_IN-1 = 7)는 sign bit.
  -- 2's complement 곱셈에서 sign bit의 가중치는 -2^(Q_IN-1)이므로
  -- 이 자리를 처리할 때는 subtract를 수행해야 합니다.
  --
  -- bit_pos는 register이므로 glitch-free.
  -- =============================================================
  sub_mode <= '1' when bit_pos = to_unsigned(Q_IN - 1, 4)
              else '0';

  -- =============================================================
  -- Output: latch_pim
  --
  -- S_WAIT_PIM에서 pim_valid rising edge 감지 시 1-cycle pulse.
  -- 이 pulse의 rising edge에서 top module이 PIM 출력 데이터를
  -- 레지스터에 캡처합니다.
  --
  -- 타이밍: latch_pim=1인 clock edge에서:
  --   (1) PIM 데이터가 레지스터에 캡처됨
  --   (2) FSM이 S_ADD로 전이
  --   → S_ADD cycle에서 캡처된 데이터로 adder tree 동작
  -- =============================================================
  latch_pim <= '1' when (state = S_WAIT_PIM and pim_valid_rise = '1')
               else '0';

  -- =============================================================
  -- Output: stall (Back-pressure to PIM)
  --
  -- PIM에게 "아직 처리 중이니 다음 group 보내지 마세요" 신호.
  -- Parse-Q가 데이터를 받을 수 있는 상태에서만 해제.
  --
  -- 실제로는 Parse-Q(100MHz)가 PIM(25MHz)보다 4배 빨라서
  -- stall이 발생하는 경우는 드물지만, 안전을 위해 유지.
  -- =============================================================
  stall <= '0' when (state = S_WAIT_PIM or
                     state = S_IDLE or
                     state = S_DONE)
           else '1';

  -- =============================================================
  -- Output: done
  -- =============================================================
  done <= '1' when state = S_DONE else '0';

  -- =============================================================
  -- Output: shared exponent n_e
  -- =============================================================
  n_e <= n_e_reg;

  -- =============================================================
  -- Output: bit_pos (debug / monitoring)
  -- =============================================================
  bit_pos_out <= bit_pos;

end architecture rtl;
-- =============================================================
-- parse_q_lane.vhd
--
-- [기능]
--   Parse-Q의 per-lane 핵심 로직 1개 채널분입니다.
--   F개 채널 중 하나를 담당하며, top에서 generate로 복제됩니다.
--
--   내부 구성:
--     (1) Bidirectional Shift Register (= accumulator)
--         - <<1 (align): 새 q_in bit position 진입 시
--         - >>1 (protect): overflow 감지 시 (arithmetic right shift)
--         - hold: 값 유지
--         - load_add: signed 덧셈/뺄셈 결과를 로드
--
--     (2) XOR Overflow Detector
--         - margin bits(상위 M_SAFE개)가 sign extension인지 검사
--         - sign bit과 margin bit이 다르면 → fire 신호 발생
--         - 순수 combinational (XOR + OR)
--
--     (3) Add/Sub Adder
--         - acc_phys + partial_s_shifted (add mode, 일반 bit position)
--         - acc_phys - partial_s_shifted (sub mode, q_in MSB=sign bit일 때)
--         - sub mode 구현: XOR로 입력 반전 + carry_in=1
--           → 별도 negation 블록 불필요, adder에 통합
--
-- [비트 레이아웃] (ACC_WIDTH = 13, M_SAFE = 4, PAYLOAD_W = 9)
--
--   Bit:  [12]  [11] [10] [9] [8]  [7] [6] [5] [4] [3] [2] [1] [0]
--         sign  |<-- M_SAFE=4 -->|  |<------- Q+1=9 payload ------->|
--         sign   m3   m2   m1  m0    p8  p7  p6  p5  p4  p3  p2  p1  p0
--
--   Safe 조건: m3=m2=m1=m0 = sign (sign extension)
--   Fire 조건: 하나라도 sign과 다르면 fire=1
--
-- [인터페이스 제어 신호] (FSM에서 공급)
--   cmd = "00": HOLD (변화 없음)
--   cmd = "01": ALIGN (<<1, LSB에 0 삽입)
--   cmd = "10": PROTECT (>>>1, arithmetic right shift, sign 유지)
--   cmd = "11": LOAD_ADD (adder 결과를 shift register에 로드)
--
-- [포트 정리]
--   입력: clk, rst, cmd, sub_mode, partial_s_shifted
--   출력: fire (overflow 감지), payload (최종 Q+1 bit 추출)
-- =============================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.parse_q_pkg.ALL;

entity parse_q_lane is
  port (
    -- Clock & Reset
    clk              : in  std_logic;
    rst              : in  std_logic;  -- synchronous reset, active high

    -- Control (FSM에서 공급)
    cmd              : in  std_logic_vector(1 downto 0);
    --   "00" = HOLD
    --   "01" = ALIGN (<<1)
    --   "10" = PROTECT (>>>1)
    --   "11" = LOAD_ADD (adder 결과 로드)

    -- Sub mode: 1이면 subtract (q_in MSB 처리 시)
    sub_mode         : in  std_logic;

    -- Pre-shifted partial sum (input_pre_shifter 출력, 13-bit signed)
    partial_s_shifted: in  signed(ACC_WIDTH-1 downto 0);

    -- Overflow detection output (이 lane의 fire 상태)
    -- combinational: acc_phys가 바뀌면 즉시 반영
    fire             : out std_logic;

    -- Final payload output (Q+1 = 9 bits signed)
    -- done 시점에 FSM이 읽어감
    payload          : out signed(PAYLOAD_W-1 downto 0)
  );
end entity parse_q_lane;

architecture rtl of parse_q_lane is

  -- =============================================================
  -- 상수 정의
  -- =============================================================

  -- Command encoding
  constant CMD_HOLD    : std_logic_vector(1 downto 0) := "00";
  constant CMD_ALIGN   : std_logic_vector(1 downto 0) := "01";
  constant CMD_PROTECT : std_logic_vector(1 downto 0) := "10";
  constant CMD_LOADADD : std_logic_vector(1 downto 0) := "11";

  -- =============================================================
  -- 내부 신호
  -- =============================================================

  -- Bidirectional Shift Register (= accumulator)
  -- 이것이 Parse-Q의 핵심: 별도 accumulator + shifter가 아닌,
  -- 하나의 shift register가 저장, 정렬, 보호를 모두 수행
  signal acc_phys : signed(ACC_WIDTH-1 downto 0);

  -- Adder/Subtractor 출력
  -- add_result = acc_phys ± partial_s_shifted
  signal add_result : signed(ACC_WIDTH-1 downto 0);

  -- Margin bits와 sign bit 추출 (overflow detection용)
  signal sign_bit   : std_logic;
  signal margin_bits: std_logic_vector(M_SAFE-1 downto 0);

  -- 각 margin bit의 XOR 결과 (sign과 다르면 1)
  signal margin_xor : std_logic_vector(M_SAFE-1 downto 0);

begin

  -- =============================================================
  -- (1) Add/Sub Adder
  --
  -- sub_mode = 0: add_result = acc_phys + partial_s_shifted
  -- sub_mode = 1: add_result = acc_phys - partial_s_shifted
  --
  -- 하드웨어 구현:
  --   subtract = NOT(input) + 1 과 동일
  --   → XOR로 conditional invert + carry_in으로 +1
  --   → 이것이 option (b)의 핵심: 기존 adder에 XOR만 추가
  --
  -- VHDL에서는 +/- 연산자로 기술하고,
  -- 합성 도구가 이를 add/sub 공유 adder로 최적화합니다.
  -- =============================================================

  add_result <= acc_phys + partial_s_shifted when sub_mode = '0'
                else acc_phys - partial_s_shifted;

  -- =============================================================
  -- (2) Bidirectional Shift Register (Synchronous)
  --
  -- clk rising edge에서 cmd에 따라 동작:
  --
  -- HOLD:     변화 없음. stall 또는 wait 상태에서 사용.
  --
  -- ALIGN:    acc_phys <= acc_phys <<< 1
  --           새 q_in bit position 진입 시 자리수 정렬.
  --           기존 값에 ×2 효과. LSB에 0 삽입.
  --           logical left shift (signed에서도 동일).
  --
  -- PROTECT:  acc_phys <= acc_phys >>> 1
  --           overflow 감지 시 값을 절반으로 축소.
  --           arithmetic right shift: sign bit 유지.
  --           n_e가 1 증가하여 스케일 보상 (FSM에서 처리).
  --
  -- LOAD_ADD: acc_phys <= add_result
  --           adder/subtractor 결과를 레지스터에 로드.
  --           partial sum 누적의 핵심 동작.
  --
  -- RST:      acc_phys <= 0
  --           새 출력 벡터 계산 시작 시 초기화.
  -- =============================================================

  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        acc_phys <= (others => '0');
      else
        case cmd is
          when CMD_HOLD =>
            -- 유지
            null;

          when CMD_ALIGN =>
            -- Left shift by 1: ×2 정렬
            -- shift_left(signed, 1): MSB 버려지고 LSB에 0
            acc_phys <= shift_left(acc_phys, 1);

          when CMD_PROTECT =>
            -- Arithmetic right shift by 1: ÷2 보호
            -- shift_right(signed, 1): MSB(sign) 복제
            acc_phys <= shift_right(acc_phys, 1);

          when CMD_LOADADD =>
            -- Adder 결과 로드
            acc_phys <= add_result;

          when others =>
            null;
        end case;
      end if;
    end if;
  end process;

  -- =============================================================
  -- (3) XOR Overflow Detector (Combinational)
  --
  -- acc_phys의 MSB(sign)와 margin bits를 비교합니다.
  --
  -- 정상 상태(safe):
  --   margin bits = sign extension
  --   즉, acc_phys[12] = acc_phys[11] = acc_phys[10] = acc_phys[9] = acc_phys[8]
  --   → 실제 값이 payload(9bit) 안에 있음
  --
  -- 위험 상태(fire):
  --   하나라도 sign과 다르면 값이 margin 영역까지 침범한 것
  --   → protective shift 필요
  --
  -- 검출 방법:
  --   sign XOR margin_bit → 같으면 0(safe), 다르면 1(fire)
  --   OR로 모아서 → 하나라도 1이면 이 lane은 fire
  --
  -- 비용: M_SAFE개의 XOR gate + (M_SAFE-1)개의 OR gate
  --       M_SAFE=4일 때: 4 XOR + 3 OR = 7 gates per lane
  -- =============================================================

  -- Sign bit 추출 (MSB)
  sign_bit <= acc_phys(ACC_WIDTH-1);

  -- Margin bits 추출 (sign 바로 아래 M_SAFE개)
  -- bit [ACC_WIDTH-2] ~ bit [ACC_WIDTH-1-M_SAFE]
  -- = bit [11] ~ bit [8] (M_SAFE=4일 때)
  gen_margin: for i in 0 to M_SAFE-1 generate
    margin_bits(i) <= std_logic(acc_phys(ACC_WIDTH - 2 - i));
  end generate gen_margin;

  -- XOR: sign과 각 margin bit 비교
  gen_xor: for i in 0 to M_SAFE-1 generate
    margin_xor(i) <= sign_bit xor margin_bits(i);
  end generate gen_xor;

  -- OR reduction: 하나라도 다르면 fire
  -- VHDL에서 or_reduce 대신 명시적으로 구현
  fire <= '1' when unsigned(margin_xor) /= 0
          else '0';

  -- =============================================================
  -- (4) Payload 추출 (Combinational)
  --
  -- 계산 완료 후 FSM이 S_DONE 상태에서 읽어가는 값.
  -- acc_phys의 하위 PAYLOAD_W(=9) 비트를 추출합니다.
  --
  -- 정상 상태에서는 margin bits = sign extension이므로,
  -- payload의 MSB(bit 8) = acc_phys의 sign bit(bit 12) = 실제 부호
  -- → payload 자체가 올바른 signed 값
  --
  -- 비트 매핑:
  --   payload[8:0] = acc_phys[8:0]
  --   (acc_phys[12:9]는 모두 sign extension → 버려도 정보 손실 없음)
  -- =============================================================

  payload <= acc_phys(PAYLOAD_W-1 downto 0);

end architecture rtl;
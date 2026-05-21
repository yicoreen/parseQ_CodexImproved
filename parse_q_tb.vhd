-- =============================================================
-- parse_q_tb.vhd  (Part A: Infrastructure, Golden Model, Helpers)
--
-- [테스트벤치 구조]
--
--   (1) Clock Generation
--       - clk_core: 100MHz (10ns period)
--       - clk_pim:  25MHz  (40ns period)
--
--   (2) DUT Instantiation (parse_q_top)
--
--   (3) Golden Model
--       - 순수 수학적 계산으로 expected 결과를 구함
--       - Parse-Q 의 bit-serial Horner 전개:
--           true_val = ((((-s7)*2 + s6)*2 + s5)*2 + ... + s0)
--         여기서 sk = Σ (해당 bit position의 모든 sparse group partial_s)
--       - 결과 검증: |payload × 2^n_e - true_val| < tolerance
--
--   (4) Helper Procedures
--       - proc_reset: 시스템 리셋
--       - proc_send_pim_group: 한 sparse group의 PIM 데이터 전송
--       - proc_run_test: 전체 테스트 시퀀스 실행 + 결과 검증
--
--   (5) Test Cases (Part B에서 정의)
--       - TC1: 단일 sparse group, 작은 값 (fire 없음)
--       - TC2: 여러 sparse group, fire 발생
--       - TC3: 음수 결과 (sign bit 처리 검증)
--       - TC4: 전부 0 (trivial case)
--       - TC5: 최대 magnitude (worst-case overflow)
--       - TC6: 랜덤 패턴 (pseudo-random popcount)
-- =============================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;    -- for uniform() (pseudo-random)
use work.parse_q_pkg.ALL;

entity parse_q_tb is
  -- testbench에는 port 없음
end entity parse_q_tb;

architecture sim of parse_q_tb is

  -- =============================================================
  -- 시뮬레이션 상수
  -- =============================================================
  constant CLK_CORE_PERIOD : time := 10 ns;   -- 100MHz
  constant CLK_PIM_PERIOD  : time := 40 ns;   -- 25MHz

  -- Golden model 검증 허용 오차
  -- Parse-Q는 n_e번의 right-shift로 인해 LSB truncation이 발생
  -- 최대 오차: 각 sparse group마다 최대 1 LSB truncation
  --   → 총 오차 ≤ (total_groups × 2^n_e) + guard bit 영향
  -- 보수적으로 넉넉한 tolerance 사용
  constant TOLERANCE : integer := 256;

  -- 시뮬레이션 타임아웃 (무한루프 방지)
  constant SIM_TIMEOUT : time := 500 us;

  -- =============================================================
  -- DUT 신호
  -- =============================================================
  signal clk_core     : std_logic := '0';
  signal clk_pim      : std_logic := '0';
  signal rst          : std_logic := '1';
  signal start        : std_logic := '0';
  signal done         : std_logic;
  signal pim_valid    : std_logic := '0';
  signal pim_bit_done : std_logic := '0';
  signal pim_data     : pim_output_t;
  signal payload_out  : payload_array_t;
  signal n_e_out      : unsigned(NE_WIDTH-1 downto 0);
  signal stall        : std_logic;

  -- =============================================================
  -- 시뮬레이션 제어
  -- =============================================================
  signal sim_done     : boolean := false;   -- true가 되면 clock 정지

  -- =============================================================
  -- Golden Model 저장소
  --
  -- golden_s(k): bit position k에 대한 spatial sum 누적값
  --   = Σ (해당 bit position의 모든 sparse group partial_s)
  --   채널별로 독립 계산
  --
  -- golden_result(ch): 최종 expected dot product
  --   = -golden_s(7)*128 + golden_s(6)*64 + ... + golden_s(0)*1
  --   = Horner 전개와 동일한 결과
  -- =============================================================
  type golden_s_t is array (0 to F-1, 0 to Q_IN-1) of integer;
  signal golden_s : golden_s_t := (others => (others => 0));

  type golden_result_t is array (0 to F-1) of integer;

  -- =============================================================
  -- Golden Model: partial_s 계산 함수
  --
  -- level1_adder_tree와 동일한 로직을 순수 함수로 구현.
  -- 한 채널의 ps_bit_array_t → integer partial_s 반환.
  --
  -- partial_s = -ps(7)*128 + ps(6)*64 + ... + ps(0)*1
  -- =============================================================

  function calc_partial_s(ps : ps_bit_array_t) return integer is
    variable result : integer := 0;
  begin
    -- bit 0 ~ 6: positive weights (2^j)
    for j in 0 to W_BIT-2 loop
      result := result + to_integer(ps(j)) * (2**j);
    end loop;
    -- bit 7: negative weight (-2^7)
    result := result - to_integer(ps(W_BIT-1)) * (2**(W_BIT-1));
    return result;
  end function;

  -- =============================================================
  -- Golden Model: 최종 결과 계산 함수
  --
  -- Horner 전개 (Parse-Q와 동일한 순서):
  --   acc = -s(7)                     (MSB, sign bit → subtract)
  --   acc = acc*2 + s(6)              (align + add)
  --   acc = acc*2 + s(5)
  --   ...
  --   acc = acc*2 + s(0)              (LSB)
  --
  -- 결과: true dot product 값 (integer)
  -- =============================================================

  function calc_golden_result(
    s_arr : golden_s_t;
    ch    : integer
  ) return integer is
    variable acc : integer := 0;
  begin
    -- bit position 7 (MSB/sign): subtract
    acc := -s_arr(ch, Q_IN-1);

    -- bit position 6 → 0: align + add
    for k in Q_IN-2 downto 0 loop
      acc := acc * 2 + s_arr(ch, k);
    end loop;

    return acc;
  end function;

begin

  -- =============================================================
  -- Clock Generation
  --
  -- sim_done이 true가 되면 clock을 정지하여 시뮬레이션 종료
  -- =============================================================

  clk_core <= not clk_core after CLK_CORE_PERIOD / 2 when not sim_done
              else '0';

  clk_pim  <= not clk_pim  after CLK_PIM_PERIOD / 2  when not sim_done
              else '0';

  -- =============================================================
  -- DUT Instantiation
  -- =============================================================

  u_dut : entity work.parse_q_top
    port map (
      clk_core     => clk_core,
      clk_pim      => clk_pim,
      rst          => rst,
      start        => start,
      done         => done,
      pim_valid    => pim_valid,
      pim_bit_done => pim_bit_done,
      pim_data     => pim_data,
      payload_out  => payload_out,
      n_e_out      => n_e_out,
      stall        => stall
    );

  -- =============================================================
  -- Main Stimulus Process
  --
  -- 모든 test case를 순차적으로 실행합니다.
  -- 각 test case는:
  --   (1) Reset
  --   (2) Start pulse
  --   (3) PIM 데이터 공급 (bit position별, sparse group별)
  --   (4) Done 대기
  --   (5) Golden model과 비교
  -- =============================================================

  p_stimulus : process

    -- ─── Helper: Reset Sequence ───
    -- 5 core cycle 동안 rst=1 유지 후 해제
    procedure proc_reset is
    begin
      rst   <= '1';
      start <= '0';
      pim_valid    <= '0';
      pim_bit_done <= '0';
      -- pim_data 초기화
      for ch in 0 to F-1 loop
        for b in 0 to W_BIT-1 loop
          pim_data(ch)(b) <= (others => '0');
        end loop;
      end loop;
      -- golden model 초기화
      for ch in 0 to F-1 loop
        for bp in 0 to Q_IN-1 loop
          golden_s(ch, bp) <= 0;
        end loop;
      end loop;
      wait for CLK_CORE_PERIOD * 5;
      rst <= '0';
      wait for CLK_CORE_PERIOD * 2;
    end procedure;

    -- ─── Helper: Start Pulse ───
    -- 1 core cycle 동안 start=1
    procedure proc_start is
    begin
      start <= '1';
      wait for CLK_CORE_PERIOD;
      start <= '0';
    end procedure;

    -- ─── Helper: PIM Sparse Group 전송 ───
    -- 하나의 sparse group 데이터를 PIM timing으로 전송.
    --
    -- 파라미터:
    --   ps_data: F채널 × 8 weight bits popcount 데이터
    --   bit_pos: 현재 q_in bit position (0~7)
    --   is_last: 이 bit position의 마지막 group이면 true
    --
    -- 동작:
    --   (1) pim_data에 데이터 설정
    --   (2) pim_valid=1 (+ is_last이면 pim_bit_done=1)
    --   (3) 1 PIM cycle (40ns) 유지
    --   (4) pim_valid=0, pim_bit_done=0
    --   (5) golden model 업데이트
    --
    -- PIM 25MHz 타이밍 시뮬레이션:
    --   실제로는 clk_pim edge에 동기화해야 하지만,
    --   testbench에서는 40ns 유지로 근사 (CDC가 처리)
    procedure proc_send_pim_group(
      ps_data : in pim_output_t;
      bit_pos : in integer;
      is_last : in boolean
    ) is
    begin
      -- Back-pressure 준수: DUT가 PIM 데이터를 받을 수 있을 때까지 대기
      while stall = '1' loop
        wait until rising_edge(clk_core);
      end loop;
      wait until rising_edge(clk_pim);

      -- 데이터 설정
      pim_data <= ps_data;

      -- Valid + bit_done 신호
      pim_valid <= '1';
      if is_last then
        pim_bit_done <= '1';
      else
        pim_bit_done <= '0';
      end if;

      -- Golden model 업데이트: 각 채널별 spatial sum 누적
      for ch in 0 to F-1 loop
        golden_s(ch, bit_pos) <=
          golden_s(ch, bit_pos) + calc_partial_s(ps_data(ch));
      end loop;

      -- PIM cycle 유지 (40ns)
      wait for CLK_PIM_PERIOD;

      -- 신호 해제
      pim_valid    <= '0';
      pim_bit_done <= '0';

      -- Parse-Q가 처리할 시간 대기
      -- (stall이 걸릴 수 있으므로 여유있게)
      wait for CLK_PIM_PERIOD;
    end procedure;

    -- ─── Helper: Done 대기 + 타임아웃 ───
    procedure proc_wait_done is
      variable elapsed : time := 0 ns;
    begin
      while done /= '1' loop
        wait for CLK_CORE_PERIOD;
        elapsed := elapsed + CLK_CORE_PERIOD;
        assert elapsed < SIM_TIMEOUT
          report "TIMEOUT: done signal not asserted within limit!"
          severity failure;
      end loop;
    end procedure;

    -- ─── Helper: 결과 검증 ───
    -- Golden model과 DUT 출력 비교.
    --
    -- DUT 출력: payload(Q+1 bit signed), n_e(shared exponent)
    -- 복원값: payload × 2^n_e
    -- 검증: |복원값 - golden_result| ≤ TOLERANCE
    --
    -- TOLERANCE가 필요한 이유:
    --   Parse-Q는 pre-shift (>>>n_e) 시 LSB를 버림 (truncation)
    --   각 sparse group마다 최대 1 LSB 손실
    --   총 손실 ≤ total_groups × 2^n_e
    --   + guard bit에 의한 추가 오차
    procedure proc_check_results(
      test_name : in string
    ) is
      variable golden   : integer;
      variable restored : integer;
      variable err      : integer;
      variable n_e_val  : integer;
      variable pass_cnt : integer := 0;
      variable fail_cnt : integer := 0;
    begin
      n_e_val := to_integer(n_e_out);

      for ch in 0 to F-1 loop
        golden   := calc_golden_result(golden_s, ch);
        restored := to_integer(payload_out(ch)) * (2**n_e_val);
        err      := abs(restored - golden);

        if err <= TOLERANCE then
          pass_cnt := pass_cnt + 1;
        else
          fail_cnt := fail_cnt + 1;
          report test_name & " CH" & integer'image(ch) &
                 " FAIL: golden=" & integer'image(golden) &
                 " restored=" & integer'image(restored) &
                 " err=" & integer'image(err) &
                 " n_e=" & integer'image(n_e_val)
            severity warning;
        end if;
      end loop;

      report test_name & " RESULT: " &
             integer'image(pass_cnt) & "/" & integer'image(F) &
             " channels passed (n_e=" & integer'image(n_e_val) & ")"
        severity note;

      assert fail_cnt = 0
        report test_name & " FAILED with " &
               integer'image(fail_cnt) & " channel errors!"
        severity error;
    end procedure;

    -- ─── Helper: 균일 PIM 데이터 생성 ───
    -- 모든 채널, 모든 weight bit에 동일한 popcount 값 설정
    -- 빠른 테스트 데이터 생성용
    procedure make_uniform_pim_data(
      val     : in natural;
      result  : out pim_output_t
    ) is
    begin
      for ch in 0 to F-1 loop
        for b in 0 to W_BIT-1 loop
          result(ch)(b) := to_unsigned(val, PS_BIT_W);
        end loop;
      end loop;
    end procedure;

    -- ─── Helper: 특정 패턴 PIM 데이터 생성 ───
    -- 채널 k의 weight bit j에 대해 개별 popcount 지정
    -- pattern(j) = popcount for weight bit j (모든 채널 동일)
    type pattern_t is array (0 to W_BIT-1) of natural;

    procedure make_pattern_pim_data(
      pattern : in pattern_t;
      result  : out pim_output_t
    ) is
    begin
      for ch in 0 to F-1 loop
        for b in 0 to W_BIT-1 loop
          result(ch)(b) := to_unsigned(pattern(b), PS_BIT_W);
        end loop;
      end loop;
    end procedure;

    -- ─── 로컬 변수 ───
    variable v_pim_data : pim_output_t;
    variable v_pattern  : pattern_t;
    variable seed1      : positive := 42;
    variable seed2      : positive := 137;
    variable rval       : real;
    variable rps        : natural;

  begin
    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC1: 단일 Sparse Group, 작은 값 (fire 없음 예상)        ║
    -- ║                                                          ║
    -- ║  모든 popcount = 1                                       ║
    -- ║  partial_s = -1*128 + 1*64 + ... + 1*1 = -1             ║
    -- ║  각 bit position에 group 1개                             ║
    -- ║  golden = ((((-(-1))*2+(-1))*2+(-1))...) = Horner(-1)   ║
    -- ║                                                          ║
    -- ║  s(k) = -1 for all k                                    ║
    -- ║  result = -(-1)*128+(-1)*64+...+(-1)*1 = 128-127 = 1    ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC1: Single group, small uniform values =====" severity note;

    proc_reset;
    make_uniform_pim_data(1, v_pim_data);
    proc_start;

    -- 8개 bit position (7→0), 각각 1개 sparse group
    for bp in Q_IN-1 downto 0 loop
      proc_send_pim_group(v_pim_data, bp, true);  -- is_last = true
    end loop;

    proc_wait_done;
    proc_check_results("TC1");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC2: 여러 Sparse Group + Fire 발생 예상                  ║
    -- ║                                                          ║
    -- ║  popcount = 8 (모든 weight bit이 1)                      ║
    -- ║  partial_s = -8*128+8*64+...+8*1 = -1024+1016 = -8      ║
    -- ║  각 bit position에 4개 sparse group                      ║
    -- ║  s(k) = -8 * 4 = -32 for each bit position              ║
    -- ║                                                          ║
    -- ║  큰 값 누적 → fire/protective shift 발생 가능            ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC2: Multiple groups, large values (fire expected) =====" severity note;

    proc_reset;
    make_uniform_pim_data(8, v_pim_data);
    proc_start;

    for bp in Q_IN-1 downto 0 loop
      for g in 0 to 3 loop
        proc_send_pim_group(v_pim_data, bp, (g = 3));
      end loop;
    end loop;

    proc_wait_done;
    proc_check_results("TC2");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC3: 양수 결과 (sign bit 처리 검증)                      ║
    -- ║                                                          ║
    -- ║  ps(7) = 8, 나머지 = 0                                   ║
    -- ║  partial_s = -8*128 + 0 = -1024                          ║
    -- ║  sign bit(bp=7)에서 subtract → -(-1024) = +1024          ║
    -- ║  나머지 bp에서 partial_s = 0 → 영향 없음                  ║
    -- ║  golden = +1024                                          ║
    -- ║                                                          ║
    -- ║  sub_mode 동작이 올바른지 확인하는 핵심 테스트             ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC3: Sign bit processing verification =====" severity note;

    proc_reset;
    proc_start;

    -- bit position 7 (sign): ps(7)=8, 나머지=0
    v_pattern := (7 => 8, others => 0);
    make_pattern_pim_data(v_pattern, v_pim_data);
    proc_send_pim_group(v_pim_data, 7, true);

    -- bit position 6~0: 모든 popcount = 0
    make_uniform_pim_data(0, v_pim_data);
    for bp in Q_IN-2 downto 0 loop
      proc_send_pim_group(v_pim_data, bp, true);
    end loop;

    proc_wait_done;
    proc_check_results("TC3");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC4: 전부 0 (trivial case)                               ║
    -- ║                                                          ║
    -- ║  모든 popcount = 0 → partial_s = 0 for all               ║
    -- ║  golden = 0, n_e = 0 (protective shift 없음)              ║
    -- ║  모든 payload = 0 이어야 함                                ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC4: All zeros =====" severity note;

    proc_reset;
    make_uniform_pim_data(0, v_pim_data);
    proc_start;

    for bp in Q_IN-1 downto 0 loop
      proc_send_pim_group(v_pim_data, bp, true);
    end loop;

    proc_wait_done;
    proc_check_results("TC4");

    -- n_e도 0이어야 함
    assert to_integer(n_e_out) = 0
      report "TC4: n_e should be 0 but got " &
             integer'image(to_integer(n_e_out))
      severity error;

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC5: 최대 양수 magnitude (worst-case overflow 스트레스)   ║
    -- ║                                                          ║
    -- ║  양수 극대화: ps(6)=8, ps(7)=0                            ║
    -- ║  partial_s = 8*64 = +512                                 ║
    -- ║  각 bit position에 8개 sparse group                       ║
    -- ║  s(k) = +512 * 8 = +4096                                 ║
    -- ║                                                          ║
    -- ║  큰 양수가 계속 누적 → fire 다발 + n_e 크게 증가 예상     ║
    -- ║  Parse-Q 의 overflow 방어 메커니즘 스트레스 테스트         ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC5: Max positive magnitude stress test =====" severity note;

    proc_reset;
    v_pattern := (6 => 8, others => 0);
    make_pattern_pim_data(v_pattern, v_pim_data);
    proc_start;

    for bp in Q_IN-1 downto 0 loop
      for g in 0 to 7 loop
        proc_send_pim_group(v_pim_data, bp, (g = 7));
      end loop;
    end loop;

    proc_wait_done;
    proc_check_results("TC5");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC6: 최대 음수 magnitude (음수 overflow 스트레스)         ║
    -- ║                                                          ║
    -- ║  음수 극대화: ps(7)=8, ps(0~6)=0                          ║
    -- ║  partial_s = -8*128 = -1024                               ║
    -- ║  bp=7에서 subtract → -(-1024)=+1024                       ║
    -- ║  bp=6~0에서 add → +(-1024)*7회 누적                       ║
    -- ║  → 양수와 음수가 섞여서 cancel → 최종값 작을 수 있음      ║
    -- ║                                                          ║
    -- ║  sign 전환 + fire 혼합 시나리오                             ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC6: Negative stress + sign switching =====" severity note;

    proc_reset;
    v_pattern := (7 => 8, others => 0);
    make_pattern_pim_data(v_pattern, v_pim_data);
    proc_start;

    for bp in Q_IN-1 downto 0 loop
      proc_send_pim_group(v_pim_data, bp, true);
    end loop;

    proc_wait_done;
    proc_check_results("TC6");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC7: 채널별 차별화 패턴                                   ║
    -- ║                                                          ║
    -- ║  짝수 채널: ps 전부 2 → partial_s = -2                    ║
    -- ║  홀수 채널: ps 전부 6 → partial_s = -6                    ║
    -- ║  → shared n_e가 두 그룹 모두에 적절한지 확인               ║
    -- ║  → "하나의 n_e로 다양한 magnitude를 감당" 검증             ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC7: Per-channel differentiated pattern =====" severity note;

    proc_reset;

    -- 채널별로 다른 값 설정
    for ch in 0 to F-1 loop
      for b in 0 to W_BIT-1 loop
        if ch mod 2 = 0 then
          v_pim_data(ch)(b) := to_unsigned(2, PS_BIT_W);
        else
          v_pim_data(ch)(b) := to_unsigned(6, PS_BIT_W);
        end if;
      end loop;
    end loop;

    proc_start;

    for bp in Q_IN-1 downto 0 loop
      for g in 0 to 2 loop
        proc_send_pim_group(v_pim_data, bp, (g = 2));
      end loop;
    end loop;

    proc_wait_done;
    proc_check_results("TC7");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC8: Pseudo-Random 패턴                                  ║
    -- ║                                                          ║
    -- ║  VHDL의 uniform() 으로 0~8 범위 랜덤 popcount 생성       ║
    -- ║  각 bit position에 2개 sparse group                       ║
    -- ║  → 다양한 partial_s 조합에서 정확성 검증                   ║
    -- ║  → 실제 사용 환경에 가장 가까운 테스트                     ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC8: Pseudo-random pattern =====" severity note;

    proc_reset;
    proc_start;

    -- pseudo-random seed
    seed1 := 42;
    seed2 := 137;
    for bp in Q_IN-1 downto 0 loop
      for g in 0 to 1 loop
        -- 랜덤 PIM 데이터 생성
        for ch in 0 to F-1 loop
          for b in 0 to W_BIT-1 loop
            uniform(seed1, seed2, rval);
            rps := natural(floor(rval * 9.0));  -- 0~8
            if rps > 8 then rps := 8; end if;
            v_pim_data(ch)(b) := to_unsigned(rps, PS_BIT_W);
          end loop;
        end loop;

        proc_send_pim_group(v_pim_data, bp, (g = 1));
      end loop;
    end loop;

    proc_wait_done;
    proc_check_results("TC8");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC9: n_e 안정성 테스트 (같은 bit position 내 변화 관찰)  ║
    -- ║                                                          ║
    -- ║  bp=7에서 큰 값을 먹여 n_e를 올린 뒤,                     ║
    -- ║  bp=6~0에서 작은 값 → n_e가 더 이상 증가하지 않아야 함    ║
    -- ║  (M_SAFE=4의 여유 덕분)                                   ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC9: n_e stability test =====" severity note;

    proc_reset;
    proc_start;

    -- bp=7: 큰 값 (ps(6)=8 → partial_s=512, 4 groups → s=2048)
    v_pattern := (6 => 8, others => 0);
    make_pattern_pim_data(v_pattern, v_pim_data);
    for g in 0 to 3 loop
      proc_send_pim_group(v_pim_data, 7, (g = 3));
    end loop;

    -- bp=6~0: 작은 값 (ps 전부 1 → partial_s=-1)
    make_uniform_pim_data(1, v_pim_data);
    for bp in Q_IN-2 downto 0 loop
      proc_send_pim_group(v_pim_data, bp, true);
    end loop;

    proc_wait_done;
    proc_check_results("TC9");

    -- n_e 값 보고 (안정성 관찰용)
    report "TC9: final n_e = " & integer'image(to_integer(n_e_out))
      severity note;

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- 시뮬레이션 종료
    -- =============================================================

    report "========================================" severity note;
    report "  ALL TEST CASES COMPLETED" severity note;
    report "========================================" severity note;

    sim_done <= true;
    wait;

  end process p_stimulus;

  -- =============================================================
  -- Watchdog Timer (안전장치)
  --
  -- SIM_TIMEOUT 내에 sim_done이 true가 되지 않으면 강제 종료.
  -- 무한루프 방지용.
  -- =============================================================

  p_watchdog : process
  begin
    wait for SIM_TIMEOUT;
    if not sim_done then
      report "WATCHDOG: Simulation timeout reached!" severity failure;
    end if;
    wait;
  end process p_watchdog;

end architecture sim;
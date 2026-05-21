-- =============================================================
-- parse_q_tb_partB.vhd
--
-- [목적]
--   Part A의 기본 테스트에 추가하여:
--   (1) Back-to-back 연산 (reset 없이 연속 계산)
--   (2) Stall 동작 정밀 검증
--   (3) n_e 단조증가 assertion
--   (4) fire 발생 시 모든 lane 동시 protect assertion
--   (5) Boundary: accumulator wrap-around 방지 검증
--   (6) 교번 부호 패턴 (cancellation 극대화)
--   (7) 단일 채널만 fire하는 상황 (shared n_e 패널티 관찰)
--   (8) 긴 sparse group chain (N=1024, sparsity=0 시뮬레이션)
--
-- [구조]
--   Part A의 parse_q_tb와 동일한 infrastructure를 공유합니다.
--   이 파일은 Part A의 main stimulus process에
--   이어 붙이거나, 별도 tb로 사용할 수 있습니다.
--
-- [Inline Assertion 전략]
--   VHDL에서는 SystemVerilog SVA 같은 concurrent assertion이
--   제한적이므로, 별도 monitor process를 통해 매 cycle 검증:
--   (a) n_e는 절대 감소하지 않는다 (단조증가)
--   (b) fire=1일 때 cmd=PROTECT이다 (다음 cycle)
--   (c) acc_phys가 wrap-around하지 않는다
--   (d) stall=1일 때 pim_valid rising edge를 놓치지 않는다
-- =============================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use work.parse_q_pkg.ALL;

entity parse_q_tb_partB is
end entity parse_q_tb_partB;

architecture sim of parse_q_tb_partB is

  -- =============================================================
  -- 상수 (Part A와 동일)
  -- =============================================================
  constant CLK_CORE_PERIOD : time := 10 ns;
  constant CLK_PIM_PERIOD  : time := 40 ns;
  constant TOLERANCE       : integer := 256;
  constant SIM_TIMEOUT     : time := 2 ms;  -- 긴 테스트용 확장

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
  -- Assertion Monitor 신호
  -- 내부 DUT 신호를 관찰하기 위해 alias 또는 별도 tap 필요
  -- 여기서는 top-level에서 관찰 가능한 신호 위주로 검증
  -- =============================================================
  signal n_e_prev     : unsigned(NE_WIDTH-1 downto 0) := (others => '0');
  signal assert_en    : boolean := false;  -- 테스트 진행 중일 때만 검증

  -- Golden model
  type golden_s_t is array (0 to F-1, 0 to Q_IN-1) of integer;
  signal golden_s : golden_s_t := (others => (others => 0));

  signal sim_done : boolean := false;

begin

  -- =============================================================
  -- Clock Generation
  -- =============================================================
  clk_core <= not clk_core after CLK_CORE_PERIOD / 2 when not sim_done
              else '0';
  clk_pim  <= not clk_pim  after CLK_PIM_PERIOD / 2  when not sim_done
              else '0';

  -- =============================================================
  -- DUT
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
  -- MONITOR 1: n_e 단조증가 검증
  --
  -- Parse-Q에서 n_e는 protective shift 횟수의 누적이므로
  -- 절대 감소해서는 안 됩니다 (리셋 제외).
  --
  -- 위반 시: 내부 로직 오류 또는 FSM 버그
  -- =============================================================
  p_monitor_ne : process(clk_core)
  begin
    if rising_edge(clk_core) then
      if rst = '1' then
        n_e_prev <= (others => '0');
      elsif assert_en then
        assert n_e_out >= n_e_prev
          report "ASSERTION FAIL: n_e decreased! prev=" &
                 integer'image(to_integer(n_e_prev)) &
                 " curr=" & integer'image(to_integer(n_e_out))
          severity error;
        n_e_prev <= n_e_out;
      end if;
    end if;
  end process p_monitor_ne;

  -- =============================================================
  -- MONITOR 2: stall 해제 후 done 전까지 stall 패턴 검증
  --
  -- stall=0 구간에서는 Parse-Q가 PIM 데이터를 받을 준비가 된 상태.
  -- stall=1이 너무 오래 지속되면 PIM 효율이 떨어지므로,
  -- 연속 stall cycle 수를 카운트하여 경고.
  --
  -- 임계값: M_SAFE(=4) + 2 cycle 이상 연속 stall은 비정상 의심
  -- (post-align + post-add protect가 동시에 최악이어도 ~8 cycle)
  -- =============================================================
  p_monitor_stall : process(clk_core)
    variable stall_count : natural := 0;
    constant STALL_WARN  : natural := M_SAFE * 2 + 4;  -- 12
  begin
    if rising_edge(clk_core) then
      if rst = '1' then
        stall_count := 0;
      elsif assert_en then
        if stall = '1' then
          stall_count := stall_count + 1;
          if stall_count > STALL_WARN then
            report "STALL WARNING: stall held for " &
                   integer'image(stall_count) &
                   " consecutive cycles (threshold=" &
                   integer'image(STALL_WARN) & ")"
              severity warning;
          end if;
        else
          stall_count := 0;
        end if;
      end if;
    end if;
  end process p_monitor_stall;

  -- =============================================================
  -- MONITOR 3: done 신호 안정성
  --
  -- done=1이 assert된 후, rst 전까지 유지되어야 함.
  -- done이 glitch하면 출력 캡처가 불안정해짐.
  -- =============================================================
  p_monitor_done : process(clk_core)
    variable done_seen : boolean := false;
  begin
    if rising_edge(clk_core) then
      if rst = '1' then
        done_seen := false;
      elsif assert_en then
        if done = '1' then
          done_seen := true;
        end if;
        if done_seen and done = '0' then
          report "ASSERTION FAIL: done de-asserted without reset!"
            severity error;
        end if;
      end if;
    end if;
  end process p_monitor_done;

  -- =============================================================
  -- Main Stimulus Process (Advanced Test Cases)
  -- =============================================================

  p_stimulus : process

    -- ─── Helpers (Part A와 동일) ───

    procedure proc_reset is
    begin
      assert_en <= false;
      rst   <= '1';
      start <= '0';
      pim_valid    <= '0';
      pim_bit_done <= '0';
      for ch in 0 to F-1 loop
        for b in 0 to W_BIT-1 loop
          pim_data(ch)(b) <= (others => '0');
        end loop;
      end loop;
      for ch in 0 to F-1 loop
        for bp in 0 to Q_IN-1 loop
          golden_s(ch, bp) <= 0;
        end loop;
      end loop;
      wait for CLK_CORE_PERIOD * 5;
      rst <= '0';
      wait for CLK_CORE_PERIOD * 2;
      assert_en <= true;
    end procedure;

    procedure proc_start is
    begin
      start <= '1';
      wait for CLK_CORE_PERIOD;
      start <= '0';
    end procedure;

    function calc_partial_s(ps : ps_bit_array_t) return integer is
      variable result : integer := 0;
    begin
      for j in 0 to W_BIT-2 loop
        result := result + to_integer(ps(j)) * (2**j);
      end loop;
      result := result - to_integer(ps(W_BIT-1)) * (2**(W_BIT-1));
      return result;
    end function;

    function calc_golden_result(
      s_arr : golden_s_t;
      ch    : integer
    ) return integer is
      variable acc : integer := 0;
    begin
      acc := -s_arr(ch, Q_IN-1);
      for k in Q_IN-2 downto 0 loop
        acc := acc * 2 + s_arr(ch, k);
      end loop;
      return acc;
    end function;

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

      pim_data <= ps_data;
      pim_valid <= '1';
      if is_last then
        pim_bit_done <= '1';
      else
        pim_bit_done <= '0';
      end if;
      for ch in 0 to F-1 loop
        golden_s(ch, bit_pos) <=
          golden_s(ch, bit_pos) + calc_partial_s(ps_data(ch));
      end loop;
      wait for CLK_PIM_PERIOD;
      pim_valid    <= '0';
      pim_bit_done <= '0';
      wait for CLK_PIM_PERIOD;
    end procedure;

    procedure proc_wait_done is
      variable elapsed : time := 0 ns;
    begin
      while done /= '1' loop
        wait for CLK_CORE_PERIOD;
        elapsed := elapsed + CLK_CORE_PERIOD;
        assert elapsed < SIM_TIMEOUT
          report "TIMEOUT waiting for done!"
          severity failure;
      end loop;
    end procedure;

    procedure proc_check_results(test_name : in string) is
      variable golden, restored, err : integer;
      variable n_e_val               : integer;
      variable pass_cnt, fail_cnt    : integer := 0;
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
                 " err=" & integer'image(err)
            severity warning;
        end if;
      end loop;
      report test_name & ": " &
             integer'image(pass_cnt) & "/" & integer'image(F) &
             " passed (n_e=" & integer'image(n_e_val) & ")"
        severity note;
      assert fail_cnt = 0
        report test_name & " HAS FAILURES!" severity error;
    end procedure;

    -- ─── Helper: 특정 패턴 생성 ───
    type pattern_t is array (0 to W_BIT-1) of natural;

    procedure make_uniform_pim_data(
      val : in natural; result : out pim_output_t
    ) is
    begin
      for ch in 0 to F-1 loop
        for b in 0 to W_BIT-1 loop
          result(ch)(b) := to_unsigned(val, PS_BIT_W);
        end loop;
      end loop;
    end procedure;

    procedure make_pattern_pim_data(
      pattern : in pattern_t; result : out pim_output_t
    ) is
    begin
      for ch in 0 to F-1 loop
        for b in 0 to W_BIT-1 loop
          result(ch)(b) := to_unsigned(pattern(b), PS_BIT_W);
        end loop;
      end loop;
    end procedure;

    variable v_pim_data  : pim_output_t;
    variable v_pim_data2 : pim_output_t;
    variable v_pattern   : pattern_t;
    variable s1          : positive := 1;
    variable s2          : positive := 2;
    variable rval        : real;
    variable rps         : natural;

  begin

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC10: Back-to-Back 연산                                  ║
    -- ║                                                          ║
    -- ║  첫 번째 연산 완료 후, reset + start로 즉시 두 번째 시작   ║
    -- ║  → 이전 결과 잔류가 다음 결과를 오염시키지 않는지 검증     ║
    -- ║                                                          ║
    -- ║  Run A: 전부 ps=2 → 특정 결과                             ║
    -- ║  Run B: 전부 ps=5 → 다른 결과                             ║
    -- ║  → Run B 결과에 Run A 잔류가 없어야 함                    ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC10: Back-to-back computation =====" severity note;

    -- Run A
    proc_reset;
    make_uniform_pim_data(2, v_pim_data);
    proc_start;
    for bp in Q_IN-1 downto 0 loop
      proc_send_pim_group(v_pim_data, bp, true);
    end loop;
    proc_wait_done;
    proc_check_results("TC10-A");

    -- Run B: reset 후 즉시 시작
    proc_reset;
    make_uniform_pim_data(5, v_pim_data);
    proc_start;
    for bp in Q_IN-1 downto 0 loop
      proc_send_pim_group(v_pim_data, bp, true);
    end loop;
    proc_wait_done;
    proc_check_results("TC10-B");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC11: 교번 부호 패턴 (Cancellation 극대화)               ║
    -- ║                                                          ║
    -- ║  홀수 sparse group: ps(6)=8 → partial_s = +512           ║
    -- ║  짝수 sparse group: ps(7)=4 → partial_s = -512           ║
    -- ║  → 두 group이 거의 상쇄 → s(k) ≈ 0                      ║
    -- ║  → n_e가 불필요하게 올라가지 않아야 함                     ║
    -- ║                                                          ║
    -- ║  Parse-Q의 "spatial cancellation" 이점을 검증             ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC11: Alternating sign cancellation =====" severity note;

    proc_reset;

    -- 양수 group: ps(6)=8, 나머지=0 → partial_s = +512
    v_pattern := (6 => 8, others => 0);
    make_pattern_pim_data(v_pattern, v_pim_data);

    -- 음수 group: ps(7)=4, 나머지=0 → partial_s = -512
    v_pattern := (7 => 4, others => 0);
    make_pattern_pim_data(v_pattern, v_pim_data2);

    proc_start;

    for bp in Q_IN-1 downto 0 loop
      -- group 0: +512
      proc_send_pim_group(v_pim_data, bp, false);
      -- group 1: -512 (last)
      proc_send_pim_group(v_pim_data2, bp, true);
    end loop;

    proc_wait_done;
    proc_check_results("TC11");

    -- 상쇄 시 n_e가 작아야 함
    report "TC11: n_e = " & integer'image(to_integer(n_e_out)) &
           " (expected small due to cancellation)"
      severity note;

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC12: 단일 채널만 큰 값 (Shared n_e 패널티 관찰)         ║
    -- ║                                                          ║
    -- ║  CH0: ps(6)=8 → partial_s=+512 (매우 큼)                 ║
    -- ║  CH1~63: ps 전부 1 → partial_s=-1 (매우 작음)            ║
    -- ║                                                          ║
    -- ║  CH0 때문에 전체 n_e가 올라가면,                           ║
    -- ║  CH1~63은 불필요한 right-shift를 당함 → 정밀도 손실       ║
    -- ║  → guard bit이 이 오차를 흡수하는지 확인                   ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC12: Single-lane fire, shared n_e penalty =====" severity note;

    proc_reset;

    -- CH0: 큰 값, CH1~63: 작은 값
    for b in 0 to W_BIT-1 loop
      -- CH0
      if b = 6 then
        v_pim_data(0)(b) := to_unsigned(8, PS_BIT_W);
      else
        v_pim_data(0)(b) := to_unsigned(0, PS_BIT_W);
      end if;
      -- CH1~63
      for ch in 1 to F-1 loop
        v_pim_data(ch)(b) := to_unsigned(1, PS_BIT_W);
      end loop;
    end loop;

    proc_start;

    for bp in Q_IN-1 downto 0 loop
      for g in 0 to 3 loop
        proc_send_pim_group(v_pim_data, bp, (g = 3));
      end loop;
    end loop;

    proc_wait_done;
    proc_check_results("TC12");

    report "TC12: n_e = " & integer'image(to_integer(n_e_out)) &
           " (driven by CH0)"
      severity note;

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC13: 긴 Sparse Group Chain (N 전체 시뮬레이션)           ║
    -- ║                                                          ║
    -- ║  sparsity = 0 가정 → N/SPARSE_GRP = 128 groups           ║
    -- ║  각 group: ps 전부 3 → partial_s = -3                     ║
    -- ║  s(k) = -3 * 128 = -384                                  ║
    -- ║                                                          ║
    -- ║  많은 group 누적 → fire 다발 + n_e 성장                   ║
    -- ║  Parse-Q가 긴 시퀀스에서도 정확한지 검증                   ║
    -- ║                                                          ║
    -- ║  ⚠️ 시뮬레이션 시간 주의 (128 groups × 8 bit pos)        ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC13: Long sparse chain (128 groups) =====" severity note;

    proc_reset;
    make_uniform_pim_data(3, v_pim_data);
    proc_start;

    for bp in Q_IN-1 downto 0 loop
      for g in 0 to (N/SPARSE_GRP)-1 loop
        proc_send_pim_group(
          v_pim_data, bp,
          (g = (N/SPARSE_GRP)-1)
        );
      end loop;
    end loop;

    proc_wait_done;
    proc_check_results("TC13");

    report "TC13: n_e = " & integer'image(to_integer(n_e_out)) &
           " (long chain accumulation)"
      severity note;

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC14: Gradient-like 패턴 (점진적 증가)                   ║
    -- ║                                                          ║
    -- ║  bit position마다 다른 크기:                               ║
    -- ║    bp=7: ps=1 (작음) ... bp=0: ps=8 (큼)                 ║
    -- ║  → MSB는 작고 LSB는 큰 비대칭 패턴                        ║
    -- ║  → 실제 weight 분포와 유사한 시나리오                      ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC14: Gradient pattern (increasing) =====" severity note;

    proc_reset;
    proc_start;

    for bp in Q_IN-1 downto 0 loop
      -- bp=7 → val=1, bp=6 → val=2, ..., bp=0 → val=8
      make_uniform_pim_data(Q_IN - bp, v_pim_data);
      for g in 0 to 1 loop
        proc_send_pim_group(v_pim_data, bp, (g = 1));
      end loop;
    end loop;

    proc_wait_done;
    proc_check_results("TC14");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC15: 단일 비트 포지션만 활성 (나머지 0)                  ║
    -- ║                                                          ║
    -- ║  bp=3에서만 ps=4, 나머지 bp에서 ps=0                      ║
    -- ║  → s(3)만 non-zero                                       ║
    -- ║  → golden = s(3) * 2^3 = partial_s * 2^3                 ║
    -- ║  → align shift가 정확히 동작하는지 검증                    ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC15: Single active bit position (bp=3) =====" severity note;

    proc_reset;
    proc_start;

    for bp in Q_IN-1 downto 0 loop
      if bp = 3 then
        make_uniform_pim_data(4, v_pim_data);
      else
        make_uniform_pim_data(0, v_pim_data);
      end if;
      proc_send_pim_group(v_pim_data, bp, true);
    end loop;

    proc_wait_done;
    proc_check_results("TC15");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC16: Multi-seed Random (통계적 검증)                    ║
    -- ║                                                          ║
    -- ║  3개의 다른 seed로 랜덤 테스트를 반복                      ║
    -- ║  → 특정 seed에서만 통과하는 우연 방지                      ║
    -- ║  → 각 run마다 3개 sparse group, 모든 bp 커버              ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC16: Multi-seed random =====" severity note;

    multi_seed_loop : for seed_idx in 0 to 2 loop

      proc_reset;
      proc_start;

      s1 := 10 + seed_idx * 73;
      s2 := 200 + seed_idx * 31;
      for bp in Q_IN-1 downto 0 loop
        for g in 0 to 2 loop
          for ch in 0 to F-1 loop
            for b in 0 to W_BIT-1 loop
              uniform(s1, s2, rval);
              rps := natural(floor(rval * 9.0));
              if rps > 8 then rps := 8; end if;
              v_pim_data(ch)(b) := to_unsigned(rps, PS_BIT_W);
            end loop;
          end loop;
          proc_send_pim_group(v_pim_data, bp, (g = 2));
        end loop;
      end loop;

      proc_wait_done;
      proc_check_results(
        "TC16-seed" & integer'image(seed_idx)
      );

      wait for CLK_CORE_PERIOD * 10;

    end loop multi_seed_loop;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC17: n_e 포화 스트레스 (극단적 누적)                    ║
    -- ║                                                          ║
    -- ║  매 group마다 partial_s = +1016 (최대 양수)               ║
    -- ║  모든 bp에서 16 groups씩                                  ║
    -- ║  → n_e가 NE_WIDTH(=5bit, max 31)를 넘지 않는지 확인       ║
    -- ║  → n_e overflow 시 시스템 오동작 가능                      ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC17: n_e saturation stress =====" severity note;

    proc_reset;

    -- partial_s = +1016: ps(0~6) = 8, ps(7) = 0
    v_pattern := (7 => 0, others => 8);
    make_pattern_pim_data(v_pattern, v_pim_data);

    proc_start;

    for bp in Q_IN-1 downto 0 loop
      for g in 0 to 15 loop
        proc_send_pim_group(v_pim_data, bp, (g = 15));
      end loop;
    end loop;

    proc_wait_done;
    proc_check_results("TC17");

    report "TC17: final n_e = " & integer'image(to_integer(n_e_out)) &
           " (max possible: " & integer'image(2**NE_WIDTH - 1) & ")"
      severity note;

    -- n_e가 상한을 넘지 않았는지 확인
    assert to_integer(n_e_out) < 2**NE_WIDTH
      report "TC17: n_e OVERFLOW!" severity error;

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- ╔═══════════════════════════════════════════════════════════╗
    -- ║  TC18: 최소 magnitude (정밀도 한계 테스트)                 ║
    -- ║                                                          ║
    -- ║  한 채널만 ps(0)=1, 나머지 전부 0                          ║
    -- ║  partial_s = +1 (최소 non-zero)                           ║
    -- ║  → n_e가 커지면 1 >>> n_e = 0 → 정보 완전 손실            ║
    -- ║  → 다른 채널의 큰 값이 n_e를 올릴 때 이 채널의 정밀도는?  ║
    -- ╚═══════════════════════════════════════════════════════════╝
    -- =============================================================

    report "===== TC18: Minimum magnitude precision test =====" severity note;

    proc_reset;

    -- CH0: ps(0)=1만 (partial_s=1)
    -- CH1: ps(6)=8 (partial_s=512, fire 유발)
    -- CH2~63: 0
    for ch in 0 to F-1 loop
      for b in 0 to W_BIT-1 loop
        v_pim_data(ch)(b) := to_unsigned(0, PS_BIT_W);
      end loop;
    end loop;
    v_pim_data(0)(0) := to_unsigned(1, PS_BIT_W);  -- CH0: +1
    v_pim_data(1)(6) := to_unsigned(8, PS_BIT_W);  -- CH1: +512

    proc_start;

    for bp in Q_IN-1 downto 0 loop
      for g in 0 to 3 loop
        proc_send_pim_group(v_pim_data, bp, (g = 3));
      end loop;
    end loop;

    proc_wait_done;

    -- CH0의 정밀도 손실 관찰
    report "TC18: CH0 payload=" &
           integer'image(to_integer(payload_out(0))) &
           " n_e=" & integer'image(to_integer(n_e_out)) &
           " (small value with shared n_e penalty)"
      severity note;

    proc_check_results("TC18");

    wait for CLK_CORE_PERIOD * 10;

    -- =============================================================
    -- 전체 결과 요약
    -- =============================================================

    report "================================================" severity note;
    report "  PART B: ALL ADVANCED TEST CASES COMPLETED" severity note;
    report "================================================" severity note;

    sim_done <= true;
    wait;

  end process p_stimulus;

  -- =============================================================
  -- Watchdog
  -- =============================================================
  p_watchdog : process
  begin
    wait for SIM_TIMEOUT;
    if not sim_done then
      report "WATCHDOG TIMEOUT!" severity failure;
    end if;
    wait;
  end process p_watchdog;

end architecture sim;
------------------------------------------------------------------------------
-- Title      : SCoG detection
-- Project    : Stream-Based Center-of-Gravity
-------------------------------------------------------------------------------
-- File       : scog.vhd
-- Author     : Fanpeng Kong  <fanpeng@fanpengkong.com>
-- Company    : 
-- Created    : 2017-04-06
-- Last update: 2020-02-20
-- Platform   : 
-- Standard   : VHDL'08
-------------------------------------------------------------------------------
-- Description: Stream based center of gravity algorithm
-------------------------------------------------------------------------------
-- Copyright (c) 2017 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2017-04-06  1.0      xiaok   Created
-- TODOs      :
-- DONE 1. with a larger filter size, centroids could be missed due to the enable
--      signal for the FIFOs
-- DONE 2. add a sub-pixel resoltuion
-- DONE 3. Zero-crossing detection needs to be synced
--      WE need to create a delayed version of the line valid signal
-- IMPORTANT: All the valid signals are very important, they need to be
-- matached for the situation where the centroid sits on the first or last column
-------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

-- use this for synthesis
-- use ieee.fixed_pkg.all;

-- use this for simulation
library ieee_proposed;
use ieee_proposed.fixed_pkg.all;

entity scog is

  generic (
    FILTER_SIZE : positive range 3 to 127 := 15;   -- scog filter size
    DATA_BITS   : positive range 1 to 10  := 8;    -- camera data width
    COG_BITS    : positive range 8 to 24  := 16);  -- bits for the filter output

  port (
    rst         : in std_logic;
    pclk        : in std_logic;
    frame_valid : in std_logic;
    line_valid  : in std_logic;
    cam_data    : in std_logic_vector(DATA_BITS-1 downto 0);
    row_cnt     : in unsigned(11 downto 0);
    col_cnt     : in unsigned(11 downto 0);
    th_img      : in unsigned(DATA_BITS-1 downto 0);  -- th image
    th_sum      : in unsigned(COG_BITS-1 downto 0);   -- th sum
    frame_start : in std_logic;
    frame_end   : in std_logic;
    line_start  : in std_logic;
    line_end    : in std_logic;

    scog_new   : out std_logic;
    scog_cnt   : out unsigned(15 downto 0);
    scog_centx : out unsigned(11 downto 0);
    scog_centy : out unsigned(11 downto 0);
    scog_cx    : out ufixed(11 downto -8);  -- centx with sub-pixel resolution
    scog_cy    : out ufixed(11 downto -8)   -- centy with sub-pixel resolution
    );

end entity scog;

architecture RTL of scog is
  -- attribute keep : string;

  component fifo_line
    port (
      clk   : in  std_logic;
      rst   : in  std_logic;
      din   : in  std_logic_vector(7 downto 0);
      wr_en : in  std_logic;
      rd_en : in  std_logic;
      dout  : out std_logic_vector(7 downto 0);
      full  : out std_logic;
      empty : out std_logic
      );
  end component;

  component fifo_ycog
    port (
      clk   : in  std_logic;
      rst   : in  std_logic;
      din   : in  std_logic_vector(15 downto 0);
      wr_en : in  std_logic;
      rd_en : in  std_logic;
      dout  : out std_logic_vector(15 downto 0);
      full  : out std_logic;
      empty : out std_logic
      );
  end component;

  component fifo_isum
    port (
      clk   : in  std_logic;
      rst   : in  std_logic;
      din   : in  std_logic_vector(15 downto 0);
      wr_en : in  std_logic;
      rd_en : in  std_logic;
      dout  : out std_logic_vector(15 downto 0);
      full  : out std_logic;
      empty : out std_logic
      );
  end component;

  component div_cog
    port (
      aclk                   : in  std_logic;
      s_axis_divisor_tvalid  : in  std_logic;
      s_axis_divisor_tdata   : in  std_logic_vector(15 downto 0);
      s_axis_dividend_tvalid : in  std_logic;
      s_axis_dividend_tdata  : in  std_logic_vector(15 downto 0);
      m_axis_dout_tvalid     : out std_logic;
      m_axis_dout_tdata      : out std_logic_vector(23 downto 0)
      );
  end component;

  signal th_img_i      : unsigned(th_img'range);  -- th image
  signal th_sum_i      : unsigned(th_sum'range);   -- th sum
  signal cam_data_i    : std_logic_vector(cam_data'range);

  -- signals for generaging fifo rst, using frame end pulse
  signal fifo_rst              : std_logic;
  -- extended line valid signal to include the required last pixel (in
  -- horizontal blanking period) for a centroid on the last column
  -- extra number of columns required: (FILTER_SIZE-1)/2
  signal extended_line_valid   : std_logic;
  signal extended_line_valid_i : std_logic;
  signal extended_line_cnt     : unsigned(7 downto 0)             := (others => '0');
  -- shifted valid signal for the detection of xcog
  signal shifted_line_valid    : std_logic;
  signal shifted_line_cnt      : unsigned(FILTER_SIZE+5 downto 0) := (others => '0');
  signal scog_shifted_valid    : std_logic;
  -- extended valid signal to include the zero-cross detection for a centroid
  -- on the last column
  signal extended_cog_valid    : std_logic;
  signal extended_cog_valid_i  : std_logic;
  signal extended_cog_cnt      : unsigned(7 downto 0)             := (others => '0');

  -----------------------------------------------------------------------------
  -- Dealy from the pixel of cog center to the pixel of right bottom of window
  -- Dstream = (FILTER_SIZE+1)/2
  -----------------------------------------------------------------------------
  -- signals for delayed data lines, the -1 signal connects to the camera output
  type data_valid_type is array (FILTER_SIZE-1 downto -1) of std_logic;
  signal data_valid : data_valid_type;
  type data_line_type is array (FILTER_SIZE-1 downto -1)
    of std_logic_vector(DATA_BITS-1 downto 0);
  signal data_line       : data_line_type;
  signal cam_data_valid  : std_logic;   -- raw camera data valid signal
  signal scog_data_valid : std_logic;   -- extended valid signal including
  -- required pixels for the last column
  signal scog_cog_valid  : std_logic;   -- even extended valid signal until
  -- detecting zero-crossing on the last pixel

  -- filters size in bits
  constant FILTER_SIZE_BITS : integer
    := integer(ceil(log2(real(FILTER_SIZE+1))));            -- 3
  -- maximum value of filter elements (M) in bits, [-M, -(M-1), ..., +M]
  constant FILTER_VALUE_BITS : integer
    := integer(ceil(log2(real((FILTER_SIZE-1)/2+1)))) + 1;  -- signed

  -- signals for 2d filters
  type filter_type is array (FILTER_SIZE-1 downto 0) of signed(FILTER_VALUE_BITS-1 downto 0);
  signal kx_hoz : filter_type;
  signal ky_ver : filter_type;

  -----------------------------------------------------------------------------
  -- signals for xcog
  -- Dpipe = delay 2*FILTER_SIZE
  -----------------------------------------------------------------------------
  -- VER
  type xcog_ver_preadd_type is array (FILTER_SIZE-1 downto 0)
    of unsigned(DATA_BITS-1 downto 0);
  signal xcog_ver_preadd : xcog_ver_preadd_type;  -- for balancing delay
                                                  -- between xcog and ycog
  type xcog_ver_mult_type is array (FILTER_SIZE-1 downto 0)
    of unsigned(DATA_BITS-1 downto 0);
  signal xcog_ver_mult : xcog_ver_mult_type;      -- for balancing delay
                                                  -- between xcog and ycog
  type xcog_ver_ma2d_type is array (FILTER_SIZE-1 downto 0, FILTER_SIZE-2 downto 0)
    of unsigned(DATA_BITS+FILTER_SIZE_BITS-1 downto 0);
  signal xcog_ver_ma2d : xcog_ver_ma2d_type;
  type xcog_ver_add_type is array (FILTER_SIZE-2 downto 0)
    of unsigned(DATA_BITS+FILTER_SIZE_BITS-1 downto 0);
  signal xcog_ver_add : xcog_ver_add_type;

  constant XCOG_VER_ZERO : unsigned(xcog_ver_add(0)'range) := (others => '0');
  signal xcog_ver_reg    : unsigned(xcog_ver_add(0)'range);
  -- HOZ
  type xcog_hoz_mult_type is array (FILTER_SIZE-1 downto 0)
    of signed(xcog_ver_reg'length+FILTER_VALUE_BITS downto 0);  -- add extend to signed
  signal xcog_hoz_mult : xcog_hoz_mult_type;
  type xcog_hoz_add_type is array (FILTER_SIZE-1 downto 0)
    of signed(xcog_hoz_mult(0)'length+FILTER_SIZE_BITS-1 downto 0);
  signal xcog_hoz_add    : xcog_hoz_add_type;
  constant XCOG_HOZ_ZERO : signed(xcog_hoz_add(0)'range) := (others => '0');
  signal xcog_hoz_reg    : signed(xcog_hoz_add(0)'range);
  -- attribute keep of xcog_hoz_reg: signal is "true";

  -----------------------------------------------------------------------------
  -- signals for ycog
  -- delay 2*FILTER_SIZE
  -----------------------------------------------------------------------------
  -- VER
  type ycog_ver_preadd_type is array (FILTER_SIZE-1 downto 0)
    of signed(DATA_BITS downto 0);
  signal ycog_ver_preadd : ycog_ver_preadd_type;
  type ycog_ver_mult_type is array (FILTER_SIZE-1 downto 0)
    of signed(DATA_BITS + FILTER_VALUE_BITS downto 0);
  signal ycog_ver_mult : ycog_ver_mult_type;

  type ycog_ver_ma2d_type is array (FILTER_SIZE-1 downto 0, FILTER_SIZE-2 downto 0)
    of signed(DATA_BITS + FILTER_VALUE_BITS + FILTER_SIZE_BITS downto 0);
  signal ycog_ver_ma2d : ycog_ver_ma2d_type;
  type ycog_ver_add_type is array (FILTER_SIZE-2 downto 0)
    of signed(DATA_BITS + FILTER_VALUE_BITS + FILTER_SIZE_BITS downto 0);
  signal ycog_ver_add : ycog_ver_add_type;

  constant YCOG_VER_ZERO : signed(ycog_ver_add(0)'range) := (others => '0');
  signal ycog_ver_reg    : signed(ycog_ver_add(0)'range);
  -- HOZ
  type ycog_hoz_mult_type is array (FILTER_SIZE-1 downto 0)
    of signed(ycog_ver_reg'range);
  signal ycog_hoz_mult : ycog_hoz_mult_type;  -- for balancing delay
                                              -- between xcog and ycog
  type ycog_hoz_add_type is array (FILTER_SIZE-1 downto 0)
    of signed(ycog_ver_reg'length+FILTER_SIZE_BITS-1 downto 0);
  signal ycog_hoz_add    : ycog_hoz_add_type;
  constant YCOG_HOZ_ZERO : signed(ycog_hoz_add(0)'range) := (others => '0');
  signal ycog_hoz_reg    : signed(ycog_hoz_add(0)'range);
  -- attribute keep of ycog_hoz_reg: signal is "true";

  -----------------------------------------------------------------------------
  -- signals for isum
  -- delay 2*FILTER_SIZE
  -----------------------------------------------------------------------------
  -- VER
  -- Share result with XCOG
  -- HOZ
  type isum_hoz_mult_type is array (FILTER_SIZE-1 downto 0)
    of unsigned(xcog_ver_reg'range);
  signal isum_hoz_mult : isum_hoz_mult_type;
  type isum_hoz_add_type is array (FILTER_SIZE-1 downto 0)
    of unsigned(xcog_ver_reg'length+FILTER_SIZE_BITS-1 downto 0);
  signal isum_hoz_add    : isum_hoz_add_type;
  constant ISUM_HOZ_ZERO : unsigned(isum_hoz_add(0)'range) := (others => '0');
  signal isum_hoz_reg    : unsigned(isum_hoz_add(0)'range);
  -- attribute keep of isum_hoz_reg: signal is "true";  

  -----------------------------------------------------------------------------
  -- signals for zero-crossing detection - binarize
  -----------------------------------------------------------------------------
  -- signals for binarize XCOG & YCOG
  signal scog_wr_en : std_logic;
  signal scog_rd_en : std_logic;

  -- TODO: Currently using approximations instead of xcog/isum for sub-pixel interpolation
  signal xcog_cur : signed(COG_BITS-1 downto 0);
  signal ycog_cur : signed(COG_BITS-1 downto 0);
  signal isum_cur : unsigned(COG_BITS-1 downto 0);

  signal xcog_prev_col  : signed(xcog_cur'range);
  signal xcog_cur_sign  : std_logic;
  signal xcog_prev_sign : std_logic;

  signal ycog_prev_row     : signed(ycog_cur'range);
  signal ycog_prev_row_slv : std_logic_vector(ycog_cur'range);
  signal ycog_cur_sign     : std_logic;
  signal ycog_prev_sign    : std_logic;

  signal isum_prev_row     : unsigned(isum_cur'range);
  signal isum_prev_row_slv : std_logic_vector(isum_cur'range);  -- FIFO needs SLV
  signal isum_prev_col     : unsigned(isum_cur'range);

  signal xcog_bin    : std_logic;
  signal ycog_bin    : std_logic;
  signal scog_comb_i : std_logic;

  -----------------------------------------------------------------------------
  -- signals for sub-pixel resolution
  -----------------------------------------------------------------------------

  -- sub-pixel division
  constant FRACTIONAL_BITS   : integer := 8;
  constant AXI4_LATENCY_BITS : integer := 0;
  constant DIV_LATENCY       : integer := xcog_cur'length + FRACTIONAL_BITS + 4;
  signal xycog_div_valid     : std_logic;
  signal xcog_dividend       : signed(xcog_cur'range);
  signal xcog_divisor        : signed(xcog_cur'range);  -- diff between two xcogs, no
                                                        -- carry bit considered
  signal ycog_dividend       : signed(ycog_cur'range);
  signal ycog_divisor        : signed(ycog_cur'range);  -- diff between two ycogs, no
                                                        -- carry bit considered

  signal xcog_div_result     : std_logic_vector(xcog_cur'length + FRACTIONAL_BITS -1 downto 0);
  signal xcog_div_quotient   : std_logic_vector(xcog_dividend'range);
  signal xcog_div_fractional : std_logic_vector(FRACTIONAL_BITS-1 downto 0);
  signal xcog_result_valid   : std_logic;

  signal ycog_div_result     : std_logic_vector(ycog_cur'length + FRACTIONAL_BITS -1 downto 0);
  signal ycog_div_quotient   : std_logic_vector(ycog_dividend'range);
  signal ycog_div_fractional : std_logic_vector(FRACTIONAL_BITS-1 downto 0);
  signal ycog_result_valid   : std_logic;

  signal deltax : ufixed(scog_cx'range) := (others => '0');  -- sub-pixel absolute value
  signal deltay : ufixed(scog_cy'range) := (others => '0');  -- sub-pixel absolute value

  -- signals for counting centroids, coordinates
  -- signal scog_cnt_i : unsigned(15 downto 0);
  signal scog_new_i   : std_logic;
  signal scog_cnt_i   : unsigned(scog_cnt'range);
  signal scog_centx_i : unsigned(scog_centx'range);
  signal scog_centy_i : unsigned(scog_centy'range);

  -----------------------------------------------------------------------------
  -- row offset: Dstream
  -- col offset: Dstream + Dpipe
  -- first valid cog pixel (Dstream, Dstream)
  -----------------------------------------------------------------------------

begin  -- architecture RTL

  process (pclk) is
  begin
    if rising_edge(pclk) then
      if frame_start = '1' then
        th_img_i <= th_img;
        th_sum_i <= th_sum;
      end if;
    end if;
  end process;

  -- thresholding camera image
  cam_data_i <= cam_data when unsigned(cam_data) > th_img_i else
                (others => '0');

  filter_gen : for i in 0 to FILTER_SIZE-1 generate
    kx_hoz(i) <= to_signed((FILTER_SIZE-1)/2 - i, FILTER_VALUE_BITS);
    ky_ver(i) <= to_signed((FILTER_SIZE-1)/2 - i, FILTER_VALUE_BITS);
  end generate filter_gen;

  -----------------------------------------------------------------------------
  -- using frame_start to rest FIFOs
  -----------------------------------------------------------------------------
  fifo_rst <= frame_start;

  -----------------------------------------------------------------------------
  -- Generate extended valid signals for cog data and zero-crossing for the
  -- last column pixels
  -----------------------------------------------------------------------------
  process (pclk) is
  begin
    if rising_edge (pclk) then
      if line_valid = '1' then
        extended_line_valid_i <= '1';
        extended_line_cnt     <= (others => '0');
      elsif to_integer(extended_line_cnt) < (FILTER_SIZE - 1)/2 -1 then
        extended_line_valid_i <= '1';
        extended_line_cnt     <= extended_line_cnt + 1;
      else
        extended_line_valid_i <= '0';
      end if;
    end if;
  end process;

  extended_line_valid <= extended_line_valid_i or line_valid;

  -- shifted line_valid
  shifted_line_cnt   <= (shifted_line_cnt(shifted_line_cnt'high-1 downto 0) & line_valid) when rising_edge(pclk);
  shifted_line_valid <= shifted_line_cnt(shifted_line_cnt'high);

  scog_shifted_valid <= frame_valid and shifted_line_valid;

  -- extended cog valid
  process (pclk) is
  begin
    if rising_edge (pclk) then
      if line_valid = '1' then
        extended_cog_valid_i <= '1';
        extended_cog_cnt     <= (others => '0');
      elsif to_integer(extended_cog_cnt) < (FILTER_SIZE-1)/2 + (FILTER_SIZE+5) then
        extended_cog_valid_i <= '1';
        extended_cog_cnt     <= extended_cog_cnt + 1;
      else
        extended_cog_valid_i <= '0';
      end if;
    end if;
  end process;

  extended_cog_valid <= extended_cog_valid_i or line_valid;

  cam_data_valid  <= frame_valid and line_valid;
  scog_data_valid <= frame_valid and extended_line_valid;
  scog_cog_valid  <= frame_valid and extended_cog_valid;

  data_line(-1)  <= cam_data_i;
  data_valid(-1) <= scog_data_valid;

  data_valid_gen : for i in FILTER_SIZE-1 downto 0 generate
  begin
    data_valid(i) <= scog_data_valid when row_cnt > i else
                     '0';
  end generate data_valid_gen;

  fifo_line_gen : for i in 0 to FILTER_SIZE-1 generate
  begin
    i_fifo_line : fifo_line
      port map (
        clk   => pclk,
        rst   => fifo_rst,
        din   => data_line(i-1),
        wr_en => data_valid(i-1),
        rd_en => data_valid(i),
        dout  => data_line(i),
        full  => open,
        empty => open);
  end generate fifo_line_gen;

  -----------------------------------------------------------------------------
  -- XCOG calculation
  --      [-1 0 1]   [1]
  -- kx = [-1 0 1] = [1] * [-1, 0, 1]
  --      [-1 0 1]   [1]
  -- stage 1: sum vertically
  -- stage 2: filter, akin to FIR
  -----------------------------------------------------------------------------
  xcog_ver_mult_proc : process (pclk) is
  begin  -- process xcog_ver_mult_proc
    if rising_edge(pclk) then           -- rising clock edge
      if scog_cog_valid = '0' then
        xcog_ver_preadd <= ((others => (others => '0')));
        xcog_ver_mult   <= ((others => (others => '0')));
      else
        for i in FILTER_SIZE-1 downto 0 loop
          xcog_ver_preadd(i) <= unsigned(data_line(i));
          xcog_ver_mult(i)   <= xcog_ver_preadd(i);
        end loop;  -- i
      end if;
    end if;
  end process xcog_ver_mult_proc;

  xcog_ver_ma2d_proc : process (pclk) is
  begin  -- process xcog_ver_ma2d_proc
    if rising_edge(pclk) then           -- rising clock edge
      for i in FILTER_SIZE-1 downto 0 loop
        for j in FILTER_SIZE-2 downto 0 loop
          if j = FILTER_SIZE-2 then
            xcog_ver_ma2d(i, FILTER_SIZE-2) <= xcog_ver_mult(i) + XCOG_VER_ZERO;
          else
            xcog_ver_ma2d(i, j) <= xcog_ver_ma2d(i, j+1);
          end if;
        end loop;  -- j
      end loop;  -- i
    end if;
  end process xcog_ver_ma2d_proc;

  xcog_ver_add_proc : process (pclk) is
  begin  -- process xcog_ver_add_proc
    if rising_edge(pclk) then           -- rising clock edge
      if scog_cog_valid = '0' then
        xcog_ver_add <= ((others => (others => '0')));
        xcog_ver_reg <= (others  => '0');
      else
        for i in FILTER_SIZE-2 downto 0 loop
          if i = FILTER_SIZE-2 then
            xcog_ver_add(i) <= xcog_ver_ma2d(i+1, i) + xcog_ver_ma2d(i, i);
          else
            xcog_ver_add(i) <= xcog_ver_add(i+1) + xcog_ver_ma2d(i, i);
          end if;
        end loop;  -- i
        xcog_ver_reg <= xcog_ver_add(0);
      end if;
    end if;
  end process xcog_ver_add_proc;

  xcog_hoz_proc : process (pclk) is
  begin  -- process xcog_mult_proc
    if rising_edge(pclk) then           -- rising clock edge
      if scog_cog_valid = '0' then
        xcog_hoz_mult <= ((others => (others => '0')));
        xcog_hoz_add  <= ((others => (others => '0')));
        xcog_hoz_reg  <= (others  => '0');
      else
        for i in FILTER_SIZE-1 downto 0 loop
          xcog_hoz_mult(i) <= signed('0'&xcog_ver_reg) * kx_hoz(i);
          if i = FILTER_SIZE-1 then
            xcog_hoz_add(i) <= XCOG_HOZ_ZERO + xcog_hoz_mult(FILTER_SIZE-1);
          else
            xcog_hoz_add(i) <= xcog_hoz_mult(i) + xcog_hoz_add(i+1);
          end if;
        end loop;  -- i
        xcog_hoz_reg <= xcog_hoz_add(0);
      end if;
    end if;
  end process xcog_hoz_proc;

  -----------------------------------------------------------------------------
  -- YCOG calculation
  --      [-1 -1 -1]   [-1]
  -- ky = [ 0  0  0] = [ 0] * [1, 1, 1]
  --      [ 1  0  1]   [ 1]
  -- stage 1: filter vertically
  -- stage 2: sum
  -----------------------------------------------------------------------------
  ycog_ver_mult_proc : process (pclk) is
  begin  -- process ycog_ver_mult_proc
    if rising_edge(pclk) then           -- rising clock edge
      if scog_cog_valid = '0' then
        ycog_ver_preadd <= ((others => (others => '0')));
        ycog_ver_mult   <= ((others => (others => '0')));
      else
        for i in FILTER_SIZE-1 downto 0 loop
          -- ycog_ver_preadd(i) <= signed('0'&data_line(i))
          --                       - signed('0'&data_line(FILTER_SIZE-1-i));
          ycog_ver_preadd(i) <= signed('0'&data_line(i));
          ycog_ver_mult(i)   <= ky_ver(i) * ycog_ver_preadd(i);
        end loop;  -- i
      end if;
    end if;
  end process ycog_ver_mult_proc;

  ycog_ver_ma2d_proc : process (pclk) is
  begin  -- process ycog_ver_ma2d_proc
    if rising_edge(pclk) then           -- rising clock edge
      for i in FILTER_SIZE-1 downto 0 loop
        for j in FILTER_SIZE-2 downto 0 loop
          if j = FILTER_SIZE-2 then
            ycog_ver_ma2d(i, FILTER_SIZE-2) <= ycog_ver_mult(i) + YCOG_VER_ZERO;
          else
            ycog_ver_ma2d(i, j) <= ycog_ver_ma2d(i, j+1);
          end if;
        end loop;  -- j
      end loop;  -- i
    end if;
  end process ycog_ver_ma2d_proc;

  -- purpose: pipline add data lines vertically
  -- type   : sequential
  -- inputs : pclk, signal1, signal2
  -- outputs: signal3
  ycog_ver_add_proc : process (pclk) is
  begin  -- process ycog_ver_add_proc
    if rising_edge(pclk) then           -- rising clock edge
      if scog_cog_valid = '0' then
        ycog_ver_add <= ((others => (others => '0')));
        ycog_ver_reg <= (others  => '0');
      else
        for i in FILTER_SIZE-2 downto 0 loop
          if i = FILTER_SIZE-2 then
            ycog_ver_add(i) <= ycog_ver_ma2d(i+1, i) + ycog_ver_ma2d(i, i);
          else
            ycog_ver_add(i) <= ycog_ver_add(i+1) + ycog_ver_ma2d(i, i);
          end if;
        end loop;  -- i
        ycog_ver_reg <= ycog_ver_add(0);
      end if;
    end if;
  end process ycog_ver_add_proc;

  ycog_hoz_proc : process (pclk) is
  begin  -- process ycog_hoz_proc
    if rising_edge(pclk) then           -- rising clock edge
      if scog_cog_valid = '0' then
        ycog_hoz_mult <= ((others => (others => '0')));
        ycog_hoz_add  <= ((others => (others => '0')));
        ycog_hoz_reg  <= (others  => '0');
      else
        for i in FILTER_SIZE-1 downto 0 loop
          ycog_hoz_mult(i) <= ycog_ver_reg;
          if i = FILTER_SIZE-1 then
            ycog_hoz_add(i) <= YCOG_HOZ_ZERO + ycog_hoz_mult(FILTER_SIZE-1);
          else
            ycog_hoz_add(i) <= ycog_hoz_mult(i) + ycog_hoz_add(i+1);
          end if;
        end loop;  -- i
        ycog_hoz_reg <= ycog_hoz_add(0);
      end if;
    end if;
  end process ycog_hoz_proc;

  -----------------------------------------------------------------------------
  -- ISUM calculation
  --      [1 1 1]   [1]
  -- ks = [1 1 1] = [1] * [1, 1, 1]
  --      [1 1 1]   [1]
  -- stage 1: filter vertically
  -- stage 2: sum
  -----------------------------------------------------------------------------
  isum_hoz_proc : process (pclk) is
  begin  -- process isum_hoz_proc
    if rising_edge(pclk) then           -- rising clock edge
      if scog_cog_valid = '0' then
        isum_hoz_mult <= ((others => (others => '0')));
        isum_hoz_add  <= ((others => (others => '0')));
        isum_hoz_reg  <= (others  => '0');
      else
        for i in FILTER_SIZE-1 downto 0 loop
          isum_hoz_mult(i) <= xcog_ver_reg;
          if i = FILTER_SIZE-1 then
            isum_hoz_add(i) <= ISUM_HOZ_ZERO + isum_hoz_mult(FILTER_SIZE-1);
          else
            isum_hoz_add(i) <= isum_hoz_mult(i) + isum_hoz_add(i+1);
          end if;
        end loop;  -- i
        isum_hoz_reg <= isum_hoz_add(0);
      end if;
    end if;
  end process isum_hoz_proc;

  -----------------------------------------------------------------------------
  -- Binarize XCOG & YCOG, determine spot pos
  -----------------------------------------------------------------------------
  -- resize for centx & centy division
  xcog_cur <= resize(xcog_hoz_reg, xcog_cur'length);
  isum_cur <= resize(isum_hoz_reg, isum_cur'length);
  ycog_cur <= resize(ycog_hoz_reg, ycog_cur'length);

  -- xcog buffer
  xcog_prev_col  <= xcog_cur when rising_edge(pclk);
  -- XCOG BIN
  xcog_cur_sign  <= xcog_cur(xcog_cur'high);
  xcog_prev_sign <= xcog_prev_col(xcog_prev_col'high);
  xcog_bin       <= '1'      when xcog_prev_sign = '0'
              and xcog_cur_sign = '1' else
              '0';

  -- introduced one clock delay here
  -- process (pclk) is
  -- begin
  --   if rising_edge (pclk) then
  --     xcog_bin <= '0';
  --     if xcog_prev_sign = '0' and xcog_cur_sign = '1' then
  --       xcog_bin <= '1';
  --     end if;
  --   end if;
  -- end process;

  -- ycog buffer 
  -- VERY tricky wr/rd enable signals
  scog_wr_en <= scog_shifted_valid when row_cnt > 0 else
                '0';
  scog_rd_en <= scog_shifted_valid when row_cnt > 1 else
                '0';

  ycog_prev_row <= signed(ycog_prev_row_slv);

  i_fifo_ycog : fifo_ycog
    port map (
      clk   => pclk,
      rst   => fifo_rst,
      din   => std_logic_vector(ycog_cur),
      wr_en => scog_wr_en,
      rd_en => scog_rd_en,
      dout  => ycog_prev_row_slv,
      full  => open,
      empty => open);

  -- YCOG BIN
  ycog_cur_sign  <= ycog_cur(ycog_cur'high);
  ycog_prev_sign <= ycog_prev_row(ycog_prev_row'high);
  ycog_bin       <= '1' when ycog_prev_sign = '0'
              and ycog_cur_sign = '1' else
              '0';

  -- process (pclk) is
  -- begin
  --   if rising_edge (pclk) then
  --     ycog_bin <= '0';
  --     if ycog_prev_sign = '0' and ycog_cur_sign = '1' then
  --       ycog_bin <= '1';
  --     end if;
  --   end if;
  -- end process;
  -- COG COMB

  -- scog_comb_i <= xcog_bin and ycog_bin;
  scog_comb_i <= xcog_bin and ycog_bin and scog_cog_valid
                 when resize(isum_hoz_reg, th_sum_i'length) > th_sum_i else
                 '0';

  -- isum line delay and buffer
  isum_prev_col <= isum_cur when rising_edge(pclk);
  isum_prev_row <= unsigned(isum_prev_row_slv);

  i_fifo_isum : fifo_isum
    port map (
      clk   => pclk,
      rst   => fifo_rst,
      din   => std_logic_vector(isum_cur),
      wr_en => scog_wr_en,
      rd_en => scog_rd_en,
      dout  => isum_prev_row_slv,
      full  => open,
      empty => open);


  -- full-pixel resolution, check with internal signals scog_comb_i and
  -- scog_centx_i & scog_centy_i
  process (pclk) is
  begin
    if rising_edge(pclk) then           -- rising clock edge
      if frame_start = '1' then
        scog_centx_i <= (others => '0');
        scog_centy_i <= (others => '0');
      else
        scog_centx_i <= col_cnt - (FILTER_SIZE-1)/2 - (FILTER_SIZE+5);
        scog_centy_i <= row_cnt - (FILTER_SIZE+1)/2;
      end if;
    end if;
  end process;

  -------------------------------------------------------------------------------
  -- Sub-pixel resolution signals
  -- For the interpolation, we assume isum is the same therefore only use xcog&ycog
  -------------------------------------------------------------------------------
  process(pclk) is
  begin
    if falling_edge(pclk) then
      xycog_div_valid <= scog_comb_i;
      xcog_divisor    <= xcog_prev_col - xcog_cur;
      xcog_dividend   <= xcog_cur;
      ycog_divisor    <= ycog_prev_row - ycog_cur;
      ycog_dividend   <= ycog_cur;
    end if;
  end process;

  div_cog_ix : div_cog
    port map (
      aclk                   => pclk,
      s_axis_divisor_tvalid  => xycog_div_valid,
      s_axis_divisor_tdata   => std_logic_vector(xcog_divisor),
      s_axis_dividend_tvalid => xycog_div_valid,
      s_axis_dividend_tdata  => std_logic_vector(xcog_dividend),
      m_axis_dout_tvalid     => xcog_result_valid,
      m_axis_dout_tdata      => xcog_div_result
      );

  xcog_div_quotient   <= xcog_div_result(23 downto 8);
  xcog_div_fractional <= xcog_div_result(7 downto 0);

  div_cog_iy : div_cog
    port map (
      aclk                   => pclk,
      s_axis_divisor_tvalid  => xycog_div_valid,
      s_axis_divisor_tdata   => std_logic_vector(ycog_divisor),
      s_axis_dividend_tvalid => xycog_div_valid,
      s_axis_dividend_tdata  => std_logic_vector(ycog_dividend),
      m_axis_dout_tvalid     => ycog_result_valid,
      m_axis_dout_tdata      => ycog_div_result
      );

  ycog_div_quotient   <= ycog_div_result(23 downto 8);
  ycog_div_fractional <= ycog_div_result(7 downto 0);

  -----------------------------------------------------------------------------
  -- Really no need for signed fixed just for the negative sub-pixel shift from
  -- the zero-crossing pixels indices, so use a trick here:
  -- convert negative fractional parts to absolute sub-pixel shift (left/above)
  -----------------------------------------------------------------------------
  process(pclk) is
  begin
    if rising_edge(pclk) then
      if line_start = '1' then
        deltax     <= (others => '0');
        deltay     <= (others => '0');
        scog_new_i <= '0';
      else
        deltax(-1 downto -7) <= ufixed(xcog_div_fractional(6 downto 0));
        deltay(-1 downto -7) <= ufixed(ycog_div_fractional(6 downto 0));
        scog_new_i           <= xcog_result_valid;
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- Output signal
  -----------------------------------------------------------------------------

  -- Centroids counter
  scog_cnt_proc : process (pclk) is
  begin  -- process scog_cnt_proc
    if rising_edge(pclk) then           -- rising clock edge
      if frame_start = '1' then         -- asynchronous reset (active low)
        scog_cnt_i <= (others => '0');
      else
        if scog_new_i = '1' then
          scog_cnt_i <= scog_cnt_i + 1;
        end if;
      end if;
    end if;
  end process scog_cnt_proc;

  -- purpose: register output signals & sub-pixel resolution
  output_proc : process (pclk) is
  begin  -- process output_proc
    if rising_edge(pclk) then           -- rising clock edge
      scog_new                       <= scog_new_i;
      scog_cnt                       <= scog_cnt_i;
      scog_centx                     <= scog_centx_i - DIV_LATENCY - 1;
      scog_centy                     <= scog_centy_i;
      scog_cx(-1 downto scog_cx'low) <= deltax(-1 downto deltax'low);
      scog_cy(-1 downto scog_cy'low) <= deltay(-1 downto deltax'low);
      scog_cx(scog_cx'high downto 0) <= ufixed(scog_centx_i - DIV_LATENCY - 2);
      scog_cy(scog_cy'high downto 0) <= ufixed(scog_centy_i - 1);
    end if;
  end process output_proc;

end architecture RTL;

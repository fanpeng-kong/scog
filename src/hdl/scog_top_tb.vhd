-------------------------------------------------------------------------------
-- Title      : Testbench for Stream-Based CoG module
-- Project    : Shack-Hartmann Wavefront Sensor
-------------------------------------------------------------------------------
-- File       : scog_top_tb.vhd<hdl>
-- Author     : Fanpeng Kong  <fanpeng@fanpengkong.com>
-- Company    : 
-- Created    : 2019-11-11
-- Last update: 2020-06-05
-- Platform   : 
-- Standard   : VHDL'08
-------------------------------------------------------------------------------
-- Description: 
-------------------------------------------------------------------------------
-- Copyright (c) 2019 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2019-11-11  1.0      fanpeng Created
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

-- use this for synthesis
-- use ieee.fixed_pkg.all;

-- use this for simulation
  library ieee_proposed;
use ieee_proposed.fixed_pkg.all;

entity scog_top_tb is

end entity scog_top_tb;

-------------------------------------------------------------------------------

architecture Sim of scog_top_tb is

  -- component generics
  constant COLUMN_SIZE         : positive := 90;
  constant ROW_SIZE            : positive := 90;
  constant HORIZONTAL_BLANKING : positive := 25;
  constant VERTICAL_BLANKING   : positive := 21;
  constant DATA_BITS           : positive := 8;

  constant FILTER_SIZE : positive := 5;   -- filter length
  constant COG_BITS    : positive := 16;  -- don't change for now

  -- component ports
  signal cam_rst         : std_logic                      := '0';
  signal cam_pclk        : std_logic                      := '0';
  signal cam_frame_valid : std_logic;
  signal cam_line_valid  : std_logic;
  signal cam_data        : std_logic_vector(DATA_BITS-1 downto 0);
  signal th_img          : unsigned(DATA_BITS-1 downto 0) := (others => '0');
  signal th_sum          : unsigned(COG_BITS-1 downto 0)  := (others => '0');

  -----------------------------------------------------------------------------
  -- component ports
  -----------------------------------------------------------------------------
  signal frame_start : std_logic;
  signal frame_end   : std_logic;
  signal line_start  : std_logic;
  signal line_end    : std_logic;
  signal row_cnt     : unsigned(11 downto 0);
  signal col_cnt     : unsigned(11 downto 0);

  signal scog_new   : std_logic;
  signal scog_cnt   : unsigned(15 downto 0);
  signal scog_centx : unsigned(11 downto 0);
  signal scog_centy : unsigned(11 downto 0);
  signal scog_cx    : ufixed(11 downto -8);
  signal scog_cy    : ufixed(11 downto -8);


  constant FCLK : positive := 50E6;          -- frequency: 50 MHz
  constant TCLK : time     := 1 sec / FCLK;  -- period
  signal done   : boolean  := false;

  -- constant FILE_NAME : string    := "../other/dat/centroids_results_filter_5.dat";
  constant FILE_NAME : string    := "/home/fanpeng/Projects/FPGA/ao/scog/src/other/dat/centroids_results_filter_5.csv";

  file fptr          : text;
  signal eof         : std_logic := '0';

begin  -- architecture Sim

  -- clock generation
  cam_pclk <= '0' when done else not cam_pclk after (TCLK / 2);

  -- reset generation
  cam_rst <= '1', '0' after TCLK;

  -- component instantiation
  camera_gen_i : entity work.camera_gen
    generic map (
      COLUMN_SIZE         => COLUMN_SIZE,
      ROW_SIZE            => ROW_SIZE,
      HORIZONTAL_BLANKING => HORIZONTAL_BLANKING,
      VERTICAL_BLANKING   => VERTICAL_BLANKING,
      DATA_BITS           => DATA_BITS)
    port map (
      pclk        => cam_pclk,
      rst         => cam_rst,
      frame_valid => cam_frame_valid,
      line_valid  => cam_line_valid,
      cam_data    => cam_data);

  scog_top_i : entity work.scog_top
    generic map (
      FILTER_SIZE => FILTER_SIZE,
      DATA_BITS   => DATA_BITS,
      COG_BITS    => COG_BITS)
    port map (
      cam_pclk        => cam_pclk,
      cam_rst         => cam_rst,
      cam_frame_valid => cam_frame_valid,
      cam_line_valid  => cam_line_valid,
      cam_data        => cam_data,
      th_img          => th_img,
      th_sum          => th_sum,

      frame_start => frame_start,
      frame_end   => frame_end,
      line_start  => line_start,
      line_end    => line_end,
      row_cnt     => row_cnt,
      col_cnt     => col_cnt,

      scog_new   => scog_new,
      scog_cnt   => scog_cnt,
      scog_centx => scog_centx,
      scog_centy => scog_centy,
      scog_cx    => scog_cx,
      scog_cy    => scog_cy);

  write_data_proc : process is

    variable fstatus   : file_open_status;
    variable file_line : line;
    variable var_cnt   : integer;
    variable var_cx    : ufixed(scog_cx'range);
    variable var_cy    : ufixed(scog_cy'range);

  begin  -- process write_data_proc

    var_cnt := 0;
    var_cx  := to_ufixed(0, var_cx'high, var_cx'low);
    var_cy  := to_ufixed(0, var_cy'high, var_cy'low);
    
    eof <= '0';

    wait until cam_rst = '0';

    file_open(fstatus, fptr, FILE_NAME, write_mode);

    wait until cam_frame_valid = '1';

    -- Write header
    write(file_line, string'("cnt, "), left);
    write(file_line, string'("cx, "), left);
    write(file_line, string'("cy, "), left);
    writeline(fptr, file_line);

    while (cam_frame_valid = '1') loop
      wait until scog_new = '1';
      var_cnt := to_integer(scog_cnt) + 1;
      var_cx := scog_cx;
      var_cy := scog_cy;

      write(file_line, var_cnt, left);
      write(file_line, string'(", "), left);
      write(file_line, to_real(var_cx), left, 8, 4);
      write(file_line, string'(", "), left);
      write(file_line, to_real(var_cy), left, 8, 4);

      writeline(fptr, file_line);
    end loop;

    wait until cam_frame_valid = '0';
    eof <= '1';
    file_close(fptr);
    wait;

  end process write_data_proc;

end architecture Sim;

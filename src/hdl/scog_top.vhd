-------------------------------------------------------------------------------
-- Title      : SCoG top module
-- Project    : Stream-Based Center-of-Gravity
-------------------------------------------------------------------------------
-- File       : scog_top.vhd
-- Author     : Fanpeng Kong  <fanpeng@fanpengkong.com>
-- Company    : 
-- Created    : 2020-02-04
-- Last update: 2020-03-25
-- Platform   : 
-- Standard   : VHDL'08
-------------------------------------------------------------------------------
-- Description: This top module takes raw camera output, perform necessary
-- sync/alignment steps and then use scog.vhd to detect potential centroids.
-------------------------------------------------------------------------------
-- Copyright (c) 2020 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2020-02-04  1.0      fanpeng Created
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

entity scog_top is

  generic (
    DATA_BITS   : positive range 1 to 10  := 8;
    FILTER_SIZE : positive range 3 to 127 := 15;   -- scog filter size
    COG_BITS    : positive range 8 to 24  := 16);  -- camera data width

  port (
    cam_rst         : in std_logic;
    cam_pclk        : in std_logic;
    cam_frame_valid : in std_logic;
    cam_line_valid  : in std_logic;
    cam_data        : in std_logic_vector(DATA_BITS-1 downto 0);
    th_img          : in unsigned(DATA_BITS-1 downto 0);
    th_sum          : in unsigned(COG_BITS-1 downto 0);

    frame_valid_sync : out std_logic;
    line_valid_sync  : out std_logic;
    frame_start      : out std_logic;
    frame_end        : out std_logic;
    line_start       : out std_logic;
    line_end         : out std_logic;
    row_cnt          : out unsigned(11 downto 0);
    col_cnt          : out unsigned(11 downto 0);

    scog_new   : out std_logic;
    scog_cnt   : out unsigned(15 downto 0);
    scog_centx : out unsigned(11 downto 0);
    scog_centy : out unsigned(11 downto 0);
    scog_cx    : out ufixed(11 downto -8);  -- centx with sub-pixel resolution
    scog_cy    : out ufixed(11 downto -8)   -- centy with sub-pixel resolution
    );

end entity scog_top;

architecture Schematic of scog_top is

  -----------------------------------------------------------------------------
  -- align row&col count with frame and line
  -----------------------------------------------------------------------------
  -- for internal/external use
  signal frame_start_i : std_logic;
  signal frame_end_i   : std_logic;
  signal line_start_i  : std_logic;
  signal line_end_i    : std_logic;
  signal row_cnt_i     : unsigned(11 downto 0);
  signal col_cnt_i     : unsigned(11 downto 0);

  -- for internal use
  signal frame_valid_sync_i : std_logic;
  signal line_valid_sync_i  : std_logic;
  signal data_sync_i        : std_logic_vector(DATA_BITS-1 downto 0);

begin  -- architecture Schematic

  -----------------------------------------------------------------------------
  -- sync control signal to falling edge of frame_valid
  -----------------------------------------------------------------------------
  i_camera_sync : entity work.camera_sync
    generic map (
      DATA_BITS => DATA_BITS)

    port map (
      pclk            => cam_pclk,
      cam_frame_valid => cam_frame_valid,
      cam_line_valid  => cam_line_valid,
      cam_data        => cam_data,

      frame_start => frame_start_i,
      frame_end   => frame_end_i,
      line_start  => line_start_i,
      line_end    => line_end_i,

      cam_frame_valid_sync => frame_valid_sync_i,
      cam_line_valid_sync  => line_valid_sync_i,
      cam_data_sync        => data_sync_i,
      row_cnt              => row_cnt_i,
      col_cnt              => col_cnt_i
      );

  i_scog : entity work.scog
    generic map (
      FILTER_SIZE => FILTER_SIZE,
      DATA_BITS   => DATA_BITS,
      COG_BITS    => COG_BITS)
    port map (
      rst         => cam_rst,
      pclk        => cam_pclk,
      frame_valid => frame_valid_sync_i,
      line_valid  => line_valid_sync_i,
      cam_data    => data_sync_i,
      row_cnt     => row_cnt_i,
      col_cnt     => col_cnt_i,
      th_img      => th_img,
      th_sum      => th_sum,
      frame_start => frame_start_i,
      frame_end   => frame_end_i,
      line_start  => line_start_i,
      line_end    => line_end_i,

      scog_new   => scog_new,
      scog_cnt   => scog_cnt,
      scog_centx => scog_centx,
      scog_centy => scog_centy,
      scog_cx    => scog_cx,
      scog_cy    => scog_cy
      );

  frame_valid_sync <= frame_valid_sync_i;
  line_valid_sync  <= line_valid_sync_i;
  frame_start      <= frame_start_i;
  frame_end        <= frame_end_i;
  line_start       <= line_start_i;
  line_end         <= line_end_i;
  row_cnt          <= row_cnt_i;
  col_cnt          <= col_cnt_i;

end architecture Schematic;

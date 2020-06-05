-------------------------------------------------------------------------------
-- Title      : Camera signals syncing module for SCoG
-- Project    : Stream-Based Center-of-Gravity
-------------------------------------------------------------------------------
-- File       : camera_sync.vhd
-- Author     : Fanpeng Kong  <fanpeng@fanpengkong.com>
-- Company    : 
-- Created    : 2020-01-31
-- Last update: 2020-02-20
-- Platform   : 
-- Standard   : VHDL'08
-------------------------------------------------------------------------------
-- Description: This module manipulate the raw camera signals to realize a row
--              and column count starting from 0. Also, the column count
--              continues during the horizontal blanking time as its often that
--              the SCoG algorithm extends its operation to this period (e.g.
--              larger filter size or spot close to the right edge etc.)
-- Outputs    : the frame_start and line_start signals starts one clock before
--              the rising edges of the synced frame_start and line_start signals.
--              while the frame_end and line_end starts one clock before the
--              falling edge of the synced frame_valid and line_valid signals.
-------------------------------------------------------------------------------
-- Copyright (c) 2020 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2020-01-31  1.0      fanpeng Created
-------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity camera_sync is
  generic (
    DATA_BITS : positive range 1 to 10 := 8);  -- camera data width

  port (
    pclk            : in std_logic;
    cam_frame_valid : in std_logic; -- raw camera frame valid signal
    cam_line_valid  : in std_logic; -- raw camera line valid signal
    cam_data        : in std_logic_vector(DATA_BITS-1 downto 0); -- raw camera
                                                                 -- data signal

    frame_start : out std_logic;
    frame_end   : out std_logic;
    line_start  : out std_logic;
    line_end    : out std_logic;

    cam_frame_valid_sync : out std_logic;
    cam_line_valid_sync  : out std_logic;
    cam_data_sync        : out std_logic_vector(DATA_BITS-1 downto 0);
    row_cnt              : out unsigned(11 downto 0);
    col_cnt              : out unsigned(11 downto 0)
    );

end entity camera_sync;

architecture RTL of camera_sync is

  signal frame_valid_reg : std_logic;
  signal frame_start_i   : std_logic;
  signal frame_end_i     : std_logic;

  signal line_valid_reg : std_logic;
  signal line_start_i   : std_logic;
  signal line_end_i     : std_logic;

  signal row_cnt_i : unsigned(11 downto 0);
  signal col_cnt_i : unsigned(11 downto 0);

begin  -- architecture RTL

  -----------------------------------------------------------------------------
  -- Edge detection
  -----------------------------------------------------------------------------
  frame_valid_reg <= cam_frame_valid when rising_edge(pclk);
  frame_start_i   <= (not frame_valid_reg) and cam_frame_valid;
  frame_end_i     <= frame_valid_reg and (not cam_frame_valid);

  line_valid_reg <= cam_line_valid when rising_edge(pclk);
  line_start_i   <= (not line_valid_reg) and cam_line_valid;
  line_end_i     <= line_valid_reg and (not cam_line_valid);

  colum_cnt_proc : process (pclk, line_start_i, cam_frame_valid) is
  begin  -- process colum_cnt_proc
    if rising_edge(pclk) then
      if line_start_i = '1' or cam_frame_valid = '0' then        -- synchronous reset (active low)
        col_cnt_i <= (others => '0');
      else
        col_cnt_i <= col_cnt_i + 1;
      end if;
    end if;
  end process colum_cnt_proc;

  row_cnt_proc : process (pclk, cam_frame_valid) is
  begin  -- process row_cnt_proc
    if rising_edge(pclk) then           -- rising clock edge
      if cam_frame_valid = '0' then         -- synchronous reset (active low)
        row_cnt_i <= (others => '0');
      else
        if line_start_i = '1' then
          row_cnt_i <= row_cnt_i + 1;
        end if;
      end if;
    end if;
  end process row_cnt_proc;

  -- output
  row_cnt     <= row_cnt_i - 1;
  col_cnt     <= col_cnt_i;

  frame_start <= frame_start_i;
  frame_end   <= frame_end_i;
  line_start  <= line_start_i;
  line_end    <= line_end_i;

  delay_proc : process (pclk) is
  begin
    if rising_edge(pclk) then
      cam_line_valid_sync <= cam_line_valid;
      cam_frame_valid_sync <= cam_frame_valid;
      cam_data_sync <= cam_data;
    end if;
  end process delay_proc;

end architecture RTL;

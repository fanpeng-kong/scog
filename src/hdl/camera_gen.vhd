-------------------------------------------------------------------------------
-- Title      : Generate artificial camera output
-- Project    : Image Processing
-------------------------------------------------------------------------------
-- File       : camera_gen.vhd
-- Author     : Fanpeng Kong  <fanpeng@fanpengkong.com>
-- Company    : 
-- Created    : 2020-01-30
-- Last update: 2020-03-25
-- Platform   : 
-- Standard   : VHDL'08
-------------------------------------------------------------------------------
-- Description: Generate artificial camera output including frame and line
--              valid signals, pixel clock and values. The image frame is
--              stored inside a ROM.
-------------------------------------------------------------------------------
-- Copyright (c) 2020 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2020-01-30  1.0      fanpeng	Created
-------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity camera_gen is

  generic (
    COLUMN_SIZE         : positive := 30;
    ROW_SIZE            : positive := 30;
    HORIZONTAL_BLANKING : positive := 25;
    VERTICAL_BLANKING   : positive := 21;
    DATA_BITS          : positive := 8);

  port (
    pclk        : in  std_logic;
    rst         : in  std_logic;
    frame_valid : out std_logic;
    line_valid  : out std_logic;
    cam_data    : out std_logic_vector(DATA_BITS-1 downto 0)
    );

end entity camera_gen;

architecture RTL of camera_gen is

  constant ADDR_WIDTH : integer
    := integer(ceil(log2(real(COLUMN_SIZE*ROW_SIZE))));

  -- TODO: is it possible to instantiate a ROM directly based on
  -- DATA_BITS and ADDR_WIDTH?
  component camera_rom
    port (
      clka  : in  std_logic;
      addra : in  std_logic_vector(ADDR_WIDTH-1 downto 0);
      douta : out std_logic_vector(DATA_BITS-1 downto 0)
      );
  end component;

  type state_type is (st_idle, st_start, st_p1, st_repeat);
  signal state : state_type;

  signal col_clks_cnt    : unsigned(11 downto 0);
  signal row_clks_cnt    : unsigned(20 downto 0);
  signal frame_valid_i   : std_logic;
  signal frame_valid_reg : std_logic;
  signal frame_start     : std_logic;
  -- IMPORTANT: usage of both line_valid(_i) and and it's one clock delay
  -- line_valid_reg is because of the one clock delay at the ROM!!!
  signal line_valid_i    : std_logic;
  signal line_valid_reg  : std_logic;

  signal cam_data_valid : std_logic;
  signal addr_cnt       : unsigned(ADDR_WIDTH-1 downto 0);
  signal rom_data       : std_logic_vector(DATA_BITS-1 downto 0);

  constant P1                 : positive := 242;
  -- constant P1                 : positive := 24;
  constant P2                 : positive := 2 + HORIZONTAL_BLANKING - 19;
  constant Q                  : positive := P1 + P2;
  constant ROW_CLOCKS         : positive := COLUMN_SIZE + Q;
  constant V                  : positive := (VERTICAL_BLANKING+1) * ROW_CLOCKS;
  constant FRAME_VALID_CLOCKS : positive := ROW_SIZE * ROW_CLOCKS;
  constant FRAME_CLOCKS       : positive := V + FRAME_VALID_CLOCKS;

begin  -- architecture RTL

  -----------------------------------------------------------------------------
  -- camera parameters
  -----------------------------------------------------------------------------


  cam_data_valid <= frame_valid_i and line_valid_reg;
  cam_data       <= rom_data when cam_data_valid = '1' else
              (others => '0');

  i_camera_rom : camera_rom
    port map (
      clka  => pclk,
      addra => std_logic_vector(addr_cnt),
      douta => rom_data);

  read_rom_proc : process (pclk, frame_valid_i) is
  begin  -- process read_rom_proc
    if frame_valid_i = '0' then         -- asynchronous reset (active high)
      addr_cnt <= (others => '0');
    elsif rising_edge(pclk) then        -- rising clock edge
      if line_valid_i = '1' then
        addr_cnt <= addr_cnt + 1;
      end if;
    end if;
  end process read_rom_proc;

  frame_valid_proc : process (pclk, rst)is
  begin  -- process frame_valid_proc
    if rst = '1' then                   -- asynchronous reset (active high)
      frame_valid_i <= '0';
      row_clks_cnt  <= (others => '0');
    elsif rising_edge(pclk) then
      if to_integer(row_clks_cnt) < FRAME_VALID_CLOCKS then
        frame_valid_i <= '1';
        row_clks_cnt  <= row_clks_cnt + 1;
      elsif to_integer(row_clks_cnt) < FRAME_CLOCKS-1 then
        frame_valid_i <= '0';
        row_clks_cnt  <= row_clks_cnt + 1;
      else
        row_clks_cnt <= (others => '0');
      end if;
    end if;
  end process frame_valid_proc;

  frame_valid_reg <= frame_valid_i when rising_edge(pclk);
  frame_start     <= (not frame_valid_reg) and frame_valid_i;

  -- THERE is one clock more in the line invalid after frame start.
  line_valid_proc : process (pclk, rst) is
  begin  -- process line_valid_proc
    if rst = '1' then                   -- asynchronous reset (active high)
      state        <= st_idle;
      line_valid_i <= '0';
      col_clks_cnt <= (others => '0');
    elsif rising_edge(pclk) then
      line_valid_i <= '0';

      case state is
        when st_idle =>
          if frame_start = '1' then
            state        <= st_p1;
            col_clks_cnt <= col_clks_cnt + 1;
          end if;

        when st_p1 =>
          if to_integer(col_clks_cnt) < P1-2 then
            state        <= st_p1;
            col_clks_cnt <= col_clks_cnt + 1;
          else
            state        <= st_repeat;
            col_clks_cnt <= (others => '0');
          end if;

        when st_repeat =>
          if frame_start = '1' then
            state        <= st_p1;
            col_clks_cnt <= to_unsigned(1, col_clks_cnt'length);
          else
            if to_integer(col_clks_cnt) < COLUMN_SIZE then
              col_clks_cnt <= col_clks_cnt + 1;
              line_valid_i <= '1';
            elsif to_integer(col_clks_cnt) < COLUMN_SIZE + Q - 1 then
              col_clks_cnt <= col_clks_cnt + 1;
              line_valid_i <= '0';
            else
              col_clks_cnt <= (others => '0');
            end if;
          end if;
        when others =>
          null;
      end case;
    end if;
  end process line_valid_proc;

  line_valid_reg <= line_valid_i when rising_edge(pclk);

  -- output
  frame_valid <= frame_valid_i;
  line_valid  <= line_valid_reg;

end architecture RTL;

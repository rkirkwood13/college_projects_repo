--------------------------------------------------------------------
--ECD322 Open Source FPGA 
--Team Members: Rachel Roof, John Dambra, Asim Adam, Yizhou Wang
--Advisor: Professor Douglas Summerville
--Senior Project 2023, Professor Meghana Jain
--Watson, Binghamton University
--
-- Turns on LED, outputs clock on PMOD3, and outputs switch on PMOD1
---------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

--------------------------------------

entity LED_on is
port(
	LED1: out std_logic;
	CLK2: in std_logic;
	PMOD1: out std_logic;
	PMOD3: out std_logic;
	SW: in std_logic
);
end LED_on;

---------------------------------------



architecture Behavioral of LED_on is


COMPONENT SB_IO_OD IS
GENERIC(
PIN_TYPE : std_logic_vector(5 downto 0) := "011000"; --non_registered, with input
IO_STANDARD: string := "SB_LVCMOS"
);
PORT(
PACKAGEPIN : out std_logic;
LATCHINPUTVALUE : in std_logic;
CLOCKENABLE : in std_logic;
INPUTCLK : in std_logic;
OUTPUTCLK : in std_logic;
OUTPUTENABLE : in std_logic;
DOUT0 : in std_logic;
DOUT1 : in std_logic;
DIN0 : out std_logic;
DIN1 : out std_logic
);
END COMPONENT SB_IO_OD;

signal led : std_logic:= '0'; --LED ON
signal pmod : std_logic;

begin

LED_COMPONENT: SB_IO_OD port map ( PACKAGEPIN => LED1,
LATCHINPUTVALUE => '0',
CLOCKENABLE => '0',
INPUTCLK => '0',
OUTPUTCLK => '0',
OUTPUTENABLE => '1',
DOUT0 => led,
DOUT1 => '0' --no ddr
);

process(CLK2) --clock process
begin
if rising_edge(CLK2) then
	pmod <= not pmod;
end if;
end process;

PMOD1 <= SW; --output switch value on pmod1
PMOD3 <= pmod; --output clock on pmod3

end Behavioral;

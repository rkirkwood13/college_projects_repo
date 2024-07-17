----------------------------------------------------------------
--ECD322 Open Source FPGA 
--Team Members: Rachel Roof, John Dambra, Asim Adam, Yizhou Wang
--Advisor: Professor Douglas Summerville
--Senior Project 2023, Professor Meghana Jain
--Watson, Binghamton University
--
-- Simple code to test the PMOD output
----------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

--------------------------------------

entity gpio is
  port (
    
    PMOD1 : out std_logic;
    PMOD2 : out std_logic;
    PMOD3 : out std_logic;
    PMOD4 : out std_logic;
    PMOD5 : out std_logic;
    PMOD6 : out std_logic;
    PMOD7 : out std_logic;
    PMOD8 : out std_logic

  );
end gpio;


architecture behavior of gpio is

begin

  PMOD1 <= '1';
  PMOD2 <= '0';
  PMOD3 <= '0';
  PMOD4 <= '0';
  PMOD5 <= '0';
  PMOD6 <= '0';
  PMOD7 <= '0';
  PMOD8 <= '0';

  
end behavior;
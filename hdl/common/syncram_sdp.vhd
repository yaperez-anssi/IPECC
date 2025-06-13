--
--  Copyright (C) 2023 - This file is part of IPECC project
--
--  Authors:
--      Karim KHALFALLAH <karim.khalfallah@ssi.gouv.fr>
--      Ryad BENADJILA <ryadbenadjila@gmail.com>
--
--  Contributors:
--      Adrian THILLARD
--      Emmanuel PROUFF
--
--  This software is licensed under GPL v2 license.
--  See LICENSE file at the root folder of the project.
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.ecc_log.all;

-- Depending on the FPGA vendor/family/device target, an extra-layer of
-- register may be present inside the Block-RAM providing such 2-cycle
-- latency, as it leads to better timing results.
-- In this case it is best for area performance to ensure that the
-- extra register layer on the read data path is held back inside
-- the Block-RAM by synthesis/back-end tools
entity syncram_sdp is
	generic(
		rdlat : positive range 1 to 2;
		datawidth : natural range 1 to integer'high;
		datadepth : natural range 1 to integer'high);
	port(
		clk : in std_logic;
		-- port A (write-only)
		addra : in std_logic_vector(log2(datadepth - 1) - 1 downto 0);
		wea : in std_logic;
		dia : in std_logic_vector(datawidth - 1 downto 0);
		-- port B (read-only)
		addrb : in std_logic_vector(log2(datadepth - 1) - 1 downto 0);
		reb : in std_logic;
		dob : out std_logic_vector(datawidth - 1 downto 0)
	);
end entity syncram_sdp;

architecture syn of syncram_sdp is

	subtype std_ram_word is std_logic_vector(datawidth - 1 downto 0);
	type mem_content_type is array(integer range 0 to datadepth - 1)
		of std_ram_word;
	signal mem_content : mem_content_type;

	-- Only used when rdlat = 2
	signal predob : std_ram_word;

begin

	process(clk) is
	begin
		if (clk'event and clk = '1') then

			-- ----------
			-- write logic (port A)
			-- ----------
			-- (in simulation, only affects array content if no METAVALUE in addra)
			-- otherwise issue a WARNING message
			if (wea = '1') then
				assert(not is_X(addra))
					report "write to syncram_sdp with a METAVALUE address"
						severity WARNING;
				mem_content(to_integer(unsigned(addra))) <= dia;
			end if;

			-- ----------
			-- read logic (port B)
			-- ----------
			if rdlat = 1 then -- statically resolved by synthesizer

				-- (in simulation returns 'force unknown' ('X') if METAVALUE in addrb)
				if (reb = '1') then
					-- pragma translate_off
					if is_X(addrb) then
						dob <= (others => 'X');
					else
					-- pragma translate_on
						dob <= mem_content(to_integer(unsigned(addrb)));
					-- pragma translate_off
					end if;
					-- pragma translate_on
				end if;

			elsif rdlat = 2 then -- statically resolved by synthesizer

				-- It is assumed that the output register layer of the SRAM block,
				-- in the reference technology, latches the data out of the memory
				-- array independently of the Read-Enable signal (which is why the
				-- 'if reb = 1' just below is commented).

				--if (reb = '1') then
				dob <= predob;
				--end if;

				-- read logic
				-- (in simulation returns 'force unknown' ('X') if METAVALUE in addrb)
				if (reb = '1') then
					-- pragma translate_off
					if is_X(addrb) then
						predob <= (others => 'X');
					else
					-- pragma translate_on
						predob <= mem_content(to_integer(unsigned(addrb)));
					-- pragma translate_off
					end if;
					-- pragma translate_on
				end if;

			end if; -- if rdlat

		end if; -- clk

	end process;

	-- pragma translate_off
	process(clk) is
	begin
		if (clk'event and clk = '1') then
			if reb = '1' and wea = '1' and addra = addrb then
				report "R & W at the same address in the same clock cycle in syncram_sdp"
					severity ERROR;
			end if;
		end if;
	end process;
	-- pragma translate_on

end architecture syn;

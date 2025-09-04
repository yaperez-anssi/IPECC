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

use work.ecc_customize.all;
use work.ecc_utils.all;
use work.ecc_pkg.all;
use work.ecc_log.all;

package mm_ndsp_pkg is

	function get_dsp_maxacc return positive;

	constant WEIGHT_BITS : positive := log2((2*w) - 1);

	type maccx_in_type is record
		rstm : std_logic;
		rstp : std_logic;
		ace : std_logic;
		bce : std_logic;
		pce : std_logic;
	end record;

	-- 'ndsp'
	--
	-- this is the actual number of DSP primitives that will be instanciated
	-- in the design at synthesis time, based on the user choice (parameter
	-- nbdsp) expressed in ecc_customize.vhd and the value of the parameter
	-- 'w' (defined from values of 'nn' and 'ww')
	-- So nbdsp is the user (designer) choice, ndsp is the value which is
	-- deduced from the user choice, in order to enforce that the nb of DSP
	-- blocks actually in the hardware does not exceed parameter 'w', which
	-- would not make sense
	constant ndsp : positive := set_ndsp; -- (s0)

	type maccx_array_in_type is array(0 to ndsp - 1) of maccx_in_type;

	-- constant NBRP is a basis term used in the calculation of the number of
	-- clock cycles that it takes from presenting read command to ORAM memory
	-- (or IRAM memory in the async = TRUE case) in order to get the first x_i
	-- operand term (or s_i or alpha_i, depending on the cycle of mult-&-acc
	-- 'mm_ndsp' is currently in, see 'xy', 'sp' & 'ap' in the ASCII "art" at
	-- the begining of file mm_ndsp.vhd) operand term, and the time by which
	-- the first term accumulated through the chain of DSP blocks has reached
	-- register r.acc.ppacc.
	--
	-- Now in order to get the exact nb of cycles, one must add the nb of x_i
	-- terms that were actually used to feed the burst of multiply-and-add into
	-- the chain of DSP blocks, this number being in the range [ 1 ... ndsp ].
	-- For instance, if the nb of x_i terms is 3, then the total latency is
	-- NBRP + 3.
	constant NBRP : positive :=
		sramlat + 1 -- ORAM read latency, incl. r.prod.rdata latch, see (s1)
		+ 1 -- latch into r.prod.bb, see (s38) in file mm_ndsp.vhd
		+ 1 -- latch into B reg. of first DSP block, see (s39) in mm_ndsp.vhd
		+ 2 -- latch into M & P register of first DSP block
		+ (ndsp - 1) -- accumulation through the chain of DSP blocks
		+ 1 -- latch into r.acc.ppend, see (s123) & (s34) in file mm_ndsp.vhd
		+ 1; -- latch into r.acc.ppacc, see (s44) in file mm_ndsp.vhd
		-- = sramlat + ndsp + 6

	-- constant NBRA is the same as NBRP except that it extends to the clock
	-- cycle by which the first term accumulated through the chain of DSP
	-- blocks is written into PRAM memory (or TRAM memory, as these memories
	-- are written in parallel).
	--
	-- Same remarks applies as for NBRP above.
	constant NBRA : positive := NBRP + sramlat + 5;
	                         -- = (2 * sramlat) + ndsp + 11

	-- Example values of NBR[PA] w/ sramlat = 2:
	--
	--     ndsp  |       NBRP      NBRA
	--    -----------------------------
	--      2    |        10        17
	--      5    |        13        20
	--      6    |        14        21
	--     16    |        24        31

	-- for bit-width of slkcnt counter, we must determine the max of the
	-- four quantities: NBRP + 6
	--                  NBRP + w + 3
	--                     w + 2
	--              and   2w + 7
	constant NB_SLK_MAX : natural := get_slk_max(NBRP, w, sramlat);

	-- In the definition of NB_SLK_BITS below, we add one extra bit to account
	-- for the possibility exists of a negative value (at the time when,
	-- in mm_ndsp.vhd, we compute quantities r.nndyn.slkpivot_[01]).
	constant NB_SLK_BITS : natural := log2(NB_SLK_MAX) + 1;

	-- range of MIN_SLK is 3 to w + 1
	constant MIN_SLK : positive := get_min_slk(sramlat, ndsp);

	-- range of MIN_SLK_LAST is 1 to w - 1
	constant MIN_SLK_LAST : positive := get_min_slk_last(ndsp);

end package mm_ndsp_pkg;

package body mm_ndsp_pkg is

	function get_dsp_maxacc return positive is
		variable tmp : positive := 2*ww + ln2(ndsp);
	begin
		if techno = spartan6 then tmp := 48;
		elsif techno = series7 or techno = virtex6 then tmp := 48;
		elsif techno = ultrascale then tmp := 48;
		elsif techno = ialtera then tmp := 64;
		elsif techno = asic then tmp := 2*ww + ln2(ndsp); -- no max
		end if;
		return tmp;
	end function get_dsp_maxacc;

end package body mm_ndsp_pkg;

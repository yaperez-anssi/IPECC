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

use work.ecc_utils.all;
use work.ecc_customize.all;
use work.ecc_pkg.all;

package ecc_trng_pkg is

	-- 'raw_ram_size'
	--
	-- This is the size in bits of TRNG memory in which all raw random bits are
	-- buffered (this memory can be accessible by software in HW unsecure mode
	-- only, to allow for statistical analysis of TRNG and entropy assessment).
	--
	-- Parameter 'trng_ramsz_raw' is defined in ecc_customize.
	constant raw_ram_size : positive := ge_pow_of_2(trng_ramsz_raw * 1024 * 8);

	-- 'pp_irn_width'
	-- 
	-- This is the bus size of data driven out by the TRNG post-processing
	-- component (if any, but there SHOULD be).
	constant pp_irn_width : positive := 32;

	-- 'irn_fifo_size_axi'
	--
	-- This is the size of the FIFO of TRNG internal random numbers served to
	-- the AXI interface, namely component ecc_axi.vhd (for on-the-fly masking
	-- of the scalar) expressed in number of elements it stores (ww-bit words).
	--
	-- Parameter 'trng_ramsz_axi' is defined in ecc_customize.
	constant irn_fifo_size_axi : positive := ge_pow_of_2(
		(trng_ramsz_axi * 1024 * 8) / ww);

	-- 'irn_fifo_size_efp'
	--
	-- This is the size of the FIFO of TRNG internal random numbers served to
	-- ecc_fp (Fp ALU) (for implementation of the NNRND instruction), expressed
	-- in number of elements it stores (ww-bit words).
	--
	-- Parameter 'trng_ramsz_efp' is defined in ecc_customize.
	constant irn_fifo_size_efp : positive := ge_pow_of_2(
		(trng_ramsz_efp * 1024 * 8) / ww);

	-- 'irn_fifo_size_crv'
	--
	-- This is the size of the FIFO of TRNG internal random numbers served to
	-- ecc_curve (for implementation of the shuffling of [XY]R[01] coordinates),
	-- expressed in number of elements it stores (2-bit words).
	--
	-- Parameter 'trng_ramsz_crv' is defined in ecc_customize.
	constant irn_fifo_size_crv : positive := ge_pow_of_2(
		(trng_ramsz_crv * 1024 * 8) / 2);

	-- 'irn_width_sh'
	--
	-- This is the size in bit of each TRNG internal random number served to
	-- ecc_fp_dram_sh (for implementation of the large numbers memory shuffling).
	constant irn_width_sh : positive := set_irn_width_sh; -- defined in ecc_pkg

	-- 'irn_fifo_size_shf'
	--
	-- This is the size of the FIFO of TRNG internal random numbers served to
	-- ecc_fp_dram_sh (for implementation of the large numbers memory shuffling),
	-- expressed in number of elements it stores (which are of size: the value
	-- returned by set_irn_width_sh() function, set in constant 'irn_width_sh',
	-- as this size depends on the time of memory shuffling set in ecc_customize).
	--
	-- Parameter trng_ramsz_shf is defined in ecc_customize.
	constant irn_fifo_size_shf : positive := ge_pow_of_2(
		(trng_ramsz_shf * 1024 * 8) / irn_width_sh);

end package ecc_trng_pkg;

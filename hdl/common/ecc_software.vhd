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
use work.ecc_log.all;
use work.ecc_pkg.all;
use work.ecc_trng_pkg.all;

package ecc_software is

	-- Software can rely on the following definitions for proper interaction
	-- with the IP, with addresses below to be considered as offset from
	-- the base address allocated to the IP in the whole AXI address system.
	--
	-- Please also refer to the comments that come with parameters 'AXIAW'
	-- and 'ADB' in ecc_pkg.vhd, including the small ASCII figure which
	-- captures in a simple way the definition of these two parameters.
	--
	-- Note: the disgracious 'rat' type is defined in ecc_pkg.vhd.
	--
	-- DO NOT remove the two comment banners '</start of ECC registers'
	-- and 'end of ECC registers/>' below, they are used by automatic
	-- scripting to generate the C header file (ecc_regs.h) used by
	-- driver to automatically get the proper address of IP hardware
	-- registers.
	--
	-- </start of ECC registers
	-- -----------------------------------------------
	-- addresses of all AXI-accessible write registers
	-- -----------------------------------------------
	constant W_CTRL : rat := std_nat(0, ADB);                -- 0x000
	constant W_WRITE_DATA : rat := std_nat(1, ADB);          -- 0x008
	constant W_R0_NULL : rat := std_nat(2, ADB);             -- 0x010
	constant W_R1_NULL : rat := std_nat(3, ADB);             -- 0x018
	constant W_PRIME_SIZE : rat := std_nat(4, ADB);          -- 0x020
	constant W_BLINDING : rat := std_nat(5, ADB);            -- 0x028
	constant W_SHUFFLE : rat := std_nat(6, ADB);             -- 0x030
	constant W_ZREMASK : rat := std_nat(7, ADB);             -- 0x038
	constant W_TOKEN : rat := std_nat(8, ADB);               -- 0x040
	constant W_IRQ : rat := std_nat(9, ADB);                 -- 0x048
	constant W_ERR_ACK : rat := std_nat(10, ADB);            -- 0x050
	constant W_SMALL_SCALAR : rat := std_nat(11, ADB);       -- 0x058
	constant W_SOFT_RESET : rat := std_nat(12, ADB);         -- 0x060
	-- reserved                                              -- 0x060...0x0f8
	-- (0x100: start of write HW unsecure/SCA features registers)
	constant W_DBG_HALT : rat := std_nat(32, ADB);           -- 0x100
	constant W_DBG_BKPT : rat := std_nat(33, ADB);           -- 0x108
	constant W_DBG_STEPS : rat := std_nat(34, ADB);          -- 0x110
	constant W_DBG_TRIG_ACT : rat := std_nat(35, ADB);       -- 0x118
	constant W_DBG_TRIG_UP : rat := std_nat(36, ADB);        -- 0x120
	constant W_DBG_TRIG_DOWN : rat := std_nat(37, ADB);      -- 0x128
	constant W_DBG_OP_WADDR : rat := std_nat(38, ADB);       -- 0x130
	constant W_DBG_OPCODE : rat := std_nat(39, ADB);         -- 0x138
	constant W_DBG_TRNG_CFG : rat := std_nat(40, ADB);       -- 0x140
	constant W_DBG_TRNG_RESET : rat := std_nat(41, ADB);     -- 0x148
	constant W_DBG_TRNG_CTRL_POSTP : rat:= std_nat(42, ADB); -- 0x150
	constant W_DBG_TRNG_CTRL_BYPASS: rat:= std_nat(43, ADB); -- 0x158
	constant W_DBG_TRNG_CTRL_NNRND : rat:= std_nat(44, ADB); -- 0x160
	constant W_DBG_TRNG_CTRL_DIAG : rat := std_nat(45, ADB); -- 0x168
	constant W_DBG_TRNG_RAW_READ : rat := std_nat(46, ADB);  -- 0x170
	constant W_DBG_FP_WADDR : rat := std_nat(47, ADB);       -- 0x178
	constant W_DBG_FP_WDATA : rat := std_nat(48, ADB);       -- 0x180
	constant W_DBG_FP_RADDR : rat := std_nat(49, ADB);       -- 0x188
	constant W_DBG_CFG_XYSHUF : rat := std_nat(50, ADB);     -- 0x190
	constant W_DBG_CFG_AXIMSK : rat := std_nat(51, ADB);     -- 0x198
	constant W_DBG_CFG_TOKEN : rat := std_nat(52, ADB);      -- 0x1a0
	constant W_ATTACK_CFG_0 : rat := std_nat(53, ADB);       -- 0x1a8
	constant W_ATTACK_CFG_1 : rat := std_nat(54, ADB);       -- 0x1b0
	constant W_ATTACK_CFG_2 : rat := std_nat(55, ADB);       -- 0x1b8
	-- reserved                                              -- 0x1c0...0x1f8
	-- ----------------------------------------------
	-- addresses of all AXI-accessible read registers
	-- ----------------------------------------------
	constant R_STATUS : rat := std_nat(0, ADB);              -- 0x000
	constant R_READ_DATA : rat := std_nat(1, ADB);           -- 0x008
	constant R_CAPABILITIES : rat := std_nat(2, ADB);        -- 0x010
	constant R_HW_VERSION : rat := std_nat(3, ADB);          -- 0x018
	constant R_PRIME_SIZE : rat := std_nat(4, ADB);          -- 0x020
	-- reserved                                              -- 0x028...0x0f8
	-- (0x100: start of read HW unsecure/SCA features registers)
	constant R_DBG_CAPABILITIES_0 : rat := std_nat(32, ADB); -- 0x100
	constant R_DBG_CAPABILITIES_1 : rat := std_nat(33, ADB); -- 0x108
	constant R_DBG_CAPABILITIES_2 : rat := std_nat(34, ADB); -- 0x110
	constant R_DBG_STATUS : rat := std_nat(35, ADB);         -- 0x118
	constant R_DBG_TIME : rat := std_nat(36, ADB);           -- 0x120
	constant R_DBG_TRNG_RAWDUR : rat := std_nat(37, ADB);    -- 0x128
	constant R_DBG_TRNG_STATUS : rat := std_nat(38, ADB);    -- 0x130
	constant R_DBG_TRNG_RAW_DATA : rat := std_nat(39, ADB);  -- 0x138
	constant R_DBG_TRNG_DIAG_MIN : rat := std_nat(40, ADB);  -- 0x140
	constant R_DBG_TRNG_DIAG_MAX : rat := std_nat(41, ADB);  -- 0x148
	constant R_DBG_TRNG_DIAG_OK : rat := std_nat(42, ADB);   -- 0x150
	constant R_DBG_TRNG_DIAG_STARV: rat := std_nat(43, ADB); -- 0x158
	constant R_DBG_FP_RDATA : rat := std_nat(44, ADB);       -- 0x160
	constant R_DBG_FP_RDATA_RDY : rat := std_nat(45, ADB);   -- 0x168
	constant R_DBG_EXP_FLAGS : rat := std_nat(46, ADB);      -- 0x170
	constant R_DBG_CLK_MHZ : rat := std_nat(47, ADB);        -- 0x178
	constant R_DBG_CLKMM_MHZ : rat := std_nat(48, ADB);      -- 0x180
	-- TODO: remove R_DBG_HW_CONFIG_[012]; R_DBG_HW_TEST_NB & R_DBG_HW_RANDOM
	constant R_DBG_HW_CONFIG_0 : rat := std_nat(49, ADB);    -- 0x188
	constant R_DBG_HW_CONFIG_1 : rat := std_nat(50, ADB);    -- 0x190
	constant R_DBG_HW_CONFIG_2 : rat := std_nat(51, ADB);    -- 0x198
	constant R_DBG_HW_TEST_NB : rat := std_nat(52, ADB);     -- 0x1a0
	constant R_DBG_HW_RANDOM : rat := std_nat(53, ADB);      -- 0x1a8
	-- reserved                                              -- 0x1b0...0x1f8
	-- end of ECC registers/>

	-- Register bank of pseudo TRNG device (external to the IP), if any.
	-- Write-only registers
	constant PSEUDOTRNG_W_SOFT_RESET : std_logic := '0';     -- 0x00
	constant PSEUDOTRNG_W_WRITE_DATA : std_logic := '1';     -- 0x08
	-- Read-only register
	constant PSEUDOTRNG_R_FIFO_COUNT : std_logic := '0';     -- 0x00

	-- ----------------------------------------------
	-- bit positions / fields in write registers
	-- ----------------------------------------------
	-- bit positions in W_CTRL register
	constant CTRL_KP : natural := 0;
	constant CTRL_PT_ADD : natural := 1;
	constant CTRL_PT_DBL : natural := 2;
	constant CTRL_PT_CHK : natural := 3;
	constant CTRL_PT_NEG : natural := 4;
	constant CTRL_PT_EQU : natural := 5;
	constant CTRL_PT_OPP : natural := 6;
	-- bits 7-9 reserved
	-- bits 10-11 reserved
	constant CTRL_RD_TOKEN : natural := 12;
	constant CTRL_WRITE_NB : natural := 16;
	constant CTRL_READ_NB : natural := 17;
	constant CTRL_WRITE_K : natural := 18;
	-- bit 19 reserved
	constant CTRL_NBADDR_LSB : natural := 20;
	constant CTRL_NBADDR_SZ : natural := 12;
	constant CTRL_NBADDR_MSB : natural := CTRL_NBADDR_LSB + CTRL_NBADDR_SZ - 1;

	-- bit positions in W_R0_NULL / W_R1_NULL registers
	constant WR0_IS_NULL : natural := 0;
	constant WR1_IS_NULL : natural := 0;

	-- bit positions in W_BLINDING register
	constant BLD_EN : natural := 0;
	constant BLD_BITS_LSB : natural := 4;
	constant BLD_BITS_MSB : natural := BLD_BITS_LSB + log2(nn) - 1;

	-- bit positions in W_SHUFFLE register
	constant SHUF_EN : natural := 0;

	-- bit positions in W_ZREMASK register
	constant ZMSK_EN : natural := 0;
	constant ZMSK_LSB : natural := 16;
	constant ZMSK_MSB : natural := ZMSK_LSB + log2(nn - 1) - 1;

	-- bit positions in W_IRQ register
	constant IRQ_EN : natural := 0;

	-- bit positions in W_ERR_ACK
	-- same as the ERR_* bits in R_STATUS (see below)

	-- bit positions in W_PRIME_SIZE register
	constant PMSZ_VALNN_LSB : natural := 0;
	constant PMSZ_VALNN_SZ : natural := log2(nn);
	constant PMSZ_VALNN_MSB : natural := PMSZ_VALNN_LSB + PMSZ_VALNN_SZ - 1;

	-- bit positions in W_DBG_HALT register
	constant DBG_HALT : natural := 0;

	-- bit positions in W_DBG_BKPT register
	constant DBG_BKPT_EN : natural := 0;
	constant DBG_BKPT_ID_LSB : natural := 1;
	constant DBG_BKPT_ID_MSB : natural := 2;
	constant DBG_BKPT_ADDR_LSB : natural := 4;
	constant DBG_BKPT_ADDR_MSB : natural := 4 + IRAM_ADDR_SZ - 1;
	constant DBG_BKPT_NBBIT_LSB : natural := 16;
	constant DBG_BKPT_NBBIT_MSB : natural := 27;
	constant DBG_BKPT_STATE_LSB : natural := 28;
	constant DBG_BKPT_STATE_MSB : natural := 31;

	-- bit positions in W_DBG_STEPS register
	constant DBG_OPCODE_RUN : natural := 0;
	constant DBG_OPCODE_NB_LSB : natural := 8;
	constant DBG_OPCODE_NB_MSB : natural := 23;
	constant DBG_RESUME : natural := 28;

	-- bit positions in W_DBG_TRIG_ACT
	constant TRIG_EN : natural := 0;

	-- bit positions in W_DBG_TRIG_UP & W_DBG_TRIG_DOWN
	constant TRIG_LSB : natural := 0;
	constant TRIG_MSB : natural := 31;

	-- bit positions in W_DBG_TRNG_CFG register
	constant DBG_TRNG_VONM : natural := 0;
	constant DBG_TRNG_TA_LSB : natural := 4;
	constant DBG_TRNG_TA_MSB : natural := 19;
	constant DBG_TRNG_IDLE_LSB : natural := 20;
	constant DBG_TRNG_IDLE_MSB : natural := 23;
	constant DBG_TRNG_USE_PSEUDO : natural := 24;

	-- bit positions in W_DBG_TRNG_RESET register
	constant DBG_TRNG_RESET_RAW : natural := 0;
	constant DBG_TRNG_RESET_IRN : natural := 4;

	-- bit positions in W_DBG_TRNG_CTRL_POSTP register
	constant DBG_TRNG_CTRL_POSTPROC_DISABLE : natural := 0;
	-- to allow software to read the content of raw random FIFO
	constant DBG_TRNG_CTRL_RAW_DISABLE_FIFO_READ_PORT : natural := 4;

	-- bit positions in W_DBG_TRNG_CTRL_BYPASS register
	constant DBG_TRNG_CTRL_COMPLETE_BYPASS : natural := 0;
	constant DBG_TRNG_CTRL_COMPLETE_BYPASS_BIT : natural := 4;

	-- bit positions in W_DBG_TRNG_CTRL_NNRND register
	constant DBG_TRNG_CTRL_NNRND_DETERMINISTIC : natural := 0;

	-- bit positions in W_DBG_TRNG_CTRL_DIAG register
	constant DBG_TRNG_CTRL_DIAG_SELECT_MSB : natural := 2;
	constant DBG_TRNG_CTRL_DIAG_SELECT_LSB : natural := 0;
	constant DBG_TRNG_CTRL_DIAG_AXI : natural := 0;
	constant DBG_TRNG_CTRL_DIAG_EFP : natural := 1;
	constant DBG_TRNG_CTRL_DIAG_CRV : natural := 2;
	constant DBG_TRNG_CTRL_DIAG_SHF : natural := 3;
	constant DBG_TRNG_CTRL_DIAG_RAW : natural := 4;

	-- bit positions in W_DBG_TRNG_RAW_READ register
	constant DBG_TRNG_CTRL_RAW_READ : natural := 0;
	constant DBG_TRNG_CTRL_RAW_ADDR_LSB : natural := 4;
	constant DBG_TRNG_CTRL_RAW_ADDR_MSB : natural := 23;

	-- bit positions in W_DBG_CFG_XYSHUF register
	constant XYSHF_EN : natural := 0;

	-- bit positions in W_DBG_CFG_AXIMSK register
	constant AXIMSK_EN : natural := 0;

	-- bit positions in W_DBG_CFG_TOKEN register
	constant TOK_EN : natural := 0;

	-- bit positions in W_ATTACK_CFG_0 register
	constant DO_NOT_ALWAYS_ADD : natural := 0;
	constant DO_NO_COLLISION_CR: natural := 4;

	-- bit positions in W_ATTACK_CFG_1 register
	constant DO_NO_NNRND_SF : natural := 0;

	-- bit positions in W_ATTACK_CFG_2 register
	constant CLK_DIV_LSB : natural := 0;
	constant CLK_DIV_MSB : natural := 15;
	constant CLKMM_DIV_LSB : natural := 16;
	constant CLKMM_DIV_MSB : natural := 31;

	-- ----------------------------------------------
	-- bit positions / fields in read registers
	-- ----------------------------------------------
	-- bit positions in R_STATUS register (AXI interface w/ software)
	constant STATUS_BUSY : natural := 0;
	constant STATUS_KP : natural := 4;
	constant STATUS_MTY : natural := 5;
	constant STATUS_POP : natural := 6;
	constant STATUS_R_OR_W : natural := 7;
	constant STATUS_INIT : natural := 8;
	constant STATUS_NNDYNACT : natural := 9;
	constant STATUS_ENOUGH_RND_WK : natural := 10;
	constant STATUS_YES : natural := 11;
	constant STATUS_R0_IS_NULL : natural := 12;
	constant STATUS_R1_IS_NULL : natural := 13;
	constant STATUS_TOKEN_GEN : natural := 14;
	constant STATUS_ERR_LSB : natural := 16;
	constant STATUS_ERR_IN_PT_NOT_ON_CURVE : natural := STATUS_ERR_LSB;
	constant STATUS_ERR_OUT_PT_NOT_ON_CURVE : natural := STATUS_ERR_LSB + 1;
	constant STATUS_ERR_I_COMP : natural := STATUS_ERR_LSB + 2;
	constant STATUS_ERR_I_WREG_FBD : natural := STATUS_ERR_LSB + 3;
	constant STATUS_ERR_I_KP_FBD : natural := STATUS_ERR_LSB + 4;
	constant STATUS_ERR_I_NNDYN : natural := STATUS_ERR_LSB + 5;
	constant STATUS_ERR_I_POP_FBD : natural := STATUS_ERR_LSB + 6;
	constant STATUS_ERR_I_RDNB_FBD : natural := STATUS_ERR_LSB + 7;
	constant STATUS_ERR_I_BLN : natural := STATUS_ERR_LSB + 8;
	constant STATUS_ERR_I_UNKNOWN_REG : natural := STATUS_ERR_LSB + 9;
	constant STATUS_ERR_I_TOKEN : natural := STATUS_ERR_LSB + 10;
	constant STATUS_ERR_I_SHUFFLE : natural := STATUS_ERR_LSB + 11;
	constant STATUS_ERR_I_ZREMASK : natural := STATUS_ERR_LSB + 12;
	constant STATUS_ERR_I_NOT_ENOUGH_RANDOM_WK : natural := STATUS_ERR_LSB + 13;
	constant STATUS_ERR_I_RREG_FBD : natural := STATUS_ERR_LSB + 14;
	constant STATUS_ERR_MSB : natural := 31;
	-- range of field .ierrid in ecc_axi.vhd
	constant STATUS_ERR_I_LSB : natural := STATUS_ERR_LSB + 2;
	constant STATUS_ERR_I_MSB : natural := STATUS_ERR_MSB;

	-- bit positions in R_CAPABILITIES register
	constant CAP_DBG_N_PROD : natural := 0;
	constant CAP_SHF : natural := 4;
	constant CAP_NNDYN : natural := 8;
	constant CAP_W64 : natural := 9;
	constant CAP_NNMAX_LSB : natural := 12;
	constant CAP_NNMAX_MSB : natural := CAP_NNMAX_LSB + log2(nn) - 1;

	-- bit positions in R_PRIME_SIZE
	--   (same definitions as for W_PRIME_SIZE register, see above)

	-- bit positions in R_HW_VERSION
	constant HW_VERSION_MAJ_LSB : natural := 24;
	constant HW_VERSION_MAJ_MSB : natural := 31;
	constant HW_VERSION_MIN_LSB : natural := 16;
	constant HW_VERSION_MIN_MSB : natural := 23;
	constant HW_VERSION_PATCH_LSB : natural := 0;
	constant HW_VERSION_PATCH_MSB : natural := 15;

	-- bit positions in R_DBG_CAPABILITIES_1 & R_DBG_CAPABILITIES_2
	constant DBG_CAP_SPLIT_1 : natural := 16;
	constant DBG_CAP_SPLIT_2 : natural := 20;

	-- bit positions in R_DBG_STATUS register
	constant DBG_STATUS_HALTED : natural := 0;
	constant DBG_STATUS_BKID_LSB : natural := 1;
	constant DBG_STATUS_BKID_MSB : natural := 2;
	constant DBG_STATUS_BK_HIT : natural := 3;
	constant DBG_STATUS_PC_LSB : natural := 4;
	constant DBG_STATUS_PC_MSB : natural := 15;
	constant DBG_STATUS_NBIT_LSB : natural := 16;
	constant DBG_STATUS_NBIT_MSB : natural := 27;
	constant DBG_STATUS_STATE_LSB : natural := 28;
	constant DBG_STATUS_STATE_MSB : natural := 31;

	-- bit positions in R_DBG_TIME register
	constant DBG_TIME_MSB : natural := 31;
	constant DBG_TIME_LSB : natural := 0;

	-- bit positions in R_DBG_TRNG_RAWDUR register
	constant DBG_RAWDUR_MSB : natural := 31;
	constant DBG_RAWDUR_LSB : natural := 0;

	-- bit positions in R_DBG_TRNG_STATUS register
	constant DBG_TRNG_STATUS_RAW_FULL : natural := 0;
	constant DBG_TRNG_STATUS_RAW_WADDR_MSB : natural := 31;
	constant DBG_TRNG_STATUS_RAW_WADDR_LSB : natural := 8;

	-- bit positions in R_DBG_TRNG_RAW_DATA register
	constant DBG_TRNG_RAW_BIT_POS : natural := 0;

	-- bit positions in R_DBG_TRNG_DIAG_MIN register
	constant R_DBG_TRNG_DIAG_MIN_MSB : natural := 31;
	constant R_DBG_TRNG_DIAG_MIN_LSB : natural := 0;

	-- bit positions in R_DBG_TRNG_DIAG_MAX register
	constant R_DBG_TRNG_DIAG_MAX_MSB : natural := 31;
	constant R_DBG_TRNG_DIAG_MAX_LSB : natural := 0;

	-- bit positions in R_DBG_TRNG_DIAG_OK register
	constant R_DBG_TRNG_DIAG_OK_MSB : natural := 31;
	constant R_DBG_TRNG_DIAG_OK_LSB : natural := 0;

	-- bit positions in R_DBG_TRNG_DIAG_STARV register
	constant R_DBG_TRNG_DIAG_ST_MSB : natural := 31;
	constant R_DBG_TRNG_DIAG_ST_LSB : natural := 0;

	-- bit positions in R_DBG_FP_RDATA register
	constant R_DBG_FP_RDATA_MSB : natural := ww - 1;
	constant R_DBG_FP_RDATA_LSB : natural := 0;

	-- bit positions in R_DBG_FP_RDATA_RDY
	constant DBG_FP_RDATA_IS_RDY : natural := 0;

	-- bit positions in R_DBG_EXP_FLAGS register
	--   (this register will be removed)

	-- bit positions in R_DBG_CLK_MHZ register
	constant R_DBG_CLK_MHZ_PRECNT :  natural := 16;
	constant R_DBG_CLK_MHZ_MSB : natural := 31;
	constant R_DBG_CLK_MHZ_LSB : natural := 0;

	-- bit positions in R_DBG_CLKMM_MHZ register
	constant R_DBG_CLKMM_MHZ_PRECNT :  natural := 16;
	constant R_DBG_CLKMM_MHZ_MSB : natural := 31;
	constant R_DBG_CLKMM_MHZ_LSB : natural := 0;

end package ecc_software;

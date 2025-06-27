/*
 *  Copyright (C) 2023 - This file is part of IPECC project
 *
 *  Authors:
 *      Karim KHALFALLAH <karim.khalfallah@ssi.gouv.fr>
 *      Ryad BENADJILA <ryadbenadjila@gmail.com>
 *
 *  Contributors:
 *      Adrian THILLARD
 *      Emmanuel PROUFF
 *
 *  This software is licensed under GPL v2 license.
 *  See LICENSE file at the root folder of the project.
 */

/* The low level driver for the HW accelerator */
#include "hw_accelerator_driver.h"

#include "ecc_addr.h"
#include "ecc_vars.h"
#include "ecc_states.h"
#include "ecc_regs.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#if defined(WITH_EC_HW_ACCELERATOR) && !defined(WITH_EC_HW_SOCKET_EMUL)
/**************************************************************************/
/******************************* IPECC ************************************/
/**************************************************************************/

#include <stdint.h>
#include <stdio.h>

/* Platform specific elements */
#include "hw_accelerator_driver_ipecc_platform.h"

/***********************************************************/
/* We default to 32-bit hardware IP */
#if !defined(WITH_EC_HW_ACCELERATOR_WORD32) && !defined(WITH_EC_HW_ACCELERATOR_WORD64)
#define WITH_EC_HW_ACCELERATOR_WORD32
#endif
#if defined(WITH_EC_HW_ACCELERATOR_WORD32) && defined(WITH_EC_HW_ACCELERATOR_WORD64)
#error "WITH_EC_HW_ACCELERATOR_WORD32 and WITH_EC_HW_ACCELERATOR_WORD64 cannot be both defined!"
#endif

#if defined(WITH_EC_HW_ACCELERATOR_WORD32)
typedef volatile uint32_t ip_ecc_word;
#define IPECC_WORD_FMT "%08x"
#else
typedef volatile uint64_t ip_ecc_word;
#define IPECC_WORD_FMT "%016x"
#endif

/*
 * DIV(i, s) returns the number of s-bit limbs required to encode
 * an i-bit number.
 *
 * Obviously this is also equal to the ceil() function applied to
 * integer quotient i / s.
 */
#define DIV(i, s) \
	( ((i) % (s)) ? ( ((i) / (s)) + 1 ) : ( (i) / (s)) )

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

/*
 * ge_pow_of_2(i) returns the power-of-2 which is either equal to
 * or directly greater than i.
 */
static inline int ge_pow_of_2(uint32_t i, uint32_t* pw)
{
	*pw = 1;

	if (i > (0x1UL<<31)) {
		printf("Error: out-of-range input in call to function ge_pow_of_2().\n\r");
		goto err;
	}
	while (*pw < i)
	{
		(*pw) *= 2;
	}
	return 0;
err:
	return -1;
}

/****************************/
/* IPECC register addresses */
/****************************/

/* GET and SET the control, status and other internal
 * registers of the IP. These are 32-bit or 64-bit wide
 * depending on the IP configuration.
 */

#if defined(WITH_EC_HW_ACCELERATOR_WORD64)
/* In 64 bits, reverse words endianness */
#define IPECC_GET_REG(reg)	((*((ip_ecc_word*)((reg)))) & 0xffffffff)
#define IPECC_SET_REG(reg, val)	\
	((*((ip_ecc_word*)((reg)))) = (((((ip_ecc_word)(val)) & 0xffffffff) << 32) \
		| (((ip_ecc_word)(val)) >> 32)))
#else
#define IPECC_GET_REG(reg)		 (*((ip_ecc_word*)((reg))))
#define IPECC_SET_REG(reg, val)		((*((ip_ecc_word*)((reg)))) = ((ip_ecc_word)(val)))
#endif

/***********************************************************/
/***********************************************************/
/* The base address of our hardware: this must be
 * configured by the software somehow.
 *
 * This is configured by the lower layer that implements platform
 * specific routines.
 */
static volatile uint64_t *ipecc_baddr = NULL;
/* Uncomment line below to use the Pseudo TRNG feature
 * (not yet officially released on the IPECC repo).
 */
/* static volatile uint64_t *ipecc_pseudotrng_baddr = NULL; */

/* NOTE: addresses in the IP are 64-bit aligned */
#define IPECC_ALIGNED(a) ((a) / sizeof(uint64_t))

/* *****************************************************************
 *   Address mapping of registers have been removed as they are    *
 *  now automatically imported from hardware through <ecc_regs.h>. *
 *******************************************************************/

/* Optional device acting as "pseudo TRNG" device, which software can push
 * some byte stream/file to.
 *
 * Using the same byte stream/file in the VHDL testbench of the IP hence
 * makes it possible to get full bit-by-bit and instruction-per-instruction 
 * comparison between VHDL simulation & real hardware.
 *
 * This is not "CABA" (cycle-accurate, bit-accurate) simulation yet,
 * rather "IACA" (instruction-accurate, bit-accurate) simulation - which,
 * using the breakpoint & the step-by-step features provided with the IP
 * and its driver, can be a powerful debugging tool.
 * */
/* Write-only registers */
#define IPECC_PSEUDOTRNG_W_SOFT_RESET   (ipecc_pseudotrng_baddr + IPECC_ALIGNED(0x00))
#define IPECC_PSEUDOTRNG_W_WRITE_DATA   (ipecc_pseudotrng_baddr + IPECC_ALIGNED(0x08))

/* Read-only registers */
#define IPECC_PSEUDOTRNG_R_FIFO_COUNT   (ipecc_pseudotrng_baddr + IPECC_ALIGNED(0x00))

/*************************************
 * Bit & fields positions in registers
 *************************************/

/* Fields for W_CTRL */
#define IPECC_W_CTRL_PT_KP		(((uint32_t)0x1) << 0)
#define IPECC_W_CTRL_PT_ADD		(((uint32_t)0x1) << 1)
#define IPECC_W_CTRL_PT_DBL		(((uint32_t)0x1) << 2)
#define IPECC_W_CTRL_PT_CHK		(((uint32_t)0x1) << 3)
#define IPECC_W_CTRL_PT_NEG		(((uint32_t)0x1) << 4)
#define IPECC_W_CTRL_PT_EQU		(((uint32_t)0x1) << 5)
#define IPECC_W_CTRL_PT_OPP		(((uint32_t)0x1) << 6)
/* bits 7-11 reserved */
#define IPECC_W_CTRL_RD_TOKEN   (((uint32_t)0x1) << 12)
#define IPECC_W_CTRL_WRITE_NB		(((uint32_t)0x1) << 16)
#define IPECC_W_CTRL_READ_NB		(((uint32_t)0x1) << 17)
#define IPECC_W_CTRL_WRITE_K		(((uint32_t)0x1) << 18)
#define IPECC_W_CTRL_NBADDR_MSK		(0xfff)
#define IPECC_W_CTRL_NBADDR_POS		(20)

/* Fields for W_R0_NULL & W_R1_NULL */
#define IPECC_W_POINT_IS_NULL      (((uint32_t)0x1) << 0)
#define IPECC_W_POINT_IS_NOT_NULL      (((uint32_t)0x0) << 0)

/* Fields for W_PRIME_SIZE & R_PRIME_SIZE */
#define IPECC_W_PRIME_SIZE_POS   (0)
#define IPECC_W_PRIME_SIZE_MSK   (0xffff)

/* Fields for W_BLINDING */
#define IPECC_W_BLINDING_EN		(((uint32_t)0x1) << 0)
#define IPECC_W_BLINDING_BITS_MSK	(0xfffffff)
#define IPECC_W_BLINDING_BITS_POS	(4)
#define IPECC_W_BLINDING_DIS		(((uint32_t)0x0) << 0)

/* Fields for W_SHUFFLE */
#define IPECC_W_SHUFFLE_EN    (((uint32_t)0x1) << 0)
#define IPECC_W_SHUFFLE_DIS    (((uint32_t)0x0) << 0)

/* Fields for W_ZREMASK */
#define IPECC_W_ZREMASK_EN    (((uint32_t)0x1) << 0)
#define IPECC_W_ZREMASK_BITS_MSK	(0xffff)
#define IPECC_W_ZREMASK_BITS_POS	(16)
#define IPECC_W_ZREMASK_DIS    (((uint32_t)0x0) << 0)

/* Fields for W_TOKEN */
/* no field here: action is performed simply by writing to the
   register address, whatever the value written */

/* Fields for W_IRQ */
/* enable IRQ (1) or disable (0) */
#define IPECC_W_IRQ_EN    (((uint32_t)0x1) << 0)

/* Fields for W_ERR_ACK */
/* These are the same as for the ERR_ bits in R_STATUS (see below) */

/* Fields for W_SMALL_SCALAR  */
#define IPECC_W_SMALL_SCALAR_K_POS    (0)
#define IPECC_W_SMALL_SCALAR_K_MSK    (0xffff)

/* Fields for W_SOFT_RESET */
/* no field here: action is performed simply by writing to the
   register address, whatever the value written */

/* Fields for W_DBG_HALT */
#define IPECC_W_DBG_HALT_DO_HALT   (((uint32_t)0x1) << 0)

/* Fields for W_DBG_BKPT */
#define IPECC_W_DBG_BKPT_EN     (((uint32_t)0x1) << 0)
#define IPECC_W_DBG_BKPT_DIS    (((uint32_t)0x0) << 0)
#define IPECC_W_DBG_BKPT_ID_POS    (1)
#define IPECC_W_DBG_BKPT_ID_MSK    (0x3)
#define IPECC_W_DBG_BKPT_ADDR_POS   (4)
#define IPECC_W_DBG_BKPT_ADDR_MSK   (0xfff)
#define IPECC_W_DBG_BKPT_NBIT_POS   (16)
#define IPECC_W_DBG_BKPT_NBIT_MSK   (0xfff)
#define IPECC_W_DBG_BKPT_STATE_POS     (28)
#define IPECC_W_DBG_BKPT_STATE_MSK     (0xf)

/* Fields for W_DBG_STEPS */
#define IPECC_W_DBG_STEPS_RUN_NB_OP    (((uint32_t)0x1) << 0)
#define IPECC_W_DBG_STEPS_NB_OP_POS    (8)
#define IPECC_W_DBG_STEPS_NB_OP_MSK    (0xffff)
#define IPECC_W_DBG_STEPS_RESUME     (((uint32_t)0x1) << 28)

/* Fields for W_DBG_TRIG_ACT */
/* enable trig (1) or disable it (0) */
#define IPECC_W_DBG_TRIG_ACT_EN     (((uint32_t)0x1) << 0)
#define IPECC_W_DBG_TRIG_ACT_DIS    (((uint32_t)0x0) << 0)

/* Fields for W_DBG_TRIG_UP & W_DBG_TRIG_DOWN */
#define IPECC_W_DBG_TRIG_POS    (0)
#define IPECC_W_DBG_TRIG_MSK    (0xffffffff)

/* Fields for W_DBG_OP_WADDR */
#define IPECC_W_DBG_OP_WADDR_POS   (0)
#define IPECC_W_DBG_OP_WADDR_MSK   (0xffff)

/* Fields for W_DBG_OPCODE */
#define IPECC_W_DBG_OPCODE_POS   (0)
#define IPECC_W_DBG_OPCODE_MSK   (0xffffffff)

/* Fields for W_DBG_TRNG_CFG */
/* Von Neumann debiaser activate */
#define IPECC_W_DBG_TRNG_CFG_ACTIVE_DEBIAS		(((uint32_t)0x1) << 0)
/* TA value (in nb of system clock cycles) */
#define IPECC_W_DBG_TRNG_CFG_TA_POS			(4)
#define IPECC_W_DBG_TRNG_CFG_TA_MSK			(0xffff)
/* latency (in nb of system clock cycles) between each phase of
   one-bit generation in the TRNG */
#define IPECC_W_DBG_TRNG_CFG_TRNG_IDLE_POS		(20)
#define IPECC_W_DBG_TRNG_CFG_TRNG_IDLE_MSK		(0xf)
#define IPECC_W_DBG_TRNG_CFG_USE_PSEUDO   (((uint32_t)0x1) << 24)

/* Fields for W_DBG_TRNG_RESET */
/* Reset the raw FIFO */
#define IPECC_W_DBG_TRNG_RESET_FIFO_RAW		(((uint32_t)0x1) << 0)
/* Reset the internal random numbers FIFOs */
#define IPECC_W_DBG_TRNG_RESET_FIFO_IRN		(((uint32_t)0x1) << 4)

/* Fields for W_DBG_TRNG_CTRL_POSTP */
/* Disable the TRNG post processing logic that pulls bytes
 * from the raw random source */
#define IPECC_W_DBG_TRNG_CTRL_POSTPROC_DISABLE_POS  (0)
/* Disable the read function of the raw random FIFO
 * (to allow debug software to read & statistically analyze
 * the raw random bits). */
#define IPECC_W_DBG_TRNG_CTRL_RAW_DISABLE_FIFO_READ_PORT_POS   (4)

/* Fields for W_DBG_TRNG_CTRL_BYPASS */
/* Complete bypass of the TRNG (1) or not (0) */
#define IPECC_W_DBG_TRNG_CTRL_TRNG_BYPASS			(((uint32_t)0x1) << 0)
/* Deterministic bit value produced when complete bypass is on */
#define IPECC_W_DBG_TRNG_CTRL_TRNG_BYPASS_VAL_POS		(4)

/* Fields for W_DBG_TRNG_CTRL_NNRND */
#define IPECC_W_DBG_TRNG_CTRL_NNRND_DETERMINISTIC   (((uint32_t)0x1) << 0)

/* Fields for W_DBG_TRNG_CTRL_DIAG */
#define IPECC_W_DBG_TRNG_CTRL_DIAG_POS   (0)
#define IPECC_W_DBG_TRNG_CTRL_DIAG_MSK   (0x7)
#define IPECC_W_DBG_TRNG_CTRL_DIAG_AXI   (0)
#define IPECC_W_DBG_TRNG_CTRL_DIAG_EFP   (1)
#define IPECC_W_DBG_TRNG_CTRL_DIAG_CRV   (2)
#define IPECC_W_DBG_TRNG_CTRL_DIAG_SHF   (3)
#define IPECC_W_DBG_TRNG_CTRL_DIAG_RAW   (4)

/* Fields for W_DBG_TRNG_RAW_READ */
/* Read one bit from raw FIFO */
#define IPECC_W_DBG_TRNG_CTRL_RAWFIFO_READ		(((uint32_t)0x1) << 0)
/* Reading offset in bits inside the FIFO on 20 bits */
#define IPECC_W_DBG_TRNG_CTRL_RAWFIFO_RADDR_MSK		(0xfffff)
#define IPECC_W_DBG_TRNG_CTRL_RAWFIFO_RADDR_POS		(4)

/* Fields for IPECC_W_DBG_FP_WADDR */
#define IPECC_W_DBG_FP_WADDR_POS     (0)
#define IPECC_W_DBG_FP_WADDR_MSK     (0xffffffff)

/* Fields for IPECC_W_DBG_FP_WDATA & IPECC_R_DBG_FP_RDATA */
#define IPECC_W_DBG_FP_DATA_POS     (0)
#define IPECC_W_DBG_FP_DATA_MSK     (0xffffffff)

/* Fields for IPECC_W_DBG_FP_RADDR */
#define IPECC_W_DBG_FP_RADDR_POS     (0)
#define IPECC_W_DBG_FP_RADDR_MSK     (0xffffffff)

/* Fields for IPECC_W_DBG_CFG_XYSHUF */
#define IPECC_W_DBG_CFG_XYSHUF_EN    (((uint32_t)0x1) << 0)
#define IPECC_W_DBG_CFG_XYSHUF_DIS    (((uint32_t)0x0) << 0)

/* Fields for IPECC_W_DBG_CFG_AXIMSK */
#define IPECC_W_DBG_CFG_AXIMSK_EN    (((uint32_t)0x1) << 0)
#define IPECC_W_DBG_CFG_AXIMSK_DIS    (((uint32_t)0x0) << 0)

/* Fields for IPECC_W_DBG_CFG_TOKEN */
#define IPECC_W_DBG_CFG_TOKEN_EN    (((uint32_t)0x1) << 0)
#define IPECC_W_DBG_CFG_TOKEN_DIS    (((uint32_t)0x0) << 0)

/* Fields for IPECC_W_ATTACK_CFG_0 */
#define IPECC_W_ATK_NOT_ALWAYS_ADD   (((uint32_t)0x1) << 0)
#define IPECC_W_ATK_NO_COLLISION_CR   (((uint32_t)0x1) << 4)

/* Fields for IPECC_W_ATTACK_CFG_1 */
#define IPECC_W_ATK_NO_NNRND_SF     (((uint32_t)0x1) << 0)

/* Fields for IPECC_W_ATTACK_CFG_2 */
#define IPECC_W_ATK_DIV_ENABLE      (0x1)
#define IPECC_W_ATK_DIVMM_ENABLE    (0x10000)
#define IPECC_W_ATK_DIV_FACTOR_POS     (0)
#define IPECC_W_ATK_DIV_FACTOR_MASK    (0xfffe)
#define IPECC_W_ATK_DIVMM_FACTOR_POS     (16)
#define IPECC_W_ATK_DIVMM_FACTOR_MASK    (0xfffe)

/* Fields for R_STATUS */
#define IPECC_R_STATUS_BUSY	   (((uint32_t)0x1) << 0)
#define IPECC_R_STATUS_KP	   (((uint32_t)0x1) << 4)
#define IPECC_R_STATUS_MTY	   (((uint32_t)0x1) << 5)
#define IPECC_R_STATUS_POP	   (((uint32_t)0x1) << 6)
#define IPECC_R_STATUS_R_OR_W	   (((uint32_t)0x1) << 7)
#define IPECC_R_STATUS_INIT     (((uint32_t)0x1) << 8)
#define IPECC_R_STATUS_NNDYNACT	   (((uint32_t)0x1) << 9)
#define IPECC_R_STATUS_ENOUGH_RND_WK   (((uint32_t)0x1) << 10)
#define IPECC_R_STATUS_YES	   (((uint32_t)0x1) << 11)
#define IPECC_R_STATUS_R0_IS_NULL   (((uint32_t)0x1) << 12)
#define IPECC_R_STATUS_R1_IS_NULL   (((uint32_t)0x1) << 13)
#define IPECC_R_STATUS_TOKEN_GEN      (((uint32_t)0x1) << 14)
#define IPECC_R_STATUS_ERRID_MSK	(0xffff)
#define IPECC_R_STATUS_ERRID_POS	(16)

/* Fields for R_CAPABILITIES */
#define IPECC_R_CAPABILITIES_DBG_N_PROD   (((uint32_t)0x1) << 0)
#define IPECC_R_CAPABILITIES_SHF   (((uint32_t)0x1) << 4)
#define IPECC_R_CAPABILITIES_NNDYN   (((uint32_t)0x1) << 8)
#define IPECC_R_CAPABILITIES_W64   (((uint32_t)0x1) << 9)
#define IPECC_R_CAPABILITIES_NNMAX_MSK	(0xfffff)
#define IPECC_R_CAPABILITIES_NNMAX_POS	(12)

/* Fields for R_HW_VERSION */
#define IPECC_R_HW_VERSION_MAJOR_POS    (24)
#define IPECC_R_HW_VERSION_MAJOR_MSK    (0xff)
#define IPECC_R_HW_VERSION_MINOR_POS    (16)
#define IPECC_R_HW_VERSION_MINOR_MSK    (0xff)
#define IPECC_R_HW_VERSION_PATCH_POS    (0)
#define IPECC_R_HW_VERSION_PATCH_MSK    (0xffff)

/* Fields for R_DBG_CAPABILITIES_0 */
#define IPECC_R_DBG_CAPABILITIES_0_WW_POS    (0)
#define IPECC_R_DBG_CAPABILITIES_0_WW_MSK    (0xffffffff)

/* Fields for R_DBG_CAPABILITIES_1 */
#define IPECC_R_DBG_CAPABILITIES_1_NBOPCODES_POS    (0)
#define IPECC_R_DBG_CAPABILITIES_1_NBOPCODES_MSK    (0xffff)
#define IPECC_R_DBG_CAPABILITIES_1_OPCODE_SZ_POS    (16)
#define IPECC_R_DBG_CAPABILITIES_1_OPCODE_SZ_MSK    (0xffff)

/* Fields for R_DBG_CAPABILITIES_2 */
#define IPECC_R_DBG_CAPABILITIES_2_RAW_RAMSZ_POS    (0)
#define IPECC_R_DBG_CAPABILITIES_2_RAW_RAMSZ_MSK    (0xffff)
#define IPECC_R_DBG_CAPABILITIES_2_IRN_SHF_WIDTH_POS    (16)
#define IPECC_R_DBG_CAPABILITIES_2_IRN_SHF_WIDTH_MSK    (0xffff)

/* Fields for R_DBG_STATUS */
#define IPECC_R_DBG_STATUS_HALTED    (((uint32_t)0x1) << 0)
#define IPECC_R_DBG_STATUS_BKID_POS     (1)
#define IPECC_R_DBG_STATUS_BKID_MSK     (0x3)
#define IPECC_R_DBG_STATUS_BK_HIT     (((uint32_t)0x1) << 3)
#define IPECC_R_DBG_STATUS_PC_POS     (4)
#define IPECC_R_DBG_STATUS_PC_MSK     (0xfff)
#define IPECC_R_DBG_STATUS_STATE_POS     (28)
#define IPECC_R_DBG_STATUS_STATE_MSK     (0xf)

/* Fields for R_DBG_TIME */
#define IPECC_R_DBG_TIME_POS     (0)
#define IPECC_R_DBG_TIME_MSK     (0xffffffff)

/* Fields for R_DBG_RAWDUR */
#define IPECC_R_DBG_RAWDUR_POS     (0)
#define IPECC_R_DBG_RAWDUR_MSK     (0xffffffff)

/* Fields for R_DBG_TRNG_STATUS */
#define IPECC_R_DBG_TRNG_STATUS_RAW_FIFO_FULL		(((uint32_t)0x1) << 0)
#define IPECC_R_DBG_TRNG_STATUS_RAW_FIFO_OFFSET_MSK	(0xffffff)
#define IPECC_R_DBG_TRNG_STATUS_RAW_FIFO_OFFSET_POS	(8)

/* Fields for R_DBG_TRNG_RAW_DATA */
#define  IPECC_R_DBG_TRNG_RAW_DATA_POS    (0)
#define  IPECC_R_DBG_TRNG_RAW_DATA_MSK    (0x1)

/* Fields for R_DBG_TRNG_DIAG_MIN */
#define IPECC_R_DBG_TRNG_DIAG_MIN_POS   (0)
#define IPECC_R_DBG_TRNG_DIAG_MIN_MSK   (0xffffffff)

/* Fields for R_DBG_TRNG_DIAG_MAX */
#define IPECC_R_DBG_TRNG_DIAG_MAX_POS   (0)
#define IPECC_R_DBG_TRNG_DIAG_MAX_MSK   (0xffffffff)

/* Fields for R_DBG_TRNG_DIAG_OK */
#define IPECC_R_DBG_TRNG_DIAG_OK_POS   (0)
#define IPECC_R_DBG_TRNG_DIAG_OK_MSK   (0xffffffff)

/* Fields for R_DBG_TRNG_DIAG_STARV */
#define IPECC_R_DBG_TRNG_DIAG_STARV_POS   (0)
#define IPECC_R_DBG_TRNG_DIAG_STARV_MSK   (0xffffffff)

/* Fields for R_DBG_FP_RDATA_RDY */
#define IPECC_R_DBG_FP_RDATA_RDY_IS_READY     (((uint32_t)0x1) << 0)

/* Fields for R_DBG_EXP_FLAGS */
#define IPECC_R_DBG_EXP_FLAGS_R0Z_POS   0
#define IPECC_R_DBG_EXP_FLAGS_R1Z_POS   1
#define IPECC_R_DBG_EXP_FLAGS_KAP_POS   2
#define IPECC_R_DBG_EXP_FLAGS_KAPP_POS   3
#define IPECC_R_DBG_EXP_FLAGS_ZU_POS   4
#define IPECC_R_DBG_EXP_FLAGS_ZC_POS   5
#define IPECC_R_DBG_EXP_FLAGS_LASTSTEP_POS   6
#define IPECC_R_DBG_EXP_FLAGS_FIRSTZDBL_POS   7
#define IPECC_R_DBG_EXP_FLAGS_FIRSTZADDU_POS  8
#define IPECC_R_DBG_EXP_FLAGS_FIRST2PZ_POS   9
#define IPECC_R_DBG_EXP_FLAGS_FIRST3PZ_POS   10
#define IPECC_R_DBG_EXP_FLAGS_TORSION2_POS   11
#define IPECC_R_DBG_EXP_FLAGS_PTS_ARE_EQUAL_POS   12
#define IPECC_R_DBG_EXP_FLAGS_PTS_ARE_OPPOS_POS   13
#define IPECC_R_DBG_EXP_FLAGS_PHIMSB_POS    14
#define IPECC_R_DBG_EXP_FLAGS_KB0END_POS    15
#define IPECC_R_DBG_EXP_FLAGS_JNBBIT_POS   16
#define IPECC_R_DBG_EXP_FLAGS_JNBBIT_MSK   (0xffff)

/* Fields for R_DBG_CLK_MHZ */
#define IPECC_R_DBG_CLK_CNT_POS     (0)
#define IPECC_R_DBG_CLK_CNT_MSK     (0xffffffff)
#define IPECC_R_DBG_CLK_PRECNT      (16)

/* Fields for R_DBG_CLKMM_MHZ */
#define IPECC_R_DBG_CLKMM_CNT_POS     (0)
#define IPECC_R_DBG_CLKMM_CNT_MSK     (0xffffffff)
#define IPECC_R_DBG_CLKMM_PRECNT      (16)

/* Fields for R_DBG_HW_CONFIG_0 */
#define IPECC_R_DBG_HW_CFG0_NBDSP_POS    (0)
#define IPECC_R_DBG_HW_CFG0_NBDSP_MSK    (0x7f)
#define IPECC_R_DBG_HW_CFG0_NBMULT       (((uint32_t)0x1) << 7)
#define IPECC_R_DBG_HW_CFG0_SRAMLAT      (((uint32_t)0x1) << 8)
#define IPECC_R_DBG_HW_CFG0_ASYNC        (((uint32_t)0x1) << 9)
#define IPECC_R_DBG_HW_CFG0_SHUFFLE_POS    (10)
#define IPECC_R_DBG_HW_CFG0_SHUFFLE_MSK    (0x3)
#define IPECC_R_DBG_HW_CFG0_ZREMASK_POS    (12)
#define IPECC_R_DBG_HW_CFG0_ZREMASK_MSK    (0x3ff)
#define IPECC_R_DBG_HW_CFG0_BLINDING_POS    (22)
#define IPECC_R_DBG_HW_CFG0_BLINDING_MSK    (0x3ff)

/* Fields for R_DBG_HW_CONFIG_1 */
#define IPECC_R_DBG_HW_CFG1_NBTRNG_POS    (0)
#define IPECC_R_DBG_HW_CFG1_NBTRNG_MSK    (0xf)
#define IPECC_R_DBG_HW_CFG1_TRNGTA_POS    (4)
#define IPECC_R_DBG_HW_CFG1_TRNGTA_MSK    (0x3ff)
#define IPECC_R_DBG_HW_CFG1_SZ_RAW_POS    (16)
#define IPECC_R_DBG_HW_CFG1_SZ_RAW_MSK    (0xff)
#define IPECC_R_DBG_HW_CFG1_SZ_AXI_POS    (24)
#define IPECC_R_DBG_HW_CFG1_SZ_AXI_MSK    (0xff)

/* Fields for R_DBG_HW_CONFIG_2 */
#define IPECC_R_DBG_HW_CFG2_SZ_FPR_POS    (0)
#define IPECC_R_DBG_HW_CFG2_SZ_FPR_MSK    (0xff)
#define IPECC_R_DBG_HW_CFG2_SZ_CRV_POS    (8)
#define IPECC_R_DBG_HW_CFG2_SZ_CRV_MSK    (0xff)
#define IPECC_R_DBG_HW_CFG2_SZ_SHF_POS    (16)
#define IPECC_R_DBG_HW_CFG2_SZ_SHF_MSK    (0xff)

/* Fields for R_DBG_HW_TEST_NB */
#define IPECC_R_DBG_HW_TEST_NB_BOARD_POS  (24)
#define IPECC_R_DBG_HW_TEST_NB_BOARD_MSK  (0xff)
#define IPECC_R_DBG_HW_TEST_NB_TEST_POS   (0)
#define IPECC_R_DBG_HW_TEST_NB_TEST_MSK   (0xffffff)

/* Fields for R_DBG_HW_RANDOM */
#define IPECC_R_DBG_HW_RANDOM_POS   (0)
#define IPECC_R_DBG_HW_RANDOM_MSK   (0xffffffff)

/*************************************************************
 * Low-level macros: actions involving a direct write or read
 * to/from an IP register, along with related helper macros.
 *
 * Hereafter sorted by their target register.
 *************************************************************/

/*         A c t i o n s   i n v o l v i n g
 *  N  O  M  I  N  A  L     R  E  G  I  S  T  E  R  S  */

/*
 * Actions involving registers R_STATUS & W_CTRL
 * *********************************************
 */
/* Handling the IP busy state.
 */
#define IPECC_BUSY_WAIT() do { \
	while(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_BUSY){}; \
} while(0)
/* The following macros IPECC_IS_BUSY_* are to obtain more info, when the IP is busy,
 * on why it is busy.
 * However one should keep in mind that polling code should restrict to IPECC_BUSY_WAIT
 * to determine if previous action/job submitted to the IP is done and if the IP is
 * ready to receive next command. The following macros (all in the form IPECC_IS_BUSY_*)
 * are only provided as a way for software to get extra information on the reason why
 * the IP being busy */

/* Is the IP busy computing a [k]P?
 */
#define IPECC_IS_BUSY_KP() 	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_KP))

/* Is the IP busy computing the Montgomery constants?
 */
#define IPECC_IS_BUSY_MTY() 	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_MTY))

/* Is the IP busy computing a point operation other than [k]P?
 * (e.g addition, doubling, etc)
 */
#define IPECC_IS_BUSY_POP() 	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_POP))

/* Is the IP busy transferring a big number from/to the AXI interface
 * to/from its internal memory of big numbers?
 */
#define IPECC_IS_BUSY_R_W() 	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_R_OR_W))

/* Is the IP is in its reset/initialization process?
 */
#define IPECC_IS_BUSY_INIT() 	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_INIT))

/* Is the IP busy computing internal signals due to the refresh
 * of 'nn' main security parameter? (only with a hardware synthesized
 * with 'nn modifiable at runtime' option)
 */
#define IPECC_IS_BUSY_NNDYNACT() \
	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_NNDYNACT))

/* To know if the IP is ready to accept a new scalar (writing the scalar is a
 * particular case of writing a big number: the IP must first gather enough random
 * to mask it on-the-fly during its transfer into the IP's memory of large numbers).
 *
 * This bit is not part of the "busy" state, meaning the IP won't show a high
 * 'STATUS_BUSY' bit just because there is not enough random to mask the scalar (yet).
 *
 * Software must first check that this bit is active (1) before writing a new scalar
 * (otherwise data written by software when transmitting the scalar will be ignored,
 * and error flag 'NOT_ENOUGH_RANDOM_WK' will be set in register 'R_STATUS').
 */
#define IPECC_IS_ENOUGH_RND_WRITE_SCALAR() \
	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_ENOUGH_RND_WK))

#define IPECC_ENOUGH_WK_RANDOM_WAIT() do { \
	while(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_ENOUGH_RND_WK){}; \
} while(0)

/* Is the IP busy generating the random token?
 */
#define IPECC_IS_BUSY_GEN_TOKEN() \
	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_TOKEN_GEN))

/* Commands */
#define IPECC_EXEC_PT_KP()  (IPECC_SET_REG(IPECC_W_CTRL, IPECC_W_CTRL_PT_KP))
#define IPECC_EXEC_PT_ADD() (IPECC_SET_REG(IPECC_W_CTRL, IPECC_W_CTRL_PT_ADD))
#define IPECC_EXEC_PT_DBL() (IPECC_SET_REG(IPECC_W_CTRL, IPECC_W_CTRL_PT_DBL))
#define IPECC_EXEC_PT_CHK() (IPECC_SET_REG(IPECC_W_CTRL, IPECC_W_CTRL_PT_CHK))
#define IPECC_EXEC_PT_EQU() (IPECC_SET_REG(IPECC_W_CTRL, IPECC_W_CTRL_PT_EQU))
#define IPECC_EXEC_PT_OPP() (IPECC_SET_REG(IPECC_W_CTRL, IPECC_W_CTRL_PT_OPP))
#define IPECC_EXEC_PT_NEG() (IPECC_SET_REG(IPECC_W_CTRL, IPECC_W_CTRL_PT_NEG))

/* On curve/equality/opposition flags handling
 */
#define IPECC_GET_ONCURVE() (!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_YES))
#define IPECC_GET_EQU()     (!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_YES))
#define IPECC_GET_OPP()     (!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_YES))

/*
 * Actions involving register W_WRITE_DATA & R_READ_DATA
 * *****************************************************
 */
/* Addresses and data handling.
 */
/* Write in register W_CTRL the address of the big number to read
 * and assert the read-command bit.
 *
 * Also assert the specific bit if the number to read is the token.
 */
#define IPECC_SET_READ_ADDR(addr, token) do { \
	ip_ecc_word val = 0; \
	val |= IPECC_W_CTRL_READ_NB; \
	val |= ((token) ? IPECC_W_CTRL_RD_TOKEN : 0); \
	val |= (((addr) & IPECC_W_CTRL_NBADDR_MSK) << IPECC_W_CTRL_NBADDR_POS); \
	IPECC_SET_REG(IPECC_W_CTRL, val); \
} while(0)

/* Big numbers internal RAM memory map (by index).
 */
#define IPECC_BNUM_P		0
#define IPECC_BNUM_A		1
#define IPECC_BNUM_B		2
#define IPECC_BNUM_Q		3
/* NOTE: K and R0_X share the same index */
#define IPECC_BNUM_K		4
#define IPECC_BNUM_R0_X		4
#define IPECC_BNUM_R0_Y		5
#define IPECC_BNUM_R1_X		6
#define IPECC_BNUM_R1_Y		7

#define IPECC_READ_DATA() (IPECC_GET_REG(IPECC_R_READ_DATA))

/* Write in register W_CTRL the address of the big number to write
 * and assert the write-command bit.
 *
 * Also assert the specific bit if the number to write is the scalar.
 */
#define IPECC_SET_WRITE_ADDR(addr, scal) do { \
	ip_ecc_word val = 0; \
	val |= IPECC_W_CTRL_WRITE_NB; \
	val |= ((scal) ? IPECC_W_CTRL_WRITE_K : 0); \
	val |= ((addr & IPECC_W_CTRL_NBADDR_MSK) << IPECC_W_CTRL_NBADDR_POS); \
	IPECC_SET_REG(IPECC_W_CTRL, val); \
} while(0)

#define IPECC_WRITE_DATA(val) do { \
	IPECC_SET_REG(IPECC_W_WRITE_DATA, val); \
} while(0)

/*
 * Actions involving registers W_R[01]_NULL & R_STATUS
 * ***************************************************
 */
/* Infinity point handling with R0/R1 NULL flags.
 */
#define IPECC_GET_R0_INF() \
	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_R0_IS_NULL))
#define IPECC_GET_R1_INF() \
	(!!(IPECC_GET_REG(IPECC_R_STATUS) & IPECC_R_STATUS_R1_IS_NULL))

#define IPECC_CLEAR_R0_INF() do { \
	IPECC_SET_REG(IPECC_W_R0_NULL, IPECC_W_POINT_IS_NOT_NULL); \
} while(0)
#define IPECC_SET_R0_INF() do { \
	IPECC_SET_REG(IPECC_W_R0_NULL, IPECC_W_POINT_IS_NULL); \
} while(0)
#define IPECC_CLEAR_R1_INF() do { \
	IPECC_SET_REG(IPECC_W_R1_NULL, IPECC_W_POINT_IS_NOT_NULL); \
} while(0)
#define IPECC_SET_R1_INF() do { \
	IPECC_SET_REG(IPECC_W_R1_NULL, IPECC_W_POINT_IS_NULL); \
} while(0)

/*
 * Actions involving registers W_PRIME_SIZE & R_PRIME_SIZE
 * *******************************************************
 *
 * NN size (static and dynamic) handling.
 */
/* To get the value of 'nn' the IP is currently set with
 * (or the static value if hardware was not synthesized
 * with the 'nn modifiable at runtime' option).
 */
#define IPECC_GET_NN() \
	(((IPECC_GET_REG(IPECC_R_PRIME_SIZE)) >> IPECC_W_PRIME_SIZE_POS) \
	 & IPECC_W_PRIME_SIZE_MSK)

/* To set the value of nn (only with hardware synthesized
 * with the 'nn modifiable at runtime' option).
 */
#define IPECC_SET_NN_SIZE(sz) do { \
	IPECC_SET_REG(IPECC_W_PRIME_SIZE, \
			((sz) & IPECC_W_PRIME_SIZE_MSK) << IPECC_W_PRIME_SIZE_POS); \
} while(0)

/*
 * Actions involving register W_BLINDING
 * (blinding handling)/
 * *************************************
 */

/* To disable blinding.
 * */
#define IPECC_DISABLE_BLINDING() do { \
	IPECC_SET_REG(IPECC_W_BLINDING, 0); \
} while(0)

/* To enable & configure blinding countermeaure.
 * */
#define IPECC_SET_BLINDING_SIZE(blinding_size) do { \
	uint32_t val = 0; \
	/* Enable blinding */ \
	val |= IPECC_W_BLINDING_EN; \
	/* Configure blinding */ \
	val |= ((blinding_size & IPECC_W_BLINDING_BITS_MSK) << \
			IPECC_W_BLINDING_BITS_POS); \
	IPECC_SET_REG(IPECC_W_BLINDING, val); \
} while(0)

/*
 * Actions involving register W_SHUFFLE
 * ************************************
 */

/* Enable shuffling countermeasure.
 *
 *   - Shuffling method/algo is set statically (at synthesis time)
 *     and cannot be modified dynamically.
 *
 *   - Shuffling can always be activated if a shuffling method/algo
 *     was selected at synthesis time (even without the synthesis
 *     constraining the systematic use of shuffle).
 *     The important thing is that actions can only increase the
 *     security.
 */
#define IPECC_ENABLE_SHUFFLE() do { \
	IPECC_SET_REG(IPECC_W_SHUFFLE, IPECC_W_SHUFFLE_EN); \
} while (0)

/* Disable the shuffling countermeasure.
 *
 *   - Shuffling cannot be deactivated if it was hardware-locked
 *     at synthesis time & the IP was synthesized in production
 *     (secure) mode.
 *
 *   - If IP was synthesized in debug (unsecure) mode, shuffling
 *     can be arbitrarily enabled/disabled.
 */
#define IPECC_DISABLE_SHUFFLE() do { \
	IPECC_SET_REG(IPECC_W_SHUFFLE, IPECC_W_SHUFFLE_DIS); \
} while (0)

/*
 * Actions involving register W_ZREMASK
 * ********************************************
 */
/* To enable & configure Z-remask countermeasure.
 */
#define IPECC_ENABLE_ZREMASK(zremask_period) do { \
	uint32_t val = 0; \
	/* Enable Z-remasking */ \
	val |= IPECC_W_ZREMASK_EN; \
	/* Configure Z-remasking */ \
	val |= ((zremask_period & IPECC_W_ZREMASK_BITS_MSK) \
			<< IPECC_W_ZREMASK_BITS_POS); \
	IPECC_SET_REG(IPECC_W_ZREMASK, val); \
} while (0)

#define IPECC_DISABLE_ZREMASK() do { \
	(IPECC_SET_REG(IPECC_W_ZREMASK, IPECC_W_ZREMASK_DIS)); \
} while (0)

/*
 * Actions involving register W_TOKEN
 * (token handling)
 * **********************************
 */
/* Have the IP generate a fresh random token. */
#define IPECC_ASK_FOR_TOKEN_GENERATION() do { \
	IPECC_SET_REG(IPECC_W_TOKEN, 1); /* written value actually is indifferent */ \
} while (0)

/*
 * Actions involving register W_IRQ
 * (interrupt request handling)
 * ********************************
 */
/* Enable interrupt requests */
#define IPECC_ENABLE_IRQ() do { \
	IPECC_SET_REG(IPECC_W_IRQ, IPECC_W_IRQ_EN); \
} while (0)

/*
 * Actions using register R_STATUS & W_ERR_ACK
 * (error detection & acknowlegment)
 * *******************************************
 */
/* Definition of error bits.
 *
 * Exact same bit positions exist both in R_STATUS register
 * and in W_ERR_ACK register.
 *
 * Hence an error set (=1) by hardware in R_STATUS register
 * can always be acknowledged by the software driver by writing
 * a 1 at the exact same bit position in W_ERR_ACK register,
 * thus having hardware reset back the error (=0) in the exact
 * same bit position in R_STATUS register.
 *
 * Note however that following bit positions start at 0 and
 * hence are relative: corresponding real bit positions are
 * actually shifted by a qty IPECC_R_STATUS_ERRID_POS (both in
 * R_STATUS and W_ERR_ACK register), however higher-level of
 * the software does not need to bother with these details
 * as they are masked by macros IPECC_GET_ERROR() &
 * IPECC_ACK_ERROR() below, which perform the actual bit-
 * shift of IPECC_R_STATUS_ERRID_POS positions.
 */
#define IPECC_ERR_IN_PT_NOT_ON_CURVE	((uint32_t)0x1 << 0)
#define IPECC_ERR_OUT_PT_NOT_ON_CURVE	((uint32_t)0x1 << 1)
#define IPECC_ERR_COMP			((uint32_t)0x1 << 2)
#define IPECC_ERR_WREG_FBD 		((uint32_t)0x1 << 3)
#define IPECC_ERR_KP_FBD   		((uint32_t)0x1 << 4)
#define IPECC_ERR_NNDYN			((uint32_t)0x1 << 5)
#define IPECC_ERR_POP_FBD		((uint32_t)0x1 << 6)
#define IPECC_ERR_RDNB_FBD		((uint32_t)0x1 << 7)
#define IPECC_ERR_BLN			((uint32_t)0x1 << 8)
#define IPECC_ERR_UNKOWN_REG		((uint32_t)0x1 << 9)
#define IPECC_ERR_TOKEN		((uint32_t)0x1 << 10)
#define IPECC_ERR_SHUFFLE   ((uint32_t)0x1 << 11)
#define IPECC_ERR_ZREMASK		((uint32_t)0x1 << 12)
#define IPECC_ERR_NOT_ENOUGH_RANDOM_WK  ((uint32_t)0x1 << 13)
#define IPECC_ERR_RREG_FBD 		((uint32_t)0x1 << 14)

/* Get the complete error field of R_STATUS
 */
#define IPECC_GET_ERROR() \
	((IPECC_GET_REG(IPECC_R_STATUS) >> IPECC_R_STATUS_ERRID_POS) \
	 & IPECC_R_STATUS_ERRID_MSK)

/* To identify 'Computation' error */
#define IPECC_ERROR_IS_COMP() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_COMP))

/* To identify 'Forbidden register-write' error */
#define IPECC_ERROR_IS_WREG_FBD() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_WREG_FBD))

/* To identify 'Forbidden register-read' error */
#define IPECC_ERROR_IS_RREG_FBD() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_RREG_FBD))

/* To identify '[k]P computation not possible' error */
#define IPECC_ERROR_IS_KP_FBD() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_KP_FBD))

/* To identify 'nn value not in authorized range' error */
#define IPECC_ERROR_IS_NNDYN() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_NNDYN))

/* To identify 'Point operation (other than [k]P) not possible' error */
#define IPECC_ERROR_IS_POP_FBD() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_POP_FBD))

/* To identify 'Read large number command cannot be satisfied' error */
#define IPECC_ERROR_IS_RDNB_FBD() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_RDNB_FBD))

/* To identify 'Blinding configuration' error */
#define IPECC_ERROR_IS_BLN() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_BLN))

/* To identify 'Unknown register' error */
#define IPECC_ERROR_IS_UNKOWN_REG() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_UNKOWN_REG))

/* To identify 'Input point is not on curve' error */
#define IPECC_ERROR_IS_IN_PT_NOT_ON_CURVE \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_IN_PT_NOT_ON_CURVE))

/* To identify 'Output point is not on curve' error */
#define IPECC_ERROR_IS_OUT_PT_NOT_ON_CURVE() \
	(!!(IPECC_GET_ERROR() & IPECC_ERR_OUT_PT_NOT_ON_CURVE))

/* To acknowledge error(s) to the IP.
 */
#define IPECC_ACK_ERROR(err) \
	(IPECC_SET_REG(IPECC_W_ERR_ACK, \
	  (((err) & IPECC_R_STATUS_ERRID_MSK) << IPECC_R_STATUS_ERRID_POS)))

/*
 * Actions using register W_SMALL_SCALAR
 * *************************************
 */
/* Set small scalar size.
 */
#define IPECC_SET_SMALL_SCALAR_SIZE(sz) do { \
	IPECC_SET_REG(IPECC_W_SMALL_SCALAR, \
			(sz & IPECC_W_SMALL_SCALAR_K_MSK) << IPECC_W_SMALL_SCALAR_K_POS); \
} while (0)

/*
 * Actions using register W_SOFT_RESET
 * (soft reset handling)
 * ***********************************
 */
/* Perform a software reset */
#define IPECC_SOFT_RESET() do { \
	(IPECC_SET_REG(IPECC_W_SOFT_RESET, 1)); /* written value actually is indifferent */ \
} while (0)

/*
 * Actions using register R_CAPABILITIES
 * (Capabilities handling)
 * *************************************
 */
/* To know if the IP hardware was synthesized with
 * the option 'nn modifiable at runtime' */
#define IPECC_IS_DYNAMIC_NN_SUPPORTED() \
	(!!((IPECC_GET_REG(IPECC_R_CAPABILITIES) & IPECC_R_CAPABILITIES_NNDYN)))

/* To know if the IP hardware was synthesized with
 * the 'shuffling memory of large numbers' countermeasure.
 */
#define IPECC_IS_SHUFFLING_SUPPORTED() \
	(!!((IPECC_GET_REG(IPECC_R_CAPABILITIES) & IPECC_R_CAPABILITIES_SHF)))
#define IPECC_IS_W64() \
	(!!((IPECC_GET_REG(IPECC_R_CAPABILITIES) & IPECC_R_CAPABILITIES_W64)))

/* Returns the maximum (and default) value allowed for 'nn' parameter (if the IP was
 * synthesized with the 'nn modifiable at runtime' option) or simply the static,
 * unique value of 'nn' the IP supports (otherwise).
 */
#define IPECC_GET_NN_MAX() \
	((IPECC_GET_REG(IPECC_R_CAPABILITIES) >> IPECC_R_CAPABILITIES_NNMAX_POS) \
	 & IPECC_R_CAPABILITIES_NNMAX_MSK)

/* To know if the IP was synthesized in debug (unsecure) mode
 * or in production (secure) mode.
 */
#define IPECC_IS_DEBUG() \
	(!!(IPECC_GET_REG(IPECC_R_CAPABILITIES) & IPECC_R_CAPABILITIES_DBG_N_PROD))

#define IPECC_IS_PROD() \
	(!(IPECC_GET_REG(IPECC_R_CAPABILITIES) & IPECC_R_CAPABILITIES_DBG_N_PROD))

/* Actions using register R_HW_VERSION
 * ***********************************
 */

/* For now register R_HW_VERSION exists both in debug (unsecure) and non-debug
 * (secure, production) mode.
 * It might become a debug-only feature in future releases. */
#define IPECC_GET_MAJOR_VERSION() \
	((IPECC_GET_REG(IPECC_R_HW_VERSION) >> IPECC_R_HW_VERSION_MAJOR_POS) \
	 & IPECC_R_HW_VERSION_MAJOR_MSK)
#define IPECC_GET_MINOR_VERSION() \
	((IPECC_GET_REG(IPECC_R_HW_VERSION) >> IPECC_R_HW_VERSION_MINOR_POS) \
	 & IPECC_R_HW_VERSION_MINOR_MSK)
#define IPECC_GET_PATCH_VERSION() \
	((IPECC_GET_REG(IPECC_R_HW_VERSION) >> IPECC_R_HW_VERSION_PATCH_POS) \
	 & IPECC_R_HW_VERSION_PATCH_MSK)

/*       A c t i o n s   i n v o l v i n g
 *  D  E  B  U  G     R  E  G  I  S  T  E  R  S  */

/* Actions involving register W_DBG_HALT
 * *************************************
 */

/* To halt the IP */
#define IPECC_HALT_NOW()  do { \
	IPECC_SET_REG(IPECC_W_DBG_HALT, IPECC_W_DBG_HALT_DO_HALT); \
} while (0)

/* Actions involving register W_DBG_BKPT
 * *************************************
 */
/* Symbols below defining states of the main FSM of the IP
 * have been removed from here and replaced by the ones in
 * file ecc_states.h, which is automatically generated by
 * the Makefile in ecc_curve_iram/.
 */
#if 0
/* IP main FSM state is accessible in debug mode,
 * below are defined the corresponding state codes
 *
 * (see also macro IPECC_GET_FSM_STATE()). */
#define IPECC_DEBUG_STATE_ANY_OR_IDLE  0
#define IPECC_DEBUG_STATE_CSTMTY  1
#define IPECC_DEBUG_STATE_CHECKONCURVE  2
#define IPECC_DEBUG_STATE_BLINDINIT  3
#define IPECC_DEBUG_STATE_BLINDBIT  4
#define IPECC_DEBUG_STATE_BLINDEXIT  5
#define IPECC_DEBUG_STATE_ADPA  6
#define IPECC_DEBUG_STATE_SETUP  7
#define IPECC_DEBUG_STATE_DOUBLE  8
/* Value 9 was tied to an obsolete state
 * which has been removed. */
#define IPECC_DEBUG_STATE_ITOH  10
#define IPECC_DEBUG_STATE_ZADDU  11
#define IPECC_DEBUG_STATE_ZADDC  12
#define IPECC_DEBUG_STATE_SUBTRACTP  13
#define IPECC_DEBUG_STATE_EXIT  14
#endif

/*
 * Set a breakpoint, valid in a specific state & for a specific bit-
 * position of the scalar.
 */
#define IPECC_SET_BKPT(id, addr, nbbit, state) do { \
	IPECC_SET_REG(IPECC_W_DBG_BKPT, IPECC_W_DBG_BKPT_EN \
			| (((id) & IPECC_W_DBG_BKPT_ID_MSK) << IPECC_W_DBG_BKPT_ID_POS ) \
	    | (((addr) & IPECC_W_DBG_BKPT_ADDR_MSK) << IPECC_W_DBG_BKPT_ADDR_POS ) \
	    | (((nbbit) & IPECC_W_DBG_BKPT_NBIT_MSK ) << IPECC_W_DBG_BKPT_NBIT_POS ) \
	    | (((state) & IPECC_W_DBG_BKPT_STATE_MSK) << IPECC_W_DBG_BKPT_STATE_POS )); \
} while (0)

/*
 * Set a breakpoint, valid for any state & for any bit of the scalar.
 */
#define IPECC_SET_BREAKPOINT(id, addr) do { \
	IPECC_SET_BKPT((id), (addr), 0, IPECC_DEBUG_STATE_ANY_OR_IDLE); \
} while (0)

/* Remove a breakpoint */
#define IPECC_REMOVE_BREAKPOINT(id) do { \
	IPECC_SET_REG(IPECC_W_DBG_BKPT, IPECC_W_DBG_BKPT_DIS \
			| (((id) & IPECC_W_DBG_BKPT_ID_MSK) << IPECC_W_DBG_BKPT_ID_POS )); \
} while (0)

/* Actions involving register W_DBG_STEPS
 * **************************************
 */
/*
 * Running part of the microcode when IP is debug-halted
 * or resuming execution.
 */
#define IPECC_RUN_OPCODES(nb) do { \
	IPECC_SET_REG(IPECC_W_DBG_STEPS, IPECC_W_DBG_STEPS_RUN_NB_OP \
			| (((nb) & IPECC_W_DBG_STEPS_NB_OP_MSK) << IPECC_W_DBG_STEPS_NB_OP_POS )); \
} while (0)

#define IPECC_SINGLE_STEP() do { \
	IPECC_RUN_OPCODES(1); \
} while (0)

#define IPECC_RESUME() do { \
	IPECC_SET_REG(IPECC_W_DBG_STEPS, IPECC_W_DBG_STEPS_RESUME); \
} while (0)

/* Actions involving register W_DBG_TRIG_ACT
 * *****************************************
 */
/*
 * Arming both signal rising-edge & falling-edge triggers.
 */
#define IPECC_ARM_TRIGGER() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRIG_ACT, IPECC_W_DBG_TRIG_ACT_EN); \
} while (0)

#define IPECC_DISARM_TRIGGER() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRIG_ACT, IPECC_W_DBG_TRIG_ACT_DIS); \
} while (0)

/* Actions involving register W_DBG_TRIG_UP & W_DBG_TRIG_DOWN
 * **********************************************************
 */
/*
 * To set the time at which the trigger output signal must be raised.
 *
 * Argument 'time' is expressed in multiple of the clock cycles starting from
 * the begining of [k]P computation.
 */
#define IPECC_SET_TRIGGER_UP(time) do { \
	IPECC_SET_REG(IPECC_W_DBG_TRIG_UP, ((time) & IPECC_W_DBG_TRIG_MSK) \
			<< IPECC_W_DBG_TRIG_POS); \
} while (0)

/*
 * To set the time at which the trigger output signal must be lowered back.
 * (same remark as for IPECC_SET_TRIGGER_UP).
 */
#define IPECC_SET_TRIGGER_DOWN(time) do { \
	IPECC_SET_REG(IPECC_W_DBG_TRIG_DOWN, ((time) & IPECC_W_DBG_TRIG_MSK) \
			<< IPECC_W_DBG_TRIG_POS); \
} while (0)

/* Actions involving register W_DBG_OP_WADDR
 * *****************************************
 */
#define IPECC_SET_OPCODE_WRITE_ADDRESS(addr) do { \
	IPECC_SET_REG(IPECC_W_DBG_OP_WADDR, ((addr) & IPECC_W_DBG_OP_WADDR_MSK) \
			<< IPECC_W_DBG_OP_WADDR_POS); \
} while (0)

/* Actions involving register W_DBG_OPCODE
 * ***************************************
 */
#define IPECC_SET_OPCODE_TO_WRITE(opcode) do { \
	IPECC_SET_REG(IPECC_W_DBG_OPCODE, ((opcode) & IPECC_W_DBG_OPCODE_MSK) \
			<< IPECC_W_DBG_OPCODE_POS); \
} while (0)

/* Actions involving register W_DBG_TRNG_CFG
 * (configuration of TRNG)
 * *****************************************
 */
#define IPECC_TRNG_CONFIG(debias, ta, idlenb) do { \
	uint32_t val = 0; \
	/* Configure Von Neumann debias logic */ \
	if (debias) { \
		val |= IPECC_W_DBG_TRNG_CFG_ACTIVE_DEBIAS ; \
	} \
	val |= ((ta) & IPECC_W_DBG_TRNG_CFG_TA_MSK) \
	  << IPECC_W_DBG_TRNG_CFG_TA_POS; \
	val |= ((idlenb) & IPECC_W_DBG_TRNG_CFG_TRNG_IDLE_MSK) \
	  << IPECC_W_DBG_TRNG_CFG_TRNG_IDLE_POS; \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CFG, val); \
} while (0)

/* Actions involving register W_DBG_TRNG_RESET
 * *******************************************
 */
/* Empty the FIFO buffering raw random bits of the TRNG
 * and reset associated logic.
 */
#define IPECC_TRNG_RESET_RAW_FIFO() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_RESET, IPECC_W_DBG_TRNG_RESET_FIFO_RAW); \
} while (0)

/* Empty the FIFOs buffering internal random bits of the TRNG
 * (all channels) and reset associated logic.
 */
#define IPECC_TRNG_RESET_IRN_FIFOS() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_RESET, IPECC_W_DBG_TRNG_RESET_FIFO_IRN); \
} while (0)

/* Actions involving register W_DBG_TRNG_CTRL_POSTP
 * ************************************************
 */
/* Disable the TRNG post-processing logic that pulls bytes from the
 * raw random source.
 *
 * Note: this macro does not take action on the physical raw random source
 * which produces raw random bits into the raw random FIFO, it takes action
 * on the post-processing function that consumes these bits, thus leading,
 * after a while, to starving all downstream production of internal random
 * numbers.
 *
 * See also macros IPECC_TRNG_[EN|DIS]ABLE_RAW_FIFO_READ_PORT).
 */
#define IPECC_TRNG_DISABLE_POSTPROC() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_POSTP, \
			((uint32_t)0x1 << IPECC_W_DBG_TRNG_CTRL_POSTPROC_DISABLE_POS)); \
} while (0)

/* (Re-)enable the TRNG post-processing logic that pulls bytes from the
 * raw random source - in the IP debug mode, that logic is disabled upon reset
 * and needs to be explicitly enabled by sofware by calling this macro.
 *
 * Watchout: implicitly remove a possibly pending complete bypass of the TRNG
 * by deasserting the 'complete bypass' bit in the same register.
 */
#define IPECC_TRNG_ENABLE_POSTPROC() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_POSTP, \
			((uint32_t)0x0 << IPECC_W_DBG_TRNG_CTRL_POSTPROC_DISABLE_POS)); \
} while (0)

/* Disable the read port of the TRNG raw random FIFO.
 *
 * Note: this macro does not take action on the post-processing function 
 * that consumes bits out of the raw random FIFO, but on the raw random
 * FIFO itself. The purpose is to inhibit the read port of the FIFO 
 * used by the post-processing function, in order to guarantee that
 * software becomes the only consumer of the FIFO, and that no portion
 * of the raw data slips out of software analysis, possibly creating
 * some bias or erronous analysis.
 *
 * See also macros IPECC_TRNG_[EN|DIS]ABLE_POSTPROC().
 *
 * Watchout: implicitly remove a possibly pending complet bypass of the TRNG
 * by deasserting the 'complete bypass' bit in the same register.
 */
#define IPECC_TRNG_DISABLE_RAW_FIFO_READ_PORT() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_POSTP, \
			((uint32_t)0x1 << IPECC_W_DBG_TRNG_CTRL_RAW_DISABLE_FIFO_READ_PORT_POS)); \
} while (0)

/* (Re-)enable the read port of the TRNG raw random FIFO.
 * Watchout: implicitly remove a possibly pending complet bypass of the TRNG
 * by deasserting the 'complete bypass' bit in the same register.
 */
#define IPECC_TRNG_ENABLE_RAW_FIFO_READ_PORT() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_POSTP, \
			((uint32_t)0x0 << IPECC_W_DBG_TRNG_CTRL_RAW_DISABLE_FIFO_READ_PORT_POS)); \
} while (0)

/* Actions involving register W_DBG_TRNG_CTRL_BYPASS
 * *************************************************
 */
/* Completely bypass the TRNG physical source.
 *
 * It does not mean that the physical TRNG stops working and producing random
 * bits, it means that its output is ignored as well as the post-processing
 * function's one, and that internal random numbers become deterministic
 * constants, made of constantly the same bit value.
 *
 * Because a null value is not always desired for the large numbers served
 * to some of the random clients, software can choose the value, 0 or 1, that
 * internal random numbers will be made of when the physical true entropy
 * source is bypassed (this is the purpose of argument 'bit' of the macro,
 * that should bet set to 0 or 1).
 */
#define IPECC_TRNG_COMPLETE_BYPASS(bit) do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_BYPASS, IPECC_W_DBG_TRNG_CTRL_TRNG_BYPASS \
			| (((bit) & 0x1) << IPECC_W_DBG_TRNG_CTRL_TRNG_BYPASS_VAL_POS)); \
} while (0)

/* Undo the action of IPECC_TRNG_COMPLETE_BYPASS(), restoring the
 * (nominal) unpredictable behaviour of internal random numbers.
 */
#define IPECC_TRNG_UNDO_COMPLETE_BYPASS() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_BYPASS, 0); \
} while (0)

/* Actions involving register W_DBG_TRNG_CTRL_NNRND
 * ************************************************
 */
/* Force a deterministic effect (all 1's) of the NNRND instruction
 */
#define IPECC_TRNG_NNRND_DETERMINISTIC() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_NNRND, IPECC_W_DBG_TRNG_CTRL_NNRND_DETERMINISTIC); \
} while (0);

/* Undo the action of IPECC_TRNG_NNRND_DETERMINISTIC(), restoring the
 * unpredictable behaviour of random large numbers generated by NNRND
 * instruction.
 */
#define IPECC_TRNG_NNRND_NOT_DETERMINISTIC() do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_NNRND, 0); \
} while (0);

/* Actions involving register W_DBG_TRNG_CTRL_DIAG
 * ***********************************************
 *
 * In debug mode, for each of the 4 internal random number source in the IP, as well
 * as for the RAW random source itself, the IP maintains a counter that is incremented
 * at each clock cycle where the client is requiring a fresh internal random number
 * and the TRNG actually satisfies it, providing a fresh random.
 * These are called the "OK" counters.
 *
 * Similarly, a second counter is incremented in each clock cycle where the client
 * is requiring a fresh IRN without the TRNG being able to provide it.
 * These are called the "STARV" counters.
 *
 * Hence there are 5 sources : - source of raw random bits
 *                                      (served to the post-proc unit, if any)
 *                                      The ID of that source is "RAW".
 *                             - source of internal random numbers served to ecc_axi
 *                                      (used to on-the-fly mask the scalar as it is
 *                                       transmitted by software to ecc_axi component).
 *                                      The ID of that source is "AXI".
 *                             - source of internal random numbers served to ecc_fp
 *                                      (used to implement the NNRND family of opcodes)
 *                                      The ID of that source is "EFP".
 *                             - source of internal random numbers served to ecc_curve
 *                                      (used to implement the [XY]R[01]-shuffling counter-
 *                                      measure).
 *                                      The ID of that source is "CRV".
 *                             - source of internal random numbers served to ecc_fp_dram_sh
 *                                      (used to implement the shuffling of the memory of
 *                                      large numbers).
 *                                      The ID of that source is "SHF".
 *
 * Macro IPECC_TRNG_SELECT_DIAG_ID() here below allows software to select which of the
 * different random sources the macros reading the source diagnostics will further access
 * (meaning access is multiplexed, to reduce register usage). The macros to read the
 * diagnostics (look below) are named "IPECC_R_DBG_TRNG_DIAG_[MIN|MAX|OK|STARV]".
 *
 * By computing for instance the ratio:
 *
 *                (R_DBG_TRNG_DIAG_OK / R_DBG_TRNG_DIAG_OK + R_DBG_TRNG_DIAG_STARV )
 *
 * for one of the 5 sources, the software driver can evaluate the percentage of
 * clock cycles where the random client was requesting a fresh random without the
 * TRNG actually having one at disposal to serve to it.
 *
 * Similarly, counter "MIN" (resp. "MAX") yields the value of the FIFO minimal
 * (resp. maximal) count for the random source currently selected using macro
 * IPECC_TRNG_SELECT_DIAG_ID(). This means the smallest (resp. largest) qty of
 * internal random nbs the FIFO count showed during last [k]P computation.
 *
 * The 4 different types of counter should allow the hardware designer/user of IPECC
 * to tune the TRNG parameters and FIFO sizes (which are parameterizable in
 * ecc_customize.vhd).
 *
 * Note that the different counters (MIN, MAX, OK, STARV) are automatically reset
 * at each start of a new [k]P computation.
 */
#define IPECC_TRNG_SELECT_DIAG_ID(id) do { \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_CTRL_DIAG, \
			((id) & IPECC_W_DBG_TRNG_CTRL_DIAG_MSK) << IPECC_W_DBG_TRNG_CTRL_DIAG_POS); \
} while (0);

/* Actions involving register W_DBG_TRNG_RAW_READ & R_DBG_TRNG_RAW_DATA
 * ********************************************************************
 */

/* Set address in the TRNG raw FIFO memory array where to read
 * a raw random bit from, and fetch the corresponding bit.
 */
#define IPECC_TRNG_SET_RAW_BIT_ADDR(addr) do { \
	ip_ecc_word val = 0; \
	val |= IPECC_W_DBG_TRNG_CTRL_RAWFIFO_READ; \
	val |= (((addr) & IPECC_W_DBG_TRNG_CTRL_RAWFIFO_RADDR_MSK) \
			<< IPECC_W_DBG_TRNG_CTRL_RAWFIFO_RADDR_POS); \
	IPECC_SET_REG(IPECC_W_DBG_TRNG_RAW_READ, val); \
} while (0)

/* Get the raw random bit fetched by IPECC_TRNG_SET_RAW_BIT_READ_ADDR
 * (c.f that macro above).
 * Note that for each read, first setting the address w/ IPECC_TRNG_SET_RAW_BIT_ADDR()
 * is mandatory, even if the read targets the same address, otherwise error 'ERR_RREG_FBD'
 * is raised in R_STATUS register.
 */
#define IPECC_TRNG_GET_RAW_BIT() \
	(((IPECC_GET_REG(IPECC_R_DBG_TRNG_RAW_DATA)) >> IPECC_R_DBG_TRNG_RAW_DATA_POS) \
	 & IPECC_R_DBG_TRNG_RAW_DATA_MSK)

#define IPECC_TRNG_READ_FIFO_RAW(addr, pd) do { \
	IPECC_TRNG_SET_RAW_BIT_ADDR(addr); \
	(*(pd)) = IPECC_TRNG_GET_RAW_BIT(); \
} while(0)

/* Actions involving registers W_DBG_FP_WADDR & W_DBG_FP_WDATA
 * ***********************************************************
 */

/* Set the address in the memory of large numbers at which to write a data word.
 * The data itself can be subsequently transmitted using IPECC_DBG_SET_FP_WRITE_DATA().
 */
#define IPECC_DBG_SET_FP_WRITE_ADDR(addr) do { \
	IPECC_SET_REG(IPECC_W_DBG_FP_WADDR, ((addr) & IPECC_W_DBG_FP_WADDR_MSK) \
			<< IPECC_W_DBG_FP_WADDR_POS); \
} while (0)

/* Set the data to be written in memory of large numbers, at the address previously
 * set using IPECC_DBG_SET_FP_WRITE_ADDR(), and performs the write.
 * This is a ww-bit word which is a limb of a larger large-number.
 */
#define IPECC_DBG_SET_FP_WRITE_DATA(limb) do { \
	IPECC_SET_REG(IPECC_W_DBG_FP_WDATA, ((limb) & IPECC_W_DBG_FP_DATA_MSK) \
			<< IPECC_W_DBG_FP_DATA_POS); \
} while (0)

/* Actions involving register W_DBG_FP_RADDR, R_DBG_FP_RDATA & R_DBG_FP_RDATA_RDY
 * ******************************************************************************
 */

/* Set the address in the memory of large numbers at which a data word is to be read.
 * The data itself can be subsequently obtained using IPECC_DBG_GET_FP_READ_DATA().
 */
#define IPECC_DBG_SET_FP_READ_ADDR(addr) do { \
	IPECC_SET_REG(IPECC_W_DBG_FP_RADDR, ((addr) & IPECC_W_DBG_FP_RADDR_MSK) \
			<< IPECC_W_DBG_FP_RADDR_POS); \
} while (0)

/* Polling macro to know when the data word to fecth from the memory of large numbers
 * (using previous macro IPECC_DBG_SET_FP_READ_ADDR) was actually read and hence if
 * the data can now be read, using the following macro IPECC_DBG_GET_FP_READ_DATA,
 * see below. */
#define IPECC_DBG_IS_FP_READ_DATA_AVAIL()  (!!(IPECC_GET_REG(IPECC_R_DBG_FP_RDATA_RDY) \
			& IPECC_R_DBG_FP_RDATA_RDY_IS_READY))

#define IPECC_DBG_POLL_UNTIL_FP_READ_DATA_AVAIL() do { \
	while (!(IPECC_DBG_IS_FP_READ_DATA_AVAIL())) {}; \
} while(0)

/* Obtain the data word from memory of large numbers whose address
 * was previously set using IPECC_DBG_SET_FP_READ_ADDR().
 */
#define IPECC_DBG_GET_FP_READ_DATA() \
	(((IPECC_GET_REG(IPECC_R_DBG_FP_RDATA)) >> IPECC_W_DBG_FP_DATA_POS) \
	 & IPECC_W_DBG_FP_DATA_MSK)

/* Actions involving register W_DBG_CFG_XYSHUF
 * *******************************************
 */

/* Enable the XY-coords shuffling of R0 & R1 sensitive points
 */
#define IPECC_DBG_ENABLE_XYSHUF() do { \
	IPECC_SET_REG(IPECC_W_DBG_CFG_XYSHUF, IPECC_W_DBG_CFG_XYSHUF_EN); \
} while (0)

/* Disable the XY-coords shuffling or R0 & R1 sensitive points
 * (can only by done in debug (unsecure) mode).
 */
#define IPECC_DBG_DISABLE_XYSHUF() do { \
	IPECC_SET_REG(IPECC_W_DBG_CFG_XYSHUF, IPECC_W_DBG_CFG_XYSHUF_DIS); \
} while (0)

/* Actions involving register W_DBG_CFG_AXIMSK
 * *******************************************/

/* Enable on-the-fly masking of the scalar by the AXI interface along its
 * writting in memory of large numbers.
 */
#define IPECC_DBG_ENABLE_AXIMSK() do { \
	IPECC_SET_REG(IPECC_W_DBG_CFG_AXIMSK, IPECC_W_DBG_CFG_AXIMSK_EN); \
} while (0)

/* Disable on-the-fly masking of the scalar by the AXI interface along its
 * writting in memory of large numbers.
 * (can only by done in debug (unsecure) mode).
 */
#define IPECC_DBG_DISABLE_AXIMSK() do { \
	IPECC_SET_REG(IPECC_W_DBG_CFG_AXIMSK, IPECC_W_DBG_CFG_AXIMSK_DIS); \
} while (0)

/* Actions involving register W_DBG_CFG_TOKEN
 * ******************************************
 */

/* Enable token feature - a random value used to mask the coordinates
 * of result [k]P that software driver gets before launching the [k]P
 * computation.
 */
#define IPECC_DBG_ENABLE_TOKEN() do { \
	IPECC_SET_REG(IPECC_W_DBG_CFG_TOKEN, IPECC_W_DBG_CFG_TOKEN_EN); \
} while (0)

/* Disale token feature.
 * (can only by done in debug (unsecure) mode).
 */
#define IPECC_DBG_DISABLE_TOKEN() do { \
	IPECC_SET_REG(IPECC_W_DBG_CFG_TOKEN, IPECC_W_DBG_CFG_TOKEN_DIS); \
} while (0)

/* Actions involving register W_ATTACK_CFG_0
 * *****************************************
 */

/* Set register W_ATTACK_CFG_0
 * CAUTION: calling this macro to set W_ATTACK_CFG_0 register might not be enough
 *          to set a proper level of (un)security. Also microcode might require
 *          be patched, and some of the counter-measures to be deactivated (in
 *          case they were on) or reactivated (in case they were off).
 */
#define IPECC_ATTACK_SET_HW_CFG(naive, nocollisioncr) do { \
	ip_ecc_word val = 0; \
	val |= ((naive) ? IPECC_W_ATK_NOT_ALWAYS_ADD : 0); \
	val |= ((nocollisioncr) ? IPECC_W_ATK_NO_COLLISION_CR : 0); \
	IPECC_SET_REG(IPECC_W_ATTACK_CFG_0, val); \
} while (0)

/* Enable masking of kappa & kappa' by shift-registers embedded in the hardware.
 * CAUTION: same remark as for macro IPECC_ATTACK_SET_HW_CFG()
 */
#define IPECC_ATTACK_ENABLE_NNRNDSF() do { \
	IPECC_SET_REG(IPECC_W_ATTACK_CFG_1, 0); \
} while (0)

/* Disable masking of kappa & kappa' by shift-registers embedded in the hardware.
 * CAUTION: same remark as for macro IPECC_ATTACK_SET_HW_CFG()
 */
#define IPECC_ATTACK_DISABLE_NNRNDSF() do { \
	IPECC_SET_REG(IPECC_W_ATTACK_CFG_1, IPECC_W_ATK_NO_NNRND_SF); \
} while (0)

/* Actions involving register W_ATTACK_CFG_2
 * *****************************************
 */
#define IPECC_ATTACK_SET_CLOCK_DIVOUT(div, divmm) do { \
	ip_ecc_word val = 0; \
	if (div != 0) { \
		val |= IPECC_W_ATK_DIV_ENABLE; \
		val |= (div & IPECC_W_ATK_DIV_FACTOR_MASK) << IPECC_W_ATK_DIV_FACTOR_POS; \
	} \
	if (divmm != 0) { \
		val |= IPECC_W_ATK_DIVMM_ENABLE; \
		val |= (divmm & IPECC_W_ATK_DIVMM_FACTOR_MASK) << IPECC_W_ATK_DIVMM_FACTOR_POS; \
	} \
	IPECC_SET_REG(IPECC_W_ATTACK_CFG_2, val); \
} while (0)

/* Actions involving register R_DBG_CAPABILITIES_0
 * ***********************************************
 */

/* Get the value of parameter 'ww'.
 *
 * Parameter 'ww' designates the bit size of the limbs that large numbers
 * are made of in large number memory.
 *
 * The software driver normally does not need to bother about this parameter,
 * otherwise in debug mode when using the macros IPECC_DBG_SET_FP_WRITE_DATA
 * & IPECC_DBG_GET_FP_READ_DATA.
 * (see these macros).
 */
#define IPECC_DBG_GET_WW() \
	(((IPECC_GET_REG(IPECC_R_DBG_CAPABILITIES_0)) >> IPECC_R_DBG_CAPABILITIES_0_WW_POS) \
		& IPECC_R_DBG_CAPABILITIES_0_WW_MSK)

/* Dynamic value of parameter 'w' can be obtained using those of
 * 'nn' and 'ww' as we simply have: w = ceil( (nn + 4) /ww ).
 */
#define IPECC_DBG_GET_W()    DIV( ((uint32_t)((IPECC_GET_NN()) + 4)) , (uint32_t)(IPECC_DBG_GET_WW()))

/* Actions involving register R_DBG_CAPABILITIES_1
 * ***********************************************
 */

/* To get the number of opcode words forming the complete footprint of
 * the microcode.
 */
#define IPECC_GET_NBOPCODES() \
	(((IPECC_GET_REG(IPECC_R_DBG_CAPABILITIES_1)) \
		>> IPECC_R_DBG_CAPABILITIES_1_NBOPCODES_POS) \
		& IPECC_R_DBG_CAPABILITIES_1_NBOPCODES_MSK)

/* To get the bitwidth of opcode words. */
#define IPECC_GET_OPCODE_SIZE() \
	(((IPECC_GET_REG(IPECC_R_DBG_CAPABILITIES_1)) \
		>> IPECC_R_DBG_CAPABILITIES_1_OPCODE_SZ_POS) \
		& IPECC_R_DBG_CAPABILITIES_1_OPCODE_SZ_MSK)

/* Actions involving register R_DBG_CAPABILITIES_2
 * ***********************************************
 */
/* To get the size (in bits) of the TRNG FIFO buffering raw random numbers. */
#define IPECC_GET_TRNG_RAW_SZ() \
	(((IPECC_GET_REG(IPECC_R_DBG_CAPABILITIES_2)) \
		>> IPECC_R_DBG_CAPABILITIES_2_RAW_RAMSZ_POS) \
		& IPECC_R_DBG_CAPABILITIES_2_RAW_RAMSZ_MSK)

/* To get the bitwidth of TRNG internal random numbers
 * served to the logic implementing the shuffling counter-
 * measure (shuffling of the memory of large numbers).
 *
 * The bitwidth is static and depends on the algorithm/
 * method used to shuffle the memory.
 */
#define IPECC_GET_TRNG_IRN_SHF_BITWIDTH() \
	(((IPECC_GET_REG(IPECC_R_DBG_CAPABILITIES_2)) \
		>> IPECC_R_DBG_CAPABILITIES_2_IRN_SHF_WIDTH_POS) \
		& IPECC_R_DBG_CAPABILITIES_2_IRN_SHF_WIDTH_MSK)

/* Actions involving register R_DBG_STATUS
 * ***************************************
 */
/* Is IP currently halted? (on a breakpoint hit, or after having
 * been asked to run a certain nb of microcode opcodes).
 */
#define IPECC_IS_IP_DEBUG_HALTED() \
	(!!(IPECC_GET_REG(IPECC_R_DBG_STATUS) & IPECC_R_DBG_STATUS_HALTED))

/* To poll the IP until its debug state shows it is halted.
 */
#define IPECC_POLL_UNTIL_DEBUG_HALTED() do { \
	while (!(IPECC_IS_IP_DEBUG_HALTED())) {}; \
} while(0)

/* Did IP was halted on a breakpoint hit? */
#define IPECC_IS_IP_DEBUG_HALTED_ON_BKPT_HIT() \
	(!!(IPECC_GET_REG(IPECC_R_DBG_STATUS) & IPECC_R_DBG_STATUS_BK_HIT))

/* Get the 'breakpoint ID' field in R_DBG_STATUS register.
 * If IPECC_IS_IP_DEBUG_HALTED_ON_BKPT_HIT() confirms that the IP
 * is halted due to a breakpoint hit, then this field gives
 * the ID of that breakpoint.
 */
#define IPECC_GET_BKPT_ID_IP_IS_HALTED_ON() \
	(((IPECC_GET_REG(IPECC_R_DBG_STATUS)) \
		>> IPECC_R_DBG_STATUS_BKID_POS) & IPECC_R_DBG_STATUS_BKID_MSK)

/* Get the current value of PC (program counter).
 * This is the value of the decode stage of the pipeline;
 * hence the opcode that address is pointing to has not
 * been executed yet.
 */
#define IPECC_GET_PC() \
	(((IPECC_GET_REG(IPECC_R_DBG_STATUS)) \
		>> IPECC_R_DBG_STATUS_PC_POS) & IPECC_R_DBG_STATUS_PC_MSK)

/* Get the ID of the state the main FSM is currently in.
 * (see also macros related to register W_DBG_BKPT, namely
 * IPECC_SET_BKPT & IPECC_SET_BREAKPOINT).
 */
#define IPECC_GET_FSM_STATE() \
	(((IPECC_GET_REG(IPECC_R_DBG_STATUS)) >> \
		IPECC_R_DBG_STATUS_STATE_POS) & IPECC_R_DBG_STATUS_STATE_MSK)

/* Actions involving register R_DBG_TIME
 * *************************************
 */

/* To get value of point-operation time counter.
 *
 * Each time a point-based operation is started (including [k]P
 * computation) an internal counter is started and incremented at
 * each cycle of the main clock. Reading this counter allows to
 * measure computation duration of point operations.
 */
#define IPECC_GET_PT_OP_TIME() \
	(((IPECC_GET_REG(IPECC_R_DBG_TIME)) >> IPECC_R_DBG_TIME_POS) & IPECC_R_DBG_TIME_MSK)

/* Actions involving register R_DBG_RAWDUR
 * ***************************************
 */

/* To get the duration it took to fill-up the TRNG raw random FIFO.
 *
 * In debug mode, after each hard/soft/debug reset, an internal
 * counter is started and incremented at each cycle of the main
 * clock. It is then stopped as soon as the TRNG raw random FIFO
 * becomes FULL.
 *
 * This allows any debug software driver to know the time it takes
 * to completely fill up the FIFO, and hence to estimate the random
 * production throughput of the TRNG main entropy source.
 *
 * Warning: this requires to first disable the post-processing
 * logic in the TRNG (which otherwise constantly empties the raw
 * random FIFO) using macro IPECC_TRNG_DISABLE_POSTPROC() (c.f
 * that macro).
 */
#define IPECC_GET_TRNG_RAW_FIFO_FILLUP_TIME() \
	(((IPECC_GET_REG(IPECC_R_DBG_TRNG_RAWDUR)) >> IPECC_R_DBG_RAWDUR_POS) & IPECC_R_DBG_RAWDUR_MSK)

/* Actions involving register R_DBG_TRNG_STATUS
 * ********************************************
 */

/* Returns the current value of the write-pointer into the TRNG raw random FIFO
 *
 * If post-processing is disabled (see macro IPECC_TRNG_DISABLE_POSTPROC)
 * and no TRNG raw random bits were read (using macros IPECC_TRNG_SET_RAW_BIT_ADDR
 * and IPECC_TRNG_GET_RAW_BIT) then this yields the current quantity of TRNG raw
 * random bits that have been produced since last reset. */
#define IPECC_GET_TRNG_RAW_FIFO_WRITE_POINTER() \
	(((IPECC_GET_REG(IPECC_R_DBG_TRNG_STATUS)) >> IPECC_R_DBG_TRNG_STATUS_RAW_FIFO_OFFSET_POS) \
	 & IPECC_R_DBG_TRNG_STATUS_RAW_FIFO_OFFSET_MSK)

/* Gives the FULL/not-FULL state of TRNG raw random FIFO */
#define IPECC_IS_TRNG_RAW_FIFO_FULL() \
	(!!(IPECC_GET_REG(IPECC_R_DBG_TRNG_STATUS) & IPECC_R_DBG_TRNG_STATUS_RAW_FIFO_FULL))

/* Poll until the TRNG raw random FIFO is full */
#define IPECC_TRNG_RAW_FIFO_FULL_BUSY_WAIT() do { \
        while(!IPECC_IS_TRNG_RAW_FIFO_FULL()){}; \
} while(0)

/* Note about following macros IPECC_R_DBG_TRNG_DIAG_[MIN|MAX|OK|STARV]
 *
 *     These macros acess diagnostic counters and allow detecting possible
 *     starvations of TRNG internal numbers during [k]P computations,
 *     so as to allow hardware designer to determine the smallest appropriate
 *     size of IRN fifos inside the IP (these sizes are customizable in
 *     ecc_customize.vhd).
 *
 *   See also macro  IPECC_TRNG_SELECT_DIAG_ID().
 */

/* Actions involving register R_DBG_TRNG_DIAG_MIN
 * **********************************************
 *
 * Note: random source should be prior selected using IPECC_TRNG_SELECT_DIAG_ID()
 * (see above).
 */
#define IPECC_GET_TRNG_DIAG_MIN() \
	((IPECC_GET_REG(IPECC_R_DBG_TRNG_DIAG_MIN) >> IPECC_R_DBG_TRNG_DIAG_MIN_POS) \
	 & IPECC_R_DBG_TRNG_DIAG_MIN_MSK)

/* Actions involving register R_DBG_TRNG_DIAG_MAX
 * **********************************************
 *
 * Note: random source should be prior selected using IPECC_TRNG_SELECT_DIAG_ID()
 * (see above).
 */
#define IPECC_GET_TRNG_DIAG_MAX() \
	((IPECC_GET_REG(IPECC_R_DBG_TRNG_DIAG_MAX) >> IPECC_R_DBG_TRNG_DIAG_MAX_POS) \
	 & IPECC_R_DBG_TRNG_DIAG_MAX_MSK)

/* Actions involving register R_DBG_TRNG_DIAG_OK
 * *********************************************
 *
 * Note: random source should be prior selected using IPECC_TRNG_SELECT_DIAG_ID()
 * (see above).
 */
#define IPECC_GET_TRNG_DIAG_OK() \
	((IPECC_GET_REG(IPECC_R_DBG_TRNG_DIAG_OK) >> IPECC_R_DBG_TRNG_DIAG_OK_POS) \
	 & IPECC_R_DBG_TRNG_DIAG_OK_MSK)

/* Actions involving register R_DBG_TRNG_DIAG_STARV
 * ************************************************
 *
 * Note: random source should be prior selected using IPECC_TRNG_SELECT_DIAG_ID()
 * (see above).
 */
#define IPECC_GET_TRNG_DIAG_STARV() \
	((IPECC_GET_REG(IPECC_R_DBG_TRNG_DIAG_STARV) >> IPECC_R_DBG_TRNG_DIAG_STARV_POS) \
	 & IPECC_R_DBG_TRNG_DIAG_STARV_MSK)

/* Actions involving register R_DBG_CLK_MHZ
 * ****************************************
 */
#define IPECC_GET_CLK_MHZ() \
	((IPECC_GET_REG(IPECC_R_DBG_CLK_MHZ) >> IPECC_R_DBG_CLK_CNT_POS) \
	 & IPECC_R_DBG_CLK_CNT_MSK)

/* Actions involving register R_DBG_CLKMM_MHZ
 * ******************************************
 */
#define IPECC_GET_CLKMM_MHZ() \
	((IPECC_GET_REG(IPECC_R_DBG_CLKMM_MHZ) >> IPECC_R_DBG_CLKMM_CNT_POS) \
	 & IPECC_R_DBG_CLKMM_CNT_MSK)

/* Actions involving register R_DBG_HW_CONFIG_0
 * ********************************************
 */
#define IPECC_GET_HW_CFG_NBDSP() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_0) >> IPECC_R_DBG_HW_CFG0_NBDSP_POS) \
	 & IPECC_R_DBG_HW_CFG0_NBDSP_MSK)

#define IPECC_GET_HW_CFG_NBMULT() \
	(!!(IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_0) & IPECC_R_DBG_HW_CFG0_NBMULT))

#define IPECC_GET_HW_CFG_SRAMLAT() \
	(!!(IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_0) & IPECC_R_DBG_HW_CFG0_SRAMLAT))

#define IPECC_GET_HW_CFG_ASYNC() \
	(!!(IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_0) & IPECC_R_DBG_HW_CFG0_ASYNC))

#define IPECC_GET_HW_CFG_SHUFFLE() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_0) >> IPECC_R_DBG_HW_CFG0_SHUFFLE_POS) \
	 & IPECC_R_DBG_HW_CFG0_SHUFFLE_MSK)

#define IPECC_GET_HW_CFG_ZREMASK() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_0) >> IPECC_R_DBG_HW_CFG0_ZREMASK_POS) \
	 & IPECC_R_DBG_HW_CFG0_ZREMASK_MSK)

#define IPECC_GET_HW_CFG_BLINDING() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_0) >> IPECC_R_DBG_HW_CFG0_BLINDING_POS) \
	 & IPECC_R_DBG_HW_CFG0_BLINDING_MSK)

/* Actions involving register R_DBG_HW_CONFIG_1
 * ********************************************
 */
#define IPECC_GET_HW_CFG_NBTRNG() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_1) >> IPECC_R_DBG_HW_CFG1_NBTRNG_POS) \
	 & IPECC_R_DBG_HW_CFG1_NBTRNG_MSK)

#define IPECC_GET_HW_CFG_TRNGTA() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_1) >> IPECC_R_DBG_HW_CFG1_TRNGTA_POS) \
	 & IPECC_R_DBG_HW_CFG1_TRNGTA_MSK)

#define IPECC_GET_HW_CFG_SZ_RAW() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_1) >> IPECC_R_DBG_HW_CFG1_SZ_RAW_POS) \
	 & IPECC_R_DBG_HW_CFG1_SZ_RAW_MSK)

#define IPECC_GET_HW_CFG_SZ_AXI() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_1) >> IPECC_R_DBG_HW_CFG1_SZ_AXI_POS) \
	 & IPECC_R_DBG_HW_CFG1_SZ_AXI_MSK)

/* Actions involving register R_DBG_HW_CONFIG_2
 * ********************************************
 */
#define IPECC_GET_HW_CFG_SZ_FPR() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_2) >> IPECC_R_DBG_HW_CFG2_SZ_FPR_POS) \
	 & IPECC_R_DBG_HW_CFG2_SZ_FPR_MSK)

#define IPECC_GET_HW_CFG_SZ_CRV() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_2) >> IPECC_R_DBG_HW_CFG2_SZ_CRV_POS) \
	 & IPECC_R_DBG_HW_CFG2_SZ_CRV_MSK)

#define IPECC_GET_HW_CFG_SZ_SHF() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_CONFIG_2) >> IPECC_R_DBG_HW_CFG2_SZ_SHF_POS) \
	 & IPECC_R_DBG_HW_CFG2_SZ_SHF_MSK)

/* Actions involving register R_DBG_HW_TEST_NB
 * *******************************************
 */
#define IPECC_GET_HW_TEST_BOARD() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_TEST_NB) >> IPECC_R_DBG_HW_TEST_NB_BOARD_POS) \
	 & IPECC_R_DBG_HW_TEST_NB_BOARD_MSK)

#define IPECC_GET_HW_TEST_TEST() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_TEST_NB) >> IPECC_R_DBG_HW_TEST_NB_TEST_POS) \
	 & IPECC_R_DBG_HW_TEST_NB_TEST_MSK)

/* Actions involving register R_DBG_HW_RANDOM
 * ******************************************
 */
#define IPECC_GET_HW_RANDOM() \
	((IPECC_GET_REG(IPECC_R_DBG_HW_RANDOM) >> IPECC_R_DBG_HW_RANDOM_POS) \
	 & IPECC_R_DBG_HW_RANDOM_MSK)


/*
 * The pseudo TRNG device (debug mode only)
 */

/* Actions using register IPECC_PSEUDOTRNG_W_SOFT_RESET
 * **************************************************
 */
/* Perform a software reset */
#define IPECC_PSEUDOTRNG_SOFT_RESET() do { \
	(IPECC_SET_REG(IPECC_PSEUDOTRNG_W_SOFT_RESET, 1)); /* written value actually is indifferent */ \
} while (0)

/* Push a data word into the FIFO of the pseudo TRNG device */
#define IPECC_PSEUDOTRNG_PUSH_DATA(data) do { \
	(IPECC_SET_REG(IPECC_PSEUDOTRNG_W_WRITE_DATA, (data))); \
} while (0)


/************************************************
 * One layer up - Middle-level macros & functions
 *
 * Hereafter sorted by category/function.
 ************************************************/

typedef enum {
	EC_HW_REG_A      = 0,
	EC_HW_REG_B      = 1,
	EC_HW_REG_P      = 2,
	EC_HW_REG_Q      = 3,
	EC_HW_REG_R0_X   = 4,
	EC_HW_REG_R0_Y   = 5,
	EC_HW_REG_R1_X   = 6,
	EC_HW_REG_R1_Y   = 7,
	EC_HW_REG_SCALAR = 8,
	EC_HW_REG_TOKEN  = 9,
} ip_ecc_register;

typedef enum {
	EC_HW_REG_READ  = 0,
	EC_HW_REG_WRITE = 1,
} ip_ecc_register_mode;

typedef uint32_t ip_ecc_error;

#if defined(WITH_EC_HW_DEBUG)
static const char *ip_ecc_error_strings[] = {
	"EC_HW_STATUS_ERR_IN_PT_NOT_ON_CURVE",
	"EC_HW_STATUS_ERR_OUT_PT_NOT_ON_CURVE",
	"EC_HW_STATUS_ERR_COMP",
	"EC_HW_STATUS_ERR_WREG_FBD",
	"EC_HW_STATUS_ERR_KP_FBD",
	"EC_HW_STATUS_ERR_NNDYN",
	"EC_HW_STATUS_ERR_POP_FBD",
	"EC_HW_STATUS_ERR_RDNB_FBD",
	"EC_HW_STATUS_ERR_BLN",
	"EC_HW_STATUS_ERR_UNKOWN_REG",
	"EC_HW_STATUS_ERR_TOKEN",
	"EC_HW_STATUS_ERR_SHUFFLE",
	"EC_HW_STATUS_ERR_ZREMASK",
	"EC_HW_STATUS_ERR_NOT_ENOUGH_RANDOM_WK",
	"EC_HW_STATUS_ERR_RREG_FBD",
};

static inline void ip_ecc_errors_print(ip_ecc_error err)
{
	uint32_t i;

	if(err){
		for(i = 0; i < 15; i++){
			if(((err >> i) & 1)){
				log_print("%s |", ip_ecc_error_strings[i]);
			}
		}
	}
	else{
		log_print("NONE");
	}
	return;
}
static inline void ip_ecc_log(const char *s)
{
	log_print("%s", s);
	/* Print our current status and error */
	log_print("Status: 0x"IPECC_WORD_FMT", Error: ", IPECC_GET_REG(IPECC_R_STATUS));
	ip_ecc_errors_print(IPECC_GET_ERROR());
	log_print("\n\r");

	return;
}
#else
/* NO DEBUG mode, empty */
static inline void ip_ecc_log(const char *s)
{
	(void)s;
	return;
}
#endif /* WITH_EC_HW_DEBUG */

/* Helper function to compute the size, in nb of words, of a big number, given its size in bytes.
 */
static inline uint32_t ip_ecc_nn_words_from_bytes_sz(uint32_t sz)
{
	uint32_t curr_word_sz = (sz / sizeof(ip_ecc_word));
	curr_word_sz = ((sz % sizeof(ip_ecc_word)) == 0) ? (curr_word_sz) : (curr_word_sz + 1);

	return curr_word_sz;
}

/* Helper function to compute the size in bytes of a big number, given its size in bits.
 */
static inline uint32_t ip_ecc_nn_bytes_from_bits_sz(uint32_t sz)
{
	uint32_t curr_bytes_sz = (sz / 8);
	curr_bytes_sz = ((sz % 8) == 0) ? (curr_bytes_sz) : (curr_bytes_sz + 1);

	return curr_bytes_sz;
}

/* Check for an error and return the error code */
static inline int ip_ecc_check_error(ip_ecc_error *out)
{
	int ret = 0;
	ip_ecc_error err;

	err = (ip_ecc_error)IPECC_GET_ERROR();

	if(out != NULL){
		(*out) = err;
	}
	if(err){
#if defined(WITH_EC_HW_DEBUG)
		printf("HW ACCEL: status: 0x"IPECC_WORD_FMT", DBG status: 0x"IPECC_WORD_FMT", got error flag 0x"IPECC_WORD_FMT":", IPECC_GET_REG(IPECC_R_STATUS), IPECC_GET_REG(IPECC_R_DBG_STATUS), err);
		ip_ecc_errors_print(err);
		printf("\n\r");
#endif
		ret = -1;
		/* Ack the errors */
		IPECC_ACK_ERROR(err);
	}

	return ret;
}

/* Select a register for R/W */
static inline int ip_ecc_select_reg(ip_ecc_register r, ip_ecc_register_mode rw)
{
	uint32_t addr = 0, scal = 0, token = 0;

	switch(r){
		case EC_HW_REG_A:{
			addr = IPECC_BNUM_A;
			break;
		}
		case EC_HW_REG_B:{
			addr = IPECC_BNUM_B;
			break;
		}
		case EC_HW_REG_P:{
			addr = IPECC_BNUM_P;
			break;
		}
		case EC_HW_REG_Q:{
			addr = IPECC_BNUM_Q;
			break;
		}
		case EC_HW_REG_R0_X:{
			addr = IPECC_BNUM_R0_X;
			break;
		}
		case EC_HW_REG_R0_Y:{
			addr = IPECC_BNUM_R0_Y;
			break;
		}
		case EC_HW_REG_R1_X:{
			addr = IPECC_BNUM_R1_X;
			break;
		}
		case EC_HW_REG_R1_Y:{
			addr = IPECC_BNUM_R1_Y;
			break;
		}
		case EC_HW_REG_SCALAR:{
			addr = IPECC_BNUM_K;
			scal = 1;
			break;
		}
		case EC_HW_REG_TOKEN:{
			addr = 0; /* value actually does not matter */
			token = 1;
			break;
		}
		default:{
			goto err;
		}
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	switch(rw){
		case EC_HW_REG_READ:{
			IPECC_SET_READ_ADDR(addr, token);
			break;
		}
		case EC_HW_REG_WRITE:{
			IPECC_SET_WRITE_ADDR(addr, scal);
			break;
		}
		default:{
			goto err;
		}
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Push a word to a given register */
static inline int ip_ecc_push_word(const ip_ecc_word *w)
{
	if(w == NULL){
		goto err;
	}
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	IPECC_WRITE_DATA((*w));

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Pop a word from a given register */
static inline int ip_ecc_pop_word(ip_ecc_word *w)
{
	if(w == NULL){
		goto err;
	}
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	(*w) = IPECC_READ_DATA();

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Set the NN size provided in bits */
static inline int ip_ecc_set_nn_bit_size(uint32_t bit_sz)
{
	/* Get the maximum NN size and check the asked size */
	if(bit_sz > IPECC_GET_NN_MAX()){
		/* If we overflow, this is an error */
		goto err;
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* NOTE: when NN dynamic is not supported we leave
	 * our inherent maximum size.
	 */
	if(IPECC_IS_DYNAMIC_NN_SUPPORTED()){
		/* Set the current dynamic value */
		IPECC_SET_NN_SIZE(bit_sz);
		/* Wait until the IP is not busy */
		IPECC_BUSY_WAIT();

		/* Check for error */
		if(ip_ecc_check_error(NULL)){
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/* Get the current dynamic NN size in bits */
static inline uint32_t ip_ecc_get_nn_bit_size(void)
{
	/* Size is in bits */
	if(IPECC_IS_DYNAMIC_NN_SUPPORTED()){
		return (uint32_t)IPECC_GET_NN();
	}
	else{
		return (uint32_t)IPECC_GET_NN_MAX();
	}
	/*
	 * Note: a sole use of IPECC_GET_NN() could also work as this
	 * macro also returns the NN_MAX size when the 'dynamic nn' feature is
	 * not supported.
	 */
}

/* Set the blinding size for scalar multiplication.
 *
 * A value of 0 for input argument 'blinding_size' means disabling
 * the blinding countermeasure.
 */
static inline int ip_ecc_enable_blinding_and_set_size(uint32_t blinding_size)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	if(blinding_size == 0){
		/* Clear the blinding */
		IPECC_DISABLE_BLINDING();
	}
	else{
		/* Set the blinding size and enable the countermeasure. */
		IPECC_SET_BLINDING_SIZE(blinding_size);
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable the blinding for scalar multiplication.
 */
static inline int ip_ecc_disable_blinding(void)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Disable the blinding */
	IPECC_DISABLE_BLINDING();

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Activate the shuffling for scalar multiplication.
 */
static inline int ip_ecc_enable_shuffling(void)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Enable shuffling but only if it's supported (otherwise raise an error) */
	if(IPECC_IS_SHUFFLING_SUPPORTED()){
		IPECC_ENABLE_SHUFFLE();

		/* Wait until the IP is not busy */
		IPECC_BUSY_WAIT();

		/* Check for error */
		if(ip_ecc_check_error(NULL)){
			goto err;
		}
	} else {
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable the shuffling for scalar multiplication.
 */
static inline int ip_ecc_disable_shuffling(void)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Disable shuffling */
	IPECC_DISABLE_SHUFFLE();

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Set the period of the Z-remask countermeasure for scalar multiplication.
 */
static inline int ip_ecc_enable_zremask_and_set_period(uint32_t period)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	if(period == 0){
		log_print("ip_ecc_enable_zremask_and_set_period(): error, a period of 0 is not supported - "
				"use ip_ecc_disable_zremask() instead to disable the countermeasure\n\r");
	}
	else{
		/* Enable the Zremask countermeasure and set its period.
		 * The low-level macro abides by the hardware API, which requires
		 * that {period + 1} be written to ZREMASK register - that's why
		 * we subtract 1 here (meaning for instance: a parameter of 1
		 * given by our caller really means a period of 1, the hardware
		 * being given the value 0 in this case which actually matches
		 * a period of 1). */
		IPECC_ENABLE_ZREMASK((period - 1));
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable the Z-remask countermeasure for scalar multiplication.
 */
static inline int ip_ecc_disable_zremask(void)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Disable the countermeasure */
	IPECC_DISABLE_ZREMASK();

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Write a big number to the IP
 *
 *   The input big number is in big-endian format, and it is sent to the IP in the
 *   endianness it expects, meaning: the numbers are little-endian in words (of 32
 *   or 64 bits) and big-endian for the bytes inside words as well as for the bits
 *   inside bytes.
 */
static inline int ip_ecc_write_bignum(const uint8_t *a, uint32_t a_sz, ip_ecc_register reg)
{
	uint32_t nn_size, curr_word_sz, words_sent, bytes_idx, j;
	uint8_t end;

	ip_ecc_word w;

	if(a == NULL){
		/* Nothing to write */
		return 0;
	}

	/* Get the current nb of words we need to send to the IP */
	nn_size = ip_ecc_nn_words_from_bytes_sz(ip_ecc_nn_bytes_from_bits_sz(ip_ecc_get_nn_bit_size()));
	/* Compute our current word size */
	curr_word_sz = ip_ecc_nn_words_from_bytes_sz(a_sz);

	if(curr_word_sz > nn_size){
		/* We overflow, this is an error! */
		goto err;
	}

	/* If the number to write to the IP is the scalar, we must first check
	 * that bit 'R_STATUS_ENOUGH_RND_WK' is actually asserted in 'R_STATUS'
	 * register, as this means the IP has gathered enough random to mask
	 * the scalar with during its transfer into its internal memory of
	 * large numbers.
	 */
	if (reg == EC_HW_REG_SCALAR)
	{
		/* Hence we poll this bit until it says we can actually write the
		 * scalar.
		 */
		IPECC_ENOUGH_WK_RANDOM_WAIT();
	}

	/* Select the write mode for the current register */
	if(ip_ecc_select_reg(reg, EC_HW_REG_WRITE)){
		goto err;
	}

	/* Send our words beginning with the last */
	words_sent = 0;
	bytes_idx = ((a_sz >= 1) ? (a_sz - 1) : 0);
	end = ((a_sz >= 1) ? 0 : 1);
	while(words_sent < nn_size){
		/* Format our words */
		w = 0;
		if(!end){
			for(j = 0; j < sizeof(w); j++){
				w |= (ip_ecc_word)(a[bytes_idx] << (8 * j));
				if(bytes_idx == 0){
					/* We have reached the end of the bytes */
					end = 1;
					break;
				}
				bytes_idx--;
			}
		}
		/* Push it to the IP */
		if(ip_ecc_push_word(&w)){
			goto err;
		}
		words_sent++;
	}

	return 0;
err:
	return -1;
}

/* Read a big number from the IP.
 *
 *   The output big number is in big-endian format, and it is read from the IP in the
 *   endianness it expects, meaning: the numbers are little-endian in words (of 32
 *   or 64 bits) and big-endian for the bytes inside words as well as for the bits
 *   inside bytes.
 */
static inline int ip_ecc_read_bignum(uint8_t *a, uint32_t a_sz, ip_ecc_register reg)
{
	uint32_t nn_size, curr_word_sz, words_received, bytes_idx, j;
	uint8_t end;

	ip_ecc_word w;

	if(a == NULL){
		/* Nothing to read */
		return 0;
	}

	/* Get the current size we need to read from the IP */
	nn_size = ip_ecc_nn_words_from_bytes_sz(ip_ecc_nn_bytes_from_bits_sz(ip_ecc_get_nn_bit_size()));
	/* Compute our current word size */
	curr_word_sz = ip_ecc_nn_words_from_bytes_sz(a_sz);

	if(curr_word_sz > nn_size){
		/* We overflow, this is an error! */
		goto err;
	}

	/* Select the read mode for the current register */
	if(ip_ecc_select_reg(reg, EC_HW_REG_READ)){
		goto err;
	}

	/* Receive our words beginning with the last */
	words_received = 0;
	bytes_idx = ((a_sz >= 1) ? (a_sz - 1) : 0);
	end = ((a_sz >= 1) ? 0 : 1);
	while(words_received < nn_size){
		/* Pop the word from the IP */
		if(ip_ecc_pop_word(&w)){
			goto err;
		}
		if(!end){
			for(j = 0; j < sizeof(w); j++){
				a[bytes_idx] = (w >> (8 * j)) & 0xff;
				if(bytes_idx == 0){
					/* We have reached the end of the bytes */
					end = 1;
					break;
				}
				bytes_idx--;
			}
		}
		words_received++;
	}

	return 0;
err:
	return -1;
}

/* Ask the IP for the generation of the random one-shot token.
 *
 * (More info in ip_ecc_get_token() header below).
 */
int ip_ecc_generate_token(void)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Write to proper register for token generation. */
	IPECC_ASK_FOR_TOKEN_GENERATION();
	
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Get from the IP a unique one-shot random token that the software
 * should later use to unmask the [k]P result of the next scalar
 * multiplication with. At the end of the next scalar multiplication,
 * the IP will whiten the coordinates of the [k]P result with this
 * token (with a simple bit-by-bit XOR) and erase the token. Thus
 * the unmasking by the software on its part will unveil the plain
 * values of the [k]P coordinates.
 *
 * This emulates some kind of secret sharing between the IP and the
 * software that only lasts the time of the scalar multiplication.
 * Obviously the "secret" is transferred as a plaintext value on the
 * bus/interconnect between the IP and the CPU, so the token just
 * constitutes an extra subsidiary countermeasure in the case where
 * the [k]P result may serve as a secret or a half-secret) e.g in
 * an ECDH exchange. (If malevolent software/agent is trying to spy
 * on the [k]P coordinates, she will have to intercept the transfers
 * between the IP and the software at both the begining and the end
 * of the scalar multiplication, given that several dozens of milli-
 * seconds may pass by in the meantime).
 *
 * The token is a large number whose bit-width is given by the current
 * value of parameter 'nn' in the IP (whether it is static or dynamic).
 *
 * Hence argument 'out_tok' should point to a buffer of size at least
 * '(ceil(nn / 8)' in bytes. The call to ip_ecc_read_bignum() will
 * enforce that the value of argument 't_sz' follows this rule.
 */
int ip_ecc_get_token(uint8_t* out_tok, uint32_t t_sz)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Ask the IP for the token generation */
	if(ip_ecc_generate_token()){
		goto err;
	}

	/* Read the "token" large number */
	if(ip_ecc_read_bignum(out_tok, t_sz, EC_HW_REG_TOKEN)){
		goto err;
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/*
 * Unmask (XOR) the provided 'in_a' large number with the provided
 * 'in_tok' large number. 
 */
int ip_ecc_unmask_with_token(const uint8_t* in_a, uint32_t a_sz, const uint8_t* in_tok,
		                         uint32_t t_sz, uint8_t* out_b, uint32_t* out_b_sz)
{
	uint32_t i;

	/* It doesn't make sense that input sizes not match. */
	if (a_sz != t_sz) {
		goto err;
	}

	for(i = 0; i < a_sz; i++){
		/* Do a simple byte-by-byte XOR */
		out_b[i] = in_a[i] ^ in_tok[i];
	}

	*out_b_sz = a_sz;

	return 0;
err:
	return -1;
}

/*
 * Clear the local copy of the token
 */
int ip_ecc_clear_token(uint8_t* tok, uint32_t t_sz)
{
	uint32_t i;

	for (i = 0; i < t_sz; i++){
		tok[i] = 0;
	}

	return 1;
}

/*
 * To know if R0 is currently the null point (aka point at infinity)
 */
static inline int ip_ecc_get_r0_inf(int *iszero)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	(*iszero) = (int)IPECC_GET_R0_INF();

	return 0;
}

/*
 * To know if R1 is currently the null point (aka point at infinity)
 */
static inline int ip_ecc_get_r1_inf(int *iszero)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	(*iszero) = (int)IPECC_GET_R1_INF();

	return 0;
}

/*
 * To set R0 as being or not being the null point (aka point at infinity).
 *
 * When R0 is set as the null point, the coordinates the IP was previously
 * holding for R0 (as e.g resulting from a previous computation) become invalid
 * and are ignored by the IP henceforth.
 *
 * The null point does not have affine coordinates, so either R0 is null and
 * the coordinates buffered in the IP for it are meaningless, or it isn't null
 * and these coordinates actually define R0.
 *
 * Note that pushing coordinates to the IP for point R0 automatically makes
 * R0 a not-null point for the IP. Hence function ip_ecc_set_r0_inf()'s
 * purpose is mainly to set R0 as the null point.
 */
static inline int ip_ecc_set_r0_inf(int val)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	switch(val){
		case 0:{
			IPECC_CLEAR_R0_INF();
			break;
		}
		case 1:{
			IPECC_SET_R0_INF();
			break;
		}
		default:{
			goto err;
		}
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/*
 * To set R1 as being or not being the null point (aka point at infinity).
 *
 * Every remark made about point R0 in function ip_ecc_set_r0_inf() (see above)
 * is also valid and apply here identically to point R1.
 */
static inline int ip_ecc_set_r1_inf(int val)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	switch(val){
		case 0:{
			IPECC_CLEAR_R1_INF();
			break;
		}
		case 1:{
			IPECC_SET_R1_INF();
			break;
		}
		default:{
			goto err;
		}
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Halt the IP - This freezes execution of microcode
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_debug_halt(void)
{
	/* we DON'T busy-wait as we assume the IP might alredy be stopped
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access */
	IPECC_HALT_NOW();

	return 0;
}

/* Set a breakpoint in the microcode.
 *
 *   Note: the breakpoint is enabled for whatever the state
 *         the IP FSM is in when the microcode hits the address.
 *         If you wish to create a breakpoint associated with
 *         a specific  state, use the macro IPECC_SET_BKPT()
 *         instead of IPECC_SET_BREAKPOINT().
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_set_breakpoint(uint32_t addr, uint32_t id)
{
	/* we DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access */
	IPECC_SET_BREAKPOINT(id, addr);

	return 0;
}

/* Remove a breakpoint in the microcode
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_remove_breakpoint(uint32_t id)
{
	/* we DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access */
	IPECC_REMOVE_BREAKPOINT(id);

	return 0;
}

/* Have IP to execute a certain nb of opcodes in microcode
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_run_opcodes(uint32_t nbops)
{
	/* we DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * INSTEAD we test that the IP is halted.
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Memory-mapped register access */
	IPECC_RUN_OPCODES(nbops);

	return 0;
err:
	return -1;
}

/* Same as ip_ecc_run_opcodes() but single step
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_single_step()
{
	/* we DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * INSTEAD we test that the IP is halted.
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Memory-mapped register access */
	IPECC_SINGLE_STEP();

	return 0;
err:
	return -1;
}

/* Resume execution of microcode
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_resume()
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. If it's not, the call
	 * will simply have no effect.
	 */

	/* Memory-mapped register access */
	IPECC_RESUME();

	return 0;
}

/* Arm trigger function
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_arm_trigger()
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the trigger to be armed.
	 */

	/* Memory-mapped register access */
	IPECC_ARM_TRIGGER();

	return 0;
}

/* Disarm (disable) trigger function
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_disarm_trigger()
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the trigger to be disabled.
	 */

	/* Memory-mapped register access */
	IPECC_DISARM_TRIGGER();

	return 0;
}

/* Set configuration of UP trigger detection
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_set_trigger_up(uint32_t time)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the trigger to be configured.
	 */

	/* Memory-mapped register access */
	IPECC_SET_TRIGGER_UP(time);

	return 0;
}

/* Set configuration of DOWN trigger detection
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_set_trigger_down(uint32_t time)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the trigger to be configured.
	 */

	/* Memory-mapped register access */
	IPECC_SET_TRIGGER_DOWN(time);

	return 0;
}

/* Patch a single opcode in the microcode.
 * 
 *   'opcode_msb' must contain the upper 32-bit part of the opcode in case
 *   size of opcodes is larger than 32-bit (and in this case 'opsz' must = 2).
 *
 *   'opcode_lsb' must contain the lower 32-bit part of the opcode in case
 *   size of the opcode is larger than 32-bit, OR the opcode word in case
 *   the size is 32-bit or less (and in this case 'opsz' must = 1).
 *
 * (should be called only in debug mode)
 */
static int ip_ecc_patch_one_opcode(uint32_t address, uint32_t opcode_msb, uint32_t opcode_lsb, uint32_t opsz)
{
	uint32_t nbopcodes_max;

	/* we DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/*
	 * Sanity check
	 */
	if ((opsz != 1) && (opsz != 2)) {
		goto err;
	}

	if (ge_pow_of_2(IPECC_GET_NBOPCODES(), &nbopcodes_max)) {
		goto err;
	}

	if (address > nbopcodes_max) {
		goto err;
	}

	/*
	 * Set opcode address in register W_DBG_OP_WADDR.
	 */
	IPECC_SET_OPCODE_WRITE_ADDRESS(address);

	/*
	 * Set opcode word in register W_DBG_OPCODE.
	 */
	if (opsz == 2) {

		/* If opcodes are larger than 32 bits, the least
		 * significant 32-bit half must be transmitted first.
		 */
		IPECC_SET_OPCODE_TO_WRITE(opcode_lsb);

		/* Wait until the IP is not busy */
		IPECC_BUSY_WAIT();

		IPECC_SET_OPCODE_TO_WRITE(opcode_msb);

	} else {

		IPECC_SET_OPCODE_TO_WRITE(opcode_lsb);
	}

	return 0;
err:
	return -1;
}

/* Patch a portion or the whole of microcode image.
 * 
 *   'buf' should point to the buffer, 'nbops' is the nb of opcodes
 *   in the buffer, 'opsz' is a flag to tell if the size of opcode
 *   words is larger than 32 bit (given that the max supported is
 *   64 bits).
 *
 * (in debug mode only)
 */
static int ip_ecc_patch_microcode(uint32_t* buf, uint32_t nbops, uint32_t opsz)
{
	uint32_t i;
	uint32_t nbopcodes_max;

	/* we DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Sanity checks.
	 *
	 * Opcodes are expected to be given in buffer 'buf' starting from
	 * address 0x0.
	 *
	 * The only allowed values for parameter 'opsz' are 1 and 2.
	 *
	 *   * 'opsz' = 1 means that opcode size is less than or equal to
	 *     32 bits.
	 *     In this case opcodes are expected to be encoded in buffer
	 *     'buf' using only one 32-bit data word for each.
	 *
	 *   * 'opsz' = 2 means that the opcode size is in the range
	 *     33 to 64 (these values included).
	 *     In this case opcodes are expected to be encoded in buffer
	 *     'buf' using exactly two 32-bit data words for each, and
	 *     the most significant 32-bit part of the opcode must be
	 *     stored first in 'buf' (hence the order is big-endian at
	 *     the 32-bit word level).
	 *
	 * Inside each 32-bit word, the expected order is also big-endian
	 * (meaning the most significant byte of each 32-bit word is the
	 * lowest address).
	 *
	 * Example: Assuming opsz = 2 with opcodes of size 33-bit - heck why not?)
	 *          and assuming the first opcodes of the microcode are:
	 *
	 *                0x 1 9100 7bfd
	 *                0x 1 9400 741d
	 *                0x 2 1100 0018
	 *                ...
	 *
	 *          then the expected content for 'buf' is:
	 *
	 *                0x00000001
	 *                0x91007bfd
	 *                0x00000001
	 *                0x9400741d
	 *                0x00000002
	 *                0x11000018
	 *                ...
	 *
	 * Parameter 'nbops' must be given in number of instruction opcodes,
	 * (not in number of 32-bit words) so depending on the value of parameter
	 * 'opsz' it should be equal to either the actual size of the buffer
	 * given in 32-bit words, or to half of it.
	 *
	 * Code below will enforce verifying that 'nbops' is in accordance
	 * with value read from live-hardware (through macro IPECC_GET_NBOPCODES())
	 * meaning that it cannot exceed the power-of-2 directly superior
	 * (or equal) to the hardware memory size, expressed in number of
	 * instruction opcodes.
	 */
	if ((opsz != 1) && (opsz != 2)) {
		goto err;
	}

	if (ge_pow_of_2(IPECC_GET_NBOPCODES(), &nbopcodes_max)) {
		goto err;
	}

	if (nbops > nbopcodes_max) {
		goto err;
	}

	for (i=0; i<nbops; i++)
	{
		/*
		 * Set opcode address in register W_DBG_OP_WADDR.
		 */
		IPECC_SET_OPCODE_WRITE_ADDRESS(i);

		/*
		 * Set opcode word in register W_DBG_OPCODE.
		 */
		if (opsz == 2) {
			/* If opcodes are larger than 32 bits, the least
			 * significant 32-bit half must be transmitted first.
			 */
			IPECC_SET_OPCODE_TO_WRITE(buf[(2*i) + 1]);
			IPECC_SET_OPCODE_TO_WRITE(buf[2*i]);

		} else {
			IPECC_SET_OPCODE_TO_WRITE(buf[i]);

		}
	}

	return 0;
err:
	return -1;
}

/* Set configuration of TRNG
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_configure_trng(int debias, uint32_t ta, uint32_t cycles)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the TRNG to be configured.
	 */

	/* Memory-mapped register access */
	/*   (TRNG configuration register) */
	IPECC_TRNG_CONFIG(debias, ta, cycles);

	return 0;
}

/* Reset the raw random bits FIFO
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_reset_trng_raw_fifo(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the raw random FIFO of TRNG to be emptied/reset.
	 */

	/* Memory-mapped register access */
	IPECC_TRNG_RESET_RAW_FIFO();

	return 0;
}

/* Reset all the internal random number FIFOs
 *
 * (should be called only in debug mode)
 */
int ip_ecc_reset_trng_irn_fifos(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the internal random nb FIFOs of TRNG to be emptied/reset.
	 */

	/* Memory-mapped register access */
	IPECC_TRNG_RESET_IRN_FIFOS();

	return 0;
}

/* Disable the TRNG post-processing logic that pulls bytes from the
 * raw random source.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_trng_postproc_disable(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for this action to take place.
	 */

	/* Memory-mapped register access */
	IPECC_TRNG_DISABLE_POSTPROC();

	return 0;
}

/* (Re-)enable the TRNG post-processing logic that pulls bytes from the
 * raw random source - in the debug mode of the IP, that logic is DISABLED
 * UPON RESET and needs to be explicitly enabled by sofware by calling
 * this macro. This particular task is enforced by driver_setup() function
 * (confer).
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_trng_postproc_enable()
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for this action to take place.
	 */

	/* Memory-mapped register access */
	IPECC_TRNG_ENABLE_POSTPROC();

	return 0;
}

/* Enable the read port of the TRNG raw random FIFO.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_enable_read_port_of_raw_fifo()
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for this action to take place.
	 */

	/* Enable the read port of the FIFO used by the post-processing. */
	IPECC_TRNG_ENABLE_RAW_FIFO_READ_PORT();

	return 0;
}

/* Disable the read port of the TRNG raw random FIFO,
 * allowing complete and exclusive access by software to the raw
 * random bits.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_disable_read_port_of_raw_fifo(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for this action to take place.
	 */

	/* Disable the read port of the FIFO used by the post-processing. */
	IPECC_TRNG_DISABLE_RAW_FIFO_READ_PORT();

	return 0;
}

/* TRNG complete bypass, also specifying deterministic bit value
 * to use instead of random bits.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_bypass_full_trng(uint32_t instead_bit)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the TRNG to be bypassed.
	 */

	/*
	 * Sanity check
	 */
	if ((instead_bit != 0) && (instead_bit != 1)){
		goto err;
	}

	/* Memory-mapped register access */
	IPECC_TRNG_COMPLETE_BYPASS(instead_bit);

	return 0;
err:
	return -1;
}

/* Have TRNG leave bypass state and return to normal generation
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_dont_bypass_trng(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for this action to take place.
	 */

	/* Memory mapped register access */
	IPECC_TRNG_UNDO_COMPLETE_BYPASS();

	return 0;
}

/* Force a deterministic effect (all 1's) of the NNRND instruction
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_trng_nnrnd_deterministic(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for this action to take place.
	 */

	/* Memory mapped register access */
	IPECC_TRNG_NNRND_DETERMINISTIC();

	return 0;
}

/* Restore nominal action of NNRND instruction, undoing action
 * of ip_ecc_trng_nnrnd_deterministic().
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_trng_nnrnd_not_deterministic(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the TRNG to be bypassed.
	 */

	/* Memory mapped register access */
	IPECC_TRNG_NNRND_NOT_DETERMINISTIC();

	return 0;
}

/* To select the random source of which to read the diagnostics
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_select_trng_diag_source(uint32_t id)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for TRNG diagnostics to be read back.
	 */

	/* Memory-mapped register access */
	IPECC_TRNG_SELECT_DIAG_ID(id);

	return 0;
}

/* Get one bit from the raw random FIFO
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_read_one_raw_random_bit(uint32_t addr, uint32_t* rawbit)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for this action to take place.
	 *
	 * HOWEVER it is strongy RECOMMENDED, before read-access the raw
	 * random FIFO, to first deactivate the pull of raw random bits
	 * by the post-processing funtion. See ip_ecc_trng_postproc_disable().
	 */

	/* Memory-mapped register access (two are required here,
	 *   1 write to set the address, 1 read to get the bit -
	 *   IPECC_TRNG_READ_FIFO_RAW perform the two at one). */
	IPECC_TRNG_READ_FIFO_RAW(addr, rawbit);

	return 0;
}

/* Write one word in the IP memory of large nbs.
 *
 *   CONFER TO hw_driver_write_word_in_lgnbmem_DBG: SAME INTERFACE APPLIES.
 *                                                  SAME LIMITATIONS APPLY.
 *
 * This function should only be used when IP was synthesized in
 * debug mode. Nominal functions to access large numbers are
 * ip_ecc_write_bignum() and ip_ecc_read_bignum().
 */
static inline int ip_ecc_write_word_in_lgnbmem(uint32_t addr, uint32_t limb)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 *
	 *     It it isn't, the access to the memory of large numbers inside
	 *     the IP can't be completed as its write data path is hold by ecc_fp
	 *     (the ALU for large numbers) as long as the microcode is running
	 *     (= as long as the IP is not debug-halted).
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Get the value of 'ww' from R_DBG_CAPABILITIES_0 */
	if (IPECC_DBG_GET_WW() > 32) {
		goto err;
	}

	/* Memory-mapped register access: to set the address */
	IPECC_DBG_SET_FP_WRITE_ADDR(addr);

	/* Memory-mapped register access: to write data. */
	IPECC_DBG_SET_FP_WRITE_DATA(limb);

	return 0;
err:
	return -1;
}

/* Write one limb of one large nb in the IP memory of large numbers.
 *
 *   CONFER TO hw_driver_write_limb_DBG: SAME INTERFACE APPLIES.
 *                                       SAME LIMITATIONS APPLY.
 *
 * This function should only be used when IP was synthesized in
 * debug mode. Nominal functions to access large numbers are
 * ip_ecc_write_bignum() and ip_ecc_read_bignum().
 */
static inline int ip_ecc_write_limb(int32_t i, uint32_t j, uint32_t limb)
{
	uint32_t w, n;

	/* Get the value of 'w' from capabilities */
	w = DIV(IPECC_GET_NN_MAX() + 4, IPECC_DBG_GET_WW());

	/* Compute n */
	if (ge_pow_of_2(w, &n)) {
		return -1;
	}

	return ip_ecc_write_word_in_lgnbmem( (i * n) + j, limb );
}

/* Write one large number in the IP memory of large numbers.
 *
 *   CONFER TO hw_driver_write_largenb_DBG: SAME INTERFACE APPLIES.
 *                                          SAME LIMITATIONS APPLY.
 *
 * This function should only be used when IP was synthesized in
 * debug mode. Nominal functions to access large numbers are
 * ip_ecc_write_bignum() and ip_ecc_read_bignum().
 */
static inline int ip_ecc_write_largenb(uint32_t i, uint32_t* limbs)
{
	uint32_t w, j;

	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 *
	 *     It it isn't, the access to the memory of large numbers inside
	 *     the IP can't be completed as its write data path is hold by ecc_fp
	 *     (the ALU for large numbers) as long as the microcode is running
	 *     (= as long as the IP is not debug-halted).
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Get the value of 'ww' from R_DBG_CAPABILITIES_0 */
	if (IPECC_DBG_GET_WW() > 32) {
		goto err;
	}

	/* Get the value of 'w' from capabilities */
	w = IPECC_DBG_GET_W();

	for (j=0; j<w; j++) {
		if (ip_ecc_write_limb(i, j, limbs[j])){
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/* Read one word from from the IP memory of large numbers.
 *
 *   CONFER TO hw_driver_read_word_from_lgnbmem_DBG: SAME INTERFACE APPLIES.
 *                                                   SAME LIMITATIONS APPLY.
 *
 * This function should only be used when IP was synthesized in
 * debug mode. Nominal functions to access large numbers are
 * ip_ecc_write_bignum() and ip_ecc_read_bignum().
 */
static inline int ip_ecc_read_word_from_lgnbmem(uint32_t addr, uint32_t* limb)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 *
	 *     It it isn't, the access to the memory of large numbers inside
	 *     the IP can't be completed as its read data path is hold by ecc_fp
	 *     (the ALU for large numbers) as long as the microcode is running
	 *     (= as long as the IP is not debug-halted).
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Get the value of 'ww' from R_DBG_CAPABILITIES_0 */
	if (IPECC_DBG_GET_WW() > 32) {
		goto err;
	}

	/* Memory-mapped register access: to set the address */
	IPECC_DBG_SET_FP_READ_ADDR(addr);

	/* Poll until the IP says the data is ready */
	IPECC_DBG_POLL_UNTIL_FP_READ_DATA_AVAIL();

	/* Memory-mapped register access: to read data. */
	*limb = IPECC_DBG_GET_FP_READ_DATA();

	return 0;
err:
	return -1;
}

/* Read one word from from the IP memory of large numbers.
 *
 *   CONFER TO hw_driver_read_limb_DBG: SAME INTERFACE APPLIES.
 *                                      SAME LIMITATIONS APPLY.
 *
 * This function should only be used when IP was synthesized in
 * debug mode. Nominal functions to access large numbers are
 * ip_ecc_write_bignum() and ip_ecc_read_bignum().
 */
static inline int ip_ecc_read_limb(int32_t i, uint32_t j, uint32_t* limb)
{
	uint32_t w, n;

	/* Get the value of 'w' from capabilities */
	w = DIV(IPECC_GET_NN_MAX() + 4, IPECC_DBG_GET_WW());

	/* Compute n */
	if (ge_pow_of_2(w, &n)) {
		return -1;
	}

	return ip_ecc_read_word_from_lgnbmem( (i * n) + j, limb );
}

/* Read one large number from the IP memory of large numbers.
 *
 *   CONFER TO hw_driver_read_largenb_DBG: SAME INTERFACE APPLIES.
 *                                         SAME LIMITATIONS APPLY.
 *
 * This function should only be used when IP was synthesized in
 * debug mode. Nominal functions to access large numbers are
 * ip_ecc_write_bignum() and ip_ecc_read_bignum().
 */
static inline int ip_ecc_read_largenb(uint32_t i, uint32_t *limbs)
{
	uint32_t w, j;

	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 *
	 *     It it isn't, the access to the memory of large numbers inside
	 *     the IP can't be completed as its write data path is hold by ecc_fp
	 *     (the ALU for large numbers) as long as the microcode is running
	 *     (= as long as the IP is not debug-halted).
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Get the value of 'ww' from R_DBG_CAPABILITIES_0 */
	if (IPECC_DBG_GET_WW() > 32) {
		goto err;
	}

	/* Get the value of 'w' from capabilities */
	w = IPECC_DBG_GET_W();

	for (j=0; j<w; j++) {
		if (ip_ecc_read_limb(i, j, &limbs[j])){
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/* Enable the XY-shuffling countermeasure.
 */
static inline int ip_ecc_enable_xyshuf(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access
	 * to enable the countermeasure. */
	IPECC_DBG_ENABLE_XYSHUF();

#if 0
	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}
#endif

	return 0;
#if 0
err:
	return -1;
#endif
}

/* Disable the XY-shuffling countermeasure
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_disable_xyshuf(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access
	 * to disable the countermeasure. */
	IPECC_DBG_DISABLE_XYSHUF();

#if 0
	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}
#endif

	return 0;
#if 0
err:
	return -1;
#endif
}

/* Enable 'on-the-fly masking of the scalar by AXI interface'
 * countermeasure.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_enable_aximsk(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access
	 * to enable the countermeasure. */
	IPECC_DBG_ENABLE_AXIMSK();

	return 0;
}

/* Disable 'on-the-fly masking of the scalar by AXI interface'
 * countermeasure.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_disable_aximsk(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access
	 * to disable the countermeasure. */
	IPECC_DBG_DISABLE_AXIMSK();

	return 0;
}

/* Enable token feature
 * (this is a feature that is on by default).
 *
 * (should be called only in debug mode)
 */
int ip_ecc_enable_token(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access */
	IPECC_DBG_ENABLE_TOKEN();

	return 0;
}

/* Disable token feature.
 *
 * (should be called only in debug mode)
 */
int ip_ecc_disable_token(void)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory mapped register access */
	IPECC_DBG_DISABLE_TOKEN();

	return 0;
}

/* To get some more capabilties than offered by ip_ecc_get_capabilities().
 *
 *     These infos are:
 *
 *         ww        the width in bits of the limbs which large nbs
 *                   are made of (in the IP memory of marge nbs).
 *
 *         nbop      the total number of instructions in the
 *                   microcode.
 *
 *         opsz      the width in bits of the opcodes of microcode.
 *
 *         rawramsz  the size in bits of the TRNG FIFO of raw
 *                   random bits.
 *
 *         irnshw    the bitwidth of the internal random numbers
 *                   which are used to implement the shuffling
 *                   countermeasure (of the IP memory of large nbs).
 *
 * (should only be called in debug mode, as some of the infos delivered here
 * might give inside knowledge of the internal features of the IP, which are
 * not at all required for its nominal (secure) usage.
 */
static inline int ip_ecc_get_more_capabilities(uint32_t* ww, uint32_t* nbop,
		uint32_t* opsz, uint32_t* rawramsz, uint32_t* irnshw)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Several memory-mapped register accesses */
	*ww = IPECC_DBG_GET_WW();
	*nbop = IPECC_GET_NBOPCODES();
	*opsz = IPECC_GET_OPCODE_SIZE();
	*rawramsz = IPECC_GET_TRNG_RAW_SZ();
	*irnshw = IPECC_GET_TRNG_IRN_SHF_BITWIDTH();

	return 0;
}

/* Is the IP currently debug-halted?
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_is_debug_halted(bool* halted)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access */
	if (IPECC_IS_IP_DEBUG_HALTED()){
		*halted = true;
	} else {
		*halted = false;
	}

	return 0;
}

/* Is the IP currently halted on a breakpoint hit?
 * (and if it is, return the breakpoint ID).
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_halted_breakpoint_hit(bool* hit, uint32_t* id)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 */

	/* Memory-mapped register access */
	if (IPECC_IS_IP_DEBUG_HALTED_ON_BKPT_HIT()){
		*hit = true;
		/* get the breakpoint ID */
		*id = IPECC_GET_BKPT_ID_IP_IS_HALTED_ON();

	} else {
		*hit = false;
	}

	return 0;
}

/* Get the value of Program Counter.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_pc(uint32_t* pc)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 *
	 *     (the PC info doesn't really have a meaning otherwise).
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Memory-mapped register access */
	*pc = IPECC_GET_PC();

	return 0;
err:
	return -1;
}

/* To get the FSM state the IP is currently in.
 *
 *   CONFER TO hw_driver_get_fsm_state: SAME INTERFACE APPLIES.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_fsm_state(char* state, uint32_t sz)
{
	uint32_t st_id;

	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 *
	 *     (the FSM state info doesn't really have a meaning otherwise).
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Memory-mapped register access */
	st_id = IPECC_GET_FSM_STATE();

	/* Copy the string identifier, using its length to enforce
	 * a minimum. */
	strncpy(state, str_ipecc_state(st_id), MIN(sz, strlen(str_ipecc_state(st_id))));
	//state[ strlen(str_ipecc_state(st_id)) ] = '\0';

	return 0;
err:
	return -1;
}

/* To get value of point-operation time counter.
 *
 *     Each time a point-based operation is started (including [k]P
 *     computation) an internal counter is started and incremented at
 *     each cycle of the main clock. Reading this counter allows to
 *     measure computation duration of point operations.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_time(uint32_t* clk_cycles)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * BUT we test that the IP is halted!
	 *
	 *     (the time-counter info doesn't really have a meaning
	 *     otherwise).
	 */
	if (!IPECC_IS_IP_DEBUG_HALTED()) {
		goto err;
	}

	/* Memory-mapped register access */
	*clk_cycles = IPECC_GET_PT_OP_TIME();

	return 0;
err:
	return -1;
}

/* To assess the production throughput of TRNG raw random bits.
 *
 *     This function measures the time it takes to fill-up
 *     the RAW random FIFO of the TRNG. It first disables
 *     the post-processing function of the TRNG (so that not to
 *     have any read empyting the FIFO), then reset (empty) it,
 *     then poll debug status until it shows that the FIFO is
 *     full. When it is, the value in register R_DBG_RAWDUR
 *     yiels the nb of cycles (of the main clock) the filling-up
 *     process took.
 *
 * It is strongly recommended to first debug-halt the IP or to wait
 * until it is in an idle state.
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_trng_raw_fifo_filling_time(uint32_t* duration)
{
	uint32_t watchdog = 0;
	bool timeout = true;

	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 */

	/* Step 1/5: disable the post-processing pull of raw random bits */
	IPECC_TRNG_DISABLE_POSTPROC();

	/* Step 2/5: reset the TRNG raw FIFO */
	IPECC_TRNG_RESET_RAW_FIFO();

	/* Step 3/5: Poll until the IP shows raw FIFO is full */
	//IPECC_TRNG_RAW_FIFO_FULL_BUSY_WAIT();
	while (watchdog < 0x1000000U) /* quite arbitrary */
	{
		if (IPECC_IS_TRNG_RAW_FIFO_FULL()) {
			timeout = false;
			break;
		}
		watchdog++;
	}
	if (timeout) {
		goto err;
	}

	/* Step 4/5: Read-back duration value from register */
	*duration = IPECC_GET_TRNG_RAW_FIFO_FILLUP_TIME();

	/* Step 5/5: re-enable post-processing function */
	IPECC_TRNG_ENABLE_POSTPROC();

	return 0;

err:
	return -1;
}

/* Returns the state of the TRNG FIFO of raw random bits.
 *
 *   *full   = true is the FIFO is full
 *   *nbbits = nb of raw random bits currently
 *             buffered in the FIFO.
 *
 * It is recommended to wait for the IP to be in idle state
 * (no running computation) or to be debug-halted before
 * calling the function.
 *
 * (should be called only in debug mode)
 */
int ip_ecc_get_trng_raw_fifo_state(bool* full, uint32_t* nbbits)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the TRNG FIFO state to have meaning. But see header
	 * above: better to wait for IP to be idle or debug-halted.
	 */

	/* Memory-mapped register access */
	if (IPECC_IS_TRNG_RAW_FIFO_FULL()){
		*full = true;
	} else {
		*full = false;
	}

	/* Memory-mapped register access */
	*nbbits = IPECC_GET_TRNG_RAW_FIFO_WRITE_POINTER();

	return 0;
}

/* Read the whole content of the TRNG FIFO of raw random bits
 *
 *   'buf' must point to a buffer whose allocated size must
 *   be sufficient to store the whole content of the FIFO,
 *   the size of which shall be obtained through a call to
 *   function hw_driver_get_more_capabilities_DBG()
 *   (see that function).
 *
 *   Bits are regrouped into bytes before being written
 *   into 'buf' hence the minimal size of the 'buf' is 1/8th
 *   of the size of the FIFO as returned by function
 *   hw_driver_get_more_capabilities_DBG().
 *
 *   The function starts by clearing (zeroing) 'buf',
 *   assuming an allocated size as described just above.
 *
 *   Upon completion, the nb of bits actually read from the
 *   FIFO is returned in *nbbits.
 *
 * The function temporarily disables the read port of the FIFO.
 * Hence it is strongly advised to wait for the IP to be in idle
 * state (no computation) or debug-halted before calling the
 * funtion.
 *
 * (should be called only in debug mode)
 */
int ip_ecc_get_content_of_trng_raw_random_fifo(char* buf, uint32_t* nbbits)
{
	uint32_t i, qty, ffsz, nb;
	uint8_t bit;

	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for the TRNG FIFO state to have meaning. But see header
	 * above: better to wait for IP to be idle or debug-halted.
	 */

	/* Step 1/ : disable the post-processing unit
	 */
	IPECC_TRNG_DISABLE_RAW_FIFO_READ_PORT();

	/* Step 2/ : Get the current size of the FIFO (nb of written bits)
	 *           This is always a power-of-2.
	 */
	ffsz = IPECC_GET_TRNG_RAW_SZ();

	/* Step 3/ : Clear the buffer */
	memset(buf, 0, ffsz/8);

	/* Step 4/ : Get the current size of the FIFO (nb of written bits)
	 */
	qty = IPECC_GET_TRNG_RAW_FIFO_WRITE_POINTER();

	/* Step 5/ : loop on the nb of bits
	 */
	for (i=0, nb=0 ; i<qty ; i++) {
		/* set debug read address */
		IPECC_TRNG_SET_RAW_BIT_ADDR(i);

		/* read bit */
		bit = (uint8_t)(IPECC_TRNG_GET_RAW_BIT());
		buf[i / 8] |= ( bit << (i % 8) );
		nb++;
	}
	*nbbits = nb;

	/* Step / : enable back the read port of the FIFO.
	 */
	IPECC_TRNG_ENABLE_RAW_FIFO_READ_PORT();

	return 0;
}

/* To get the diagnostic counters for random source "AXI"
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_trng_diag_axi(uint32_t* min, uint32_t* max, uint32_t* ok, uint32_t* starv)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for TRNG diagnostics to be read back.
	 */

	/* First select the random source */
	IPECC_TRNG_SELECT_DIAG_ID(IPECC_W_DBG_TRNG_CTRL_DIAG_AXI);

	/* Memory-mapped register accesses */
	*min = IPECC_GET_TRNG_DIAG_MIN();
	*max = IPECC_GET_TRNG_DIAG_MAX();
	*ok = IPECC_GET_TRNG_DIAG_OK();
	*starv = IPECC_GET_TRNG_DIAG_STARV();

	return 0;
}

/* To get the diagnostic counters for random source "EFP"
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_trng_diag_efp(uint32_t* min, uint32_t* max, uint32_t* ok, uint32_t* starv)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for TRNG diagnostics to be read back.
	 */

	/* First select the random source */
	IPECC_TRNG_SELECT_DIAG_ID(IPECC_W_DBG_TRNG_CTRL_DIAG_EFP);

	/* Memory-mapped register accesses */
	*min = IPECC_GET_TRNG_DIAG_MIN();
	*max = IPECC_GET_TRNG_DIAG_MAX();
	*ok = IPECC_GET_TRNG_DIAG_OK();
	*starv = IPECC_GET_TRNG_DIAG_STARV();

	return 0;
}

/* To get the diagnostic counters for random source "CRV"
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_trng_diag_crv(uint32_t* min, uint32_t* max, uint32_t* ok, uint32_t* starv)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for TRNG diagnostics to be read back.
	 */

	/* First select the random source */
	IPECC_TRNG_SELECT_DIAG_ID(IPECC_W_DBG_TRNG_CTRL_DIAG_CRV);

	/* Memory-mapped register accesses */
	*min = IPECC_GET_TRNG_DIAG_MIN();
	*max = IPECC_GET_TRNG_DIAG_MAX();
	*ok = IPECC_GET_TRNG_DIAG_OK();
	*starv = IPECC_GET_TRNG_DIAG_STARV();

	return 0;
}

/* To get the diagnostic counters for random source "SHF"
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_trng_diag_shf(uint32_t* min, uint32_t* max, uint32_t* ok, uint32_t* starv)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for TRNG diagnostics to be read back.
	 */

	/* First select the random source */
	IPECC_TRNG_SELECT_DIAG_ID(IPECC_W_DBG_TRNG_CTRL_DIAG_SHF);

	/* Memory-mapped register accesses */
	*min = IPECC_GET_TRNG_DIAG_MIN();
	*max = IPECC_GET_TRNG_DIAG_MAX();
	*ok = IPECC_GET_TRNG_DIAG_OK();
	*starv = IPECC_GET_TRNG_DIAG_STARV();

	return 0;
}

/* To get the diagnostic counters for random source "RAW"
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_trng_diag_raw(uint32_t* min, uint32_t* max, uint32_t* ok, uint32_t* starv)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for TRNG diagnostics to be read back.
	 */

	/* First select the random source */
	IPECC_TRNG_SELECT_DIAG_ID(IPECC_W_DBG_TRNG_CTRL_DIAG_RAW);

	/* Memory-mapped register accesses */
	*min = IPECC_GET_TRNG_DIAG_MIN();
	*max = IPECC_GET_TRNG_DIAG_MAX();
	*ok = IPECC_GET_TRNG_DIAG_OK();
	*starv = IPECC_GET_TRNG_DIAG_STARV();

	return 0;
}

/* To get the diagnostic counters of all random sources at once
 *
 * Note : diag. counters for the "SHF" source are only read if in
 *        shuffle mode (as per capabilities displayed by the IP
 *        at runtime).
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_trng_diagnostics(trng_diagcnt_t* tdg)
{
	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for TRNG diagnostics to be read back.
	 */

	/* Call to low level routines */
	/*   random source "AXI" */
	ip_ecc_get_trng_diag_axi(&tdg->aximin, &tdg->aximax, &tdg->axiok, &tdg->axistarv);
	/*   random source "EFP" */
	ip_ecc_get_trng_diag_efp(&tdg->efpmin, &tdg->efpmax, &tdg->efpok, &tdg->efpstarv);
	/*   random source "CRV" */
	ip_ecc_get_trng_diag_crv(&tdg->crvmin, &tdg->crvmax, &tdg->crvok, &tdg->crvstarv);
	/*   random source "SHF" */
	if(IPECC_IS_SHUFFLING_SUPPORTED()){
		ip_ecc_get_trng_diag_shf(&tdg->shfmin, &tdg->shfmax, &tdg->shfok, &tdg->shfstarv);
	}
	/*   random source "RAW" */
	ip_ecc_get_trng_diag_raw(&tdg->rawmin, &tdg->rawmax, &tdg->rawok, &tdg->rawstarv);

	return 0;
}

/* To evaluate the frequency of 'clk' & 'clkmm' clocks
 *
 * (should be called only in debug mode)
 */
static inline int ip_ecc_get_clocks_freq(uint32_t* mhz, uint32_t* mhz_mm, uint32_t sec)
{
	uint32_t c0, c0_mm;
	uint32_t c1, c1_mm;

	/* We DON'T busy-wait as we assume the IP might be debug-halted
	 * in the course of a computation (hence with the busy BIT on,
	 * in which case it would create a dead-lock situation).
	 *
	 * We don't test if the IP is halted. it doesn't need to be
	 * for clock freqs to be estimated.
	 */

	/* Get counters */
	c0 = IPECC_GET_CLK_MHZ();
	c0_mm = IPECC_GET_CLKMM_MHZ();

	sleep(sec);

	/* Get counters again */
	c1 = IPECC_GET_CLK_MHZ();
	c1_mm = IPECC_GET_CLKMM_MHZ();

	*mhz = (((uint32_t)(c1 - c0)) * (0x1u << IPECC_R_DBG_CLK_PRECNT)) / ((sec)*1000000);
	*mhz_mm = (((uint32_t)(c1_mm - c0_mm)) * (0x1u << IPECC_R_DBG_CLK_PRECNT)) / ((sec)*1000000);

	return 0;
}

#if defined(KP_TRACE) || defined(KP_CHECK_ZMASK)
void ip_debug_read_all_limbs(uint32_t lgnb, uint32_t* nbbuf)
{
	uint32_t i;
	for (i = 0; i < IPECC_DBG_GET_W(); i++) {
		ip_ecc_read_limb(lgnb, i, &nbbuf[i]);
	}
}
#endif

#ifdef KP_TRACE
static void get_exp_flags(kp_exp_flags_t* flg)
{
	uint32_t dbg_exp_flags;

	/* read register R_DBG_EXP_FLAGS */
	dbg_exp_flags = IPECC_GET_REG(IPECC_R_DBG_EXP_FLAGS);
	/* copy flags to pointer */
	flg->r0z = (dbg_exp_flags >> IPECC_R_DBG_EXP_FLAGS_R0Z_POS) & 0x1;
	flg->r1z = (dbg_exp_flags >> IPECC_R_DBG_EXP_FLAGS_R1Z_POS) & 0x1;
	flg->kap = (dbg_exp_flags >> IPECC_R_DBG_EXP_FLAGS_KAP_POS) & 0x1;
	flg->kapp = (dbg_exp_flags >> IPECC_R_DBG_EXP_FLAGS_KAPP_POS) & 0x1;
	flg->zu = (dbg_exp_flags >> IPECC_R_DBG_EXP_FLAGS_ZU_POS) & 0x1;
	flg->zc = (dbg_exp_flags >> IPECC_R_DBG_EXP_FLAGS_ZC_POS) & 0x1;
	flg->jnbbit = (dbg_exp_flags >> IPECC_R_DBG_EXP_FLAGS_JNBBIT_POS) & IPECC_R_DBG_EXP_FLAGS_JNBBIT_MSK;
}

static void kp_trace_msg_append(kp_trace_info_t* ktrc, const char* fmt, ...)
{
#if 0
	uint32_t msglen;
#endif
	static bool overflow = false;
	va_list ap;

#if 0
	msglen = strlen(msg);
	if (overflow) {
		return;
	}
	if ( (ktrc->msgsz + msglen) > ktrc->msgsz_max ) {
		if (overflow == false) {
			printf("Warning: reached max size of [k]P trace buffer\n\r");
			overflow = true;
		}
		return;
	}
#endif

	if (overflow) {
		return;
	}
	/* We can sprintf now that we know we won't overflow the statically
	 * allocated trace log buffer.
	 */
	va_start(ap, fmt);
	ktrc->msgsz += vsprintf(ktrc->msg + ktrc->msgsz, fmt, ap);
	va_end(ap);
	if (ktrc->msgsz > ktrc->msgsz_max - 32) {
		if (overflow == false) {
			printf("%sWarning! About to reach max allocated size for [k]P trace buffer!..."
					" Losing subsequent trace logs%s\n\r", KUNK, KNRM);
			overflow = true;
		}
		return;
	}
}

void print_all_limbs_of_number(kp_trace_info_t* ktrc, const char* msg, uint32_t *nb)
{
  int32_t i;
  kp_trace_msg_append(ktrc, "%s", msg);
  for (i = IPECC_DBG_GET_W() - 1; i >= 0; i--) {
    kp_trace_msg_append(ktrc, "%0*x", DIV(IPECC_DBG_GET_WW(), 4), nb[i]);
  }
}

static inline void ip_read_and_print_xyr0(kp_trace_info_t* ktrc, kp_exp_flags_t* flg)
{
	ip_debug_read_all_limbs(IPECC_LARGE_NB_XR0_ADDR, ktrc->nb_xr0);
	ip_debug_read_all_limbs(IPECC_LARGE_NB_YR0_ADDR, ktrc->nb_yr0);
	print_all_limbs_of_number(ktrc, "[VHD-CMP-SAGE]     @ 4   XR0 = 0x", ktrc->nb_xr0);
	if (flg->r0z) { kp_trace_msg_append(ktrc, " but R0 = 0"); } kp_trace_msg_append(ktrc, "\n\r");
	print_all_limbs_of_number(ktrc, "[VHD-CMP-SAGE]     @ 5   YR0 = 0x", ktrc->nb_yr0);
	if (flg->r0z) { kp_trace_msg_append(ktrc, " but R0 = 0"); } kp_trace_msg_append(ktrc, "\n\r");
}

static inline void ip_read_and_print_xyr1(kp_trace_info_t* ktrc, kp_exp_flags_t* flg)
{
	ip_debug_read_all_limbs(IPECC_LARGE_NB_XR1_ADDR, ktrc->nb_xr1);
	ip_debug_read_all_limbs(IPECC_LARGE_NB_YR1_ADDR, ktrc->nb_yr1);
	print_all_limbs_of_number(ktrc, "[VHD-CMP-SAGE]     @ 6   XR1 = 0x", ktrc->nb_xr1);
	if (flg->r1z) { kp_trace_msg_append(ktrc, " but R1 = 0"); } kp_trace_msg_append(ktrc, "\n\r");
	print_all_limbs_of_number(ktrc, "[VHD-CMP-SAGE]     @ 7   YR1 = 0x", ktrc->nb_yr1);
	if (flg->r1z) { kp_trace_msg_append(ktrc, " but R1 = 0"); } kp_trace_msg_append(ktrc, "\n\r");
}

static inline void ip_read_and_print_zr01(kp_trace_info_t* ktrc)
{
	ip_debug_read_all_limbs(IPECC_LARGE_NB_ZR01_ADDR, ktrc->nb_zr01);
	print_all_limbs_of_number(ktrc, "[VHD-CMP-SAGE]     @ 26 ZR01 = 0x", ktrc->nb_zr01);
	kp_trace_msg_append(ktrc, "\n");
}

static int kp_debug_trace(kp_trace_info_t* ktrc)
{
	uint32_t dbgpc, dbgstate;
	kp_exp_flags_t flags;

	if (ktrc == NULL) {
		printf("Error: calling kp_debug_trace() with a null kp_trace_info_t pointer!\n\r");
		goto err;
	}

	/* Set first breakpoint on the first instruction
	 * of routine .checkoncurveL of the microcode.
	 */
	kp_trace_msg_append(ktrc, "Setting first breakpoint (on .checkoncurveL)\n\r");
	ip_ecc_set_breakpoint(DEBUG_ECC_IRAM_CHKCURVE_OP1_ADDR, 0);

	/* Transmit the [k]P run command to the IP. */
	kp_trace_msg_append(ktrc, "Running [k]P\n\r");
	IPECC_EXEC_PT_KP();

	/* Poll register R_DBG_STATUS until it shows IP is halted
	 * in debug mode.
	 */
	kp_trace_msg_append(ktrc, "Polling until debug halt\n\r");
	IPECC_POLL_UNTIL_DEBUG_HALTED();

	kp_trace_msg_append(ktrc, "IP is halted\n\r");
	/* IPECC IS HALTED */
	/* Get the PC & state from IPECC_R_DBG_STATUS */
	dbgpc = IPECC_GET_PC();
	dbgstate = IPECC_GET_FSM_STATE();
	
	/* Check that PC matchs 1st opcode of .checkoncurveL */
	if (dbgpc != DEBUG_ECC_IRAM_CHKCURVE_OP1_ADDR) {
		printf("Error in kp_debug_trace(): breakpoint was expected on 1st opcode "
				"of .checkoncurveL (0x%03x)\n\r", DEBUG_ECC_IRAM_CHKCURVE_OP1_ADDR);
		printf("      and instead it is on 0x%03x\n\r", dbgpc);
		goto err;
	}
	if (dbgstate != IPECC_DEBUG_STATE_CHECKONCURVE) {
		printf("Error in kp_debug_trace(): should be in state %d\n\r", IPECC_DEBUG_STATE_CHECKONCURVE);
		printf("      and instead in state (%d)\n\r", dbgstate);
		goto err;
	}

	kp_trace_msg_append(ktrc, "Starting step-by-step execution\n\r");
	/*
	 * Step-by-step loop
	 */
	do {
		/*
		 * Iterate an instruction
		 */
		IPECC_SINGLE_STEP();
		/*
		 * Poll register R_DBG_STATUS until it shows IP is halted
		 * in debug mode.
		 */
		IPECC_POLL_UNTIL_DEBUG_HALTED();
		ktrc->nb_steps++;
		/*
		 * Get current value of PC & state.
		 */
		dbgpc = IPECC_GET_PC();
		dbgstate = IPECC_GET_FSM_STATE();
		/*
		 * Get exception flags from register R_DBG_EXP_FLAGS
		 */
		get_exp_flags(&flags);

		switch (dbgpc) {

			case DEBUG_ECC_IRAM_RANDOM_ALPHA_ADDR:
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				kp_trace_msg_append(ktrc, "%sGetting alpha%s\n\r", KUNK, KNRM);
				ip_debug_read_all_limbs(IPECC_LARGE_NB_ALF_ADDR, ktrc->alpha);
				ktrc->alpha_valid = true;
				kp_trace_msg_append(ktrc, "%s", KUNK);
				print_all_limbs_of_number(ktrc, "alf = 0x", ktrc->alpha);
				kp_trace_msg_append(ktrc, "%s\n\r", KNRM);
				break;

			case DEBUG_ECC_IRAM_RANDOM_PHI01_ADDR:
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				kp_trace_msg_append(ktrc, "%sGetting phi0 & phi1%s\n\r", KUNK, KNRM);
				ip_debug_read_all_limbs(IPECC_LARGE_NB_PHI0_ADDR, ktrc->phi0);
				ktrc->phi0_valid = true;
				kp_trace_msg_append(ktrc, "%s", KUNK);
				print_all_limbs_of_number(ktrc, "phi0 = 0x", ktrc->phi0);
				kp_trace_msg_append(ktrc, "%s\n\r", KNRM);
				ip_debug_read_all_limbs(IPECC_LARGE_NB_PHI1_ADDR, ktrc->phi1);
				ktrc->phi1_valid = true;
				kp_trace_msg_append(ktrc, "%s", KUNK);
				print_all_limbs_of_number(ktrc, "phi1 = 0x", ktrc->phi1);
				kp_trace_msg_append(ktrc, "%s\n\r", KNRM);
				break;

			case DEBUG_ECC_IRAM_RANDOM_LAMBDA_ADDR:
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				if (flags.jnbbit == 1) {
					kp_trace_msg_append(ktrc, "%sGetting lambda (aka first Z-mask)%s\n\r", KUNK, KNRM);
				} else {
					kp_trace_msg_append(ktrc, "%sGetting periodic Z-remask%s\n\r", KUNK, KNRM);
				}
				ip_debug_read_all_limbs(IPECC_LARGE_NB_LAMBDA_ADDR, ktrc->lambda);
				ktrc->lambda_valid = true;
				kp_trace_msg_append(ktrc, "%s", KUNK);
				if (flags.jnbbit == 1) {
					print_all_limbs_of_number(ktrc, "lambda = 0x", ktrc->lambda);
				} else {
					print_all_limbs_of_number(ktrc, "Z-remask = 0x", ktrc->lambda);
				}
				kp_trace_msg_append(ktrc, "%s\n\r", KNRM);
				break;

			case DEBUG_ECC_IRAM_ZADDU_OP1_ADDR:
				/* 1st instruction of .zadduL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_SETUP)
				{
					/* We're still in setup (so we're about to compute
					 * (2P,P) -> (3P,P) using a call to ZADDU operator.
					 */
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (first part of setup, "
							"R0 <- [2]P), R1 <- [P])\n");
					/* Read values of [XY]R[01] and flags r0z and r1z.
					 */
					ip_read_and_print_xyr0(ktrc, &flags);
					ip_read_and_print_xyr1(ktrc, &flags);
					ip_read_and_print_zr01(ktrc);
				}
				break;

			case DEBUG_ECC_IRAM_ITOH_ADDR: /* PC_ITOH_FIRST */
				/* 1st instruction of .itohL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_ITOH) {
					if (flags.jnbbit == 1) {
						kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
						kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (second part of setup, "
								"[3]P <- [2]P + P by ZADDU completed)\n");
						/* read values of [XY]R[01] and flags r0z and r1z */
						ip_read_and_print_xyr0(ktrc, &flags);
						ip_read_and_print_xyr1(ktrc, &flags);
						ip_read_and_print_zr01(ktrc);
					} else {
						kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
						kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates after ZADDC of BIT %d "
								"(kap%d = %d,  kap'%d = %d)\n",
								flags.jnbbit, flags.jnbbit, flags.kap, flags.jnbbit, flags.kapp);
						/* read values of [XY]R[01] and flags r0z and r1z */
						ip_read_and_print_xyr0(ktrc, &flags);
						ip_read_and_print_xyr1(ktrc, &flags);
						ip_read_and_print_zr01(ktrc);
					}
				}
				break;

			case DEBUG_ECC_IRAM_PRE_ZADDC_OP1_ADDR: /* PC_PREZADDC_FIRST */
				/* 1st instruction of .pre_zaddcL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_ZADDC) {
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates after ZADDU of BIT %d "
							"(kap%d = %d,  kap'%d = %d)\n",
							flags.jnbbit, flags.jnbbit, flags.kap, flags.jnbbit, flags.kapp);
					/* read values of [XY]R[01] and flags r0z and r1z */
					ip_read_and_print_xyr0(ktrc, &flags);
					ip_read_and_print_xyr1(ktrc, &flags);
					ip_read_and_print_zr01(ktrc);
				}
				break;

			case DEBUG_ECC_IRAM_SUBTRACTP_OP1_ADDR: /* PC_SUBTRACTP_FIRST */
				/* 1st instruction of .subtractPL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_SUBTRACTP) {
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates after ZADDC of BIT %d "
							"(kap%d = %d,  kap'%d = %d)\n",
							flags.jnbbit, flags.jnbbit, flags.kap, flags.jnbbit, flags.kapp);
					/* read values of [XY]R[01] and flags r0z and r1z */
					ip_read_and_print_xyr0(ktrc, &flags);
					ip_read_and_print_xyr1(ktrc, &flags);
					ip_read_and_print_zr01(ktrc);
				}
				break;

			case DEBUG_ECC_IRAM_ZADDC_OP1_ADDR: /* PC_ZADDC_FIRST */
				/* 1st instruction of .zaddcL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_SUBTRACTP) {
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (first part of subtractP, "
							"[k + 1 - (k mod 2)]P & P made Co-Z)\n");
					/* read values of [XY]R[01] and flags r0z and r1z */
					ip_read_and_print_xyr0(ktrc, &flags);
					ip_read_and_print_xyr1(ktrc, &flags);
					ip_read_and_print_zr01(ktrc);
				}
				break;

			case DEBUG_ECC_IRAM_ZDBL_OP1_ADDR: /* PC_ZDBL_FIRST */
				/* 1st instruction of .zdblL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_SUBTRACTP) {
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (first part of subtractP, "
							"[k + 1 - (k mod 2)]P & P made Co-Z)\n");
					/* read values of [XY]R[01] and flags r0z and r1z */
					ip_read_and_print_xyr0(ktrc, &flags);
					ip_read_and_print_xyr1(ktrc, &flags);
					ip_read_and_print_zr01(ktrc);
				}
				break;

			case DEBUG_ECC_IRAM_ZNEGC_OP1_ADDR: /* PC_ZNEGC_FIRST */
				/* 1st instruction of .znegcL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_SUBTRACTP) {
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (first part of subtractP, "
							"[k + 1 - (k mod 2)]P & P made Co-Z)\n");
					/* read values of [XY]R[01] and flags r0z and r1z */
					ip_read_and_print_xyr0(ktrc, &flags);
					ip_read_and_print_xyr1(ktrc, &flags);
					ip_read_and_print_zr01(ktrc);
				}
				break;

			case DEBUG_ECC_IRAM_EXIT_OP1_ADDR: /* PC_EXIT_FIRST */
				/* 1st instruction of .exitL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_EXIT) {
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R1 coordinates (second part of subtractP, "
							"cond. sub. [k + 1 - (k mod 2)]P - P completed)\n");
					/* read values of [XY]R[01] and flags r0z and r1z */
					ip_read_and_print_xyr1(ktrc, &flags);
				}
				break;

			case DEBUG_ECC_IRAM_CHKCURVE_OPLAST_ADDR: /* PC_CHECK_CURVE_LAST */
				/* 1st instruction of .chkcurveL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_EXIT) {
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R1 coordinates (after exit routine, "
							"end of computation, result is in R1 if not null)\n");
					/* read values of [XY]R[01] and flags r0z and r1z */
					ip_read_and_print_xyr1(ktrc, &flags);
				}
				break;

			case DEBUG_ECC_IRAM_ZADD_VOID_ADDR:
				/* Only instruction of .zadd_voidL
				 */
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (in .zadd_voidL)\n");
					/* Read values of [XY]R[01] and flags r0z and r1z.
					 */
				ip_read_and_print_xyr0(ktrc, &flags);
				ip_read_and_print_xyr1(ktrc, &flags);
				ip_read_and_print_zr01(ktrc);
				break;

			case DEBUG_ECC_IRAM_ZDBL_NOT_ALWAYS_OP1_ADDR:
				/* 1st instruction of .zdbl_not_alwaysL
				 */
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (entrance of .zdbl_not_alwaysL)\n");
					/* Read values of [XY]R[01] and flags r0z and r1z.
					 */
				ip_read_and_print_xyr0(ktrc, &flags);
				ip_read_and_print_xyr1(ktrc, &flags);
				ip_read_and_print_zr01(ktrc);
				break;

			case DEBUG_ECC_IRAM_ZDBL_NOT_ALWAYS_OPLAST_ADDR:
				/* Last instruction of .zdbl_not_alwaysL
				 */
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (terminated .zdbl_not_alwaysL)\n");
					/* Read values of [XY]R[01] and flags r0z and r1z.
					 */
				ip_read_and_print_xyr0(ktrc, &flags);
				ip_read_and_print_xyr1(ktrc, &flags);
				ip_read_and_print_zr01(ktrc);
				break;

			case DEBUG_ECC_IRAM_PRE_ZADDU_LAST_ADDR:
				/* Last instruction of .zadduL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_ZADDU)
				{
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (terminated .pre_zadduL)\n");
					/* Read values of [XY]R[01] and flags r0z and r1z.
					 */
					ip_read_and_print_xyr0(ktrc, &flags);
					ip_read_and_print_xyr1(ktrc, &flags);
					ip_read_and_print_zr01(ktrc);
				}
				break;

			case DEBUG_ECC_IRAM_ZADDU_OPLAST_ADDR:
				/* Last instruction of .zadduL
				 */
				if (dbgstate == IPECC_DEBUG_STATE_ZADDU)
				{
					kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
					kp_trace_msg_append(ktrc, "[VHD-CMP-SAGE] R0/R1 coordinates (terminated .zadduL)\n");
					/* Read values of [XY]R[01] and flags r0z and r1z.
					 */
					ip_read_and_print_xyr0(ktrc, &flags);
					ip_read_and_print_xyr1(ktrc, &flags);
					ip_read_and_print_zr01(ktrc);
				}
				break;

			case DEBUG_ECC_IRAM_RANDOM_KAPMSK_ADDR:
				/* Read kap0msk & kap1msk */
				ip_debug_read_all_limbs(IPECC_LARGE_NB_KAP0MSK_ADDR, ktrc->kap0msk);
				ktrc->kap0msk_valid = true;
				ip_debug_read_all_limbs(IPECC_LARGE_NB_KAP1MSK_ADDR, ktrc->kap1msk);
				ktrc->kap1msk_valid = true;
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				print_all_limbs_of_number(ktrc, "  kap0msk  = 0x", ktrc->kap0msk); kp_trace_msg_append(ktrc, "\n\r");
				print_all_limbs_of_number(ktrc, "  kap1msk  = 0x", ktrc->kap1msk); kp_trace_msg_append(ktrc, "\n\r");
				break;

			case DEBUG_ECC_IRAM_RANDOM_KAPPMSK_ADDR:
				/* Read kapP0msk & kapP1msk */
				ip_debug_read_all_limbs(IPECC_LARGE_NB_KAPP0MSK_ADDR, ktrc->kapP0msk);
				ktrc->kapP0msk_valid = true;
				ip_debug_read_all_limbs(IPECC_LARGE_NB_KAPP1MSK_ADDR, ktrc->kapP1msk);
				ktrc->kapP1msk_valid = true;
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				print_all_limbs_of_number(ktrc, "  kapP0msk  = 0x", ktrc->kapP0msk); kp_trace_msg_append(ktrc, "\n\r");
				print_all_limbs_of_number(ktrc, "  kapP1msk  = 0x", ktrc->kapP1msk); kp_trace_msg_append(ktrc, "\n\r");
				break;

			case DEBUG_ECC_IRAM_RANDOM_PHIMSK_ADDR:
				/* Read phi0msk & phi1msk */
				ip_debug_read_all_limbs(IPECC_LARGE_NB_PHI0MSK_ADDR, ktrc->phi0msk);
				ktrc->phi0msk_valid = true;
				ip_debug_read_all_limbs(IPECC_LARGE_NB_PHI1MSK_ADDR, ktrc->phi1msk);
				ktrc->phi1msk_valid = true;
				kp_trace_msg_append(ktrc, "PC=%s0x%03x%s (%s%s%s)\n\r", KGRN, dbgpc, KNRM, KYEL, str_ipecc_state(dbgstate), KNRM);
				print_all_limbs_of_number(ktrc, "  phi0msk  = 0x", ktrc->phi0msk); kp_trace_msg_append(ktrc, "\n\r");
				print_all_limbs_of_number(ktrc, "  phi1msk  = 0x", ktrc->phi1msk); kp_trace_msg_append(ktrc, "\n\r");
				break;

			default:
				break;
		} /* switch-case on dbgpc */
		/*
		 * If IP is halted in state 'exits' and is about to
		 * execute the last opcode of routine .chkcurveL
		 * we exit the loop.
		 */
		if ( (dbgpc == DEBUG_ECC_IRAM_CHKCURVE_OPLAST_ADDR) && (dbgstate == IPECC_DEBUG_STATE_EXIT) ) {
			break;
		}

	} while (1);

	kp_trace_msg_append(ktrc, "%d debug steps for this [k]P computation.\n", ktrc->nb_steps);

	kp_trace_msg_append(ktrc, "Removing breakpoint & resuming.\n\r");
	IPECC_REMOVE_BREAKPOINT(0);
	IPECC_RESUME();

	return 0;
err:
	return -1;
}
#endif /* KP_TRACE */

#ifdef KP_SET_ZMASK
static int kp_run_with_specific_zmask(uint32_t* zmask)
{
	int i;
	uint32_t w, n;
	uint32_t dbgpc, dbgstate;

	/* 32768 bits are more than enough for any practical
	 * use of elliptic curve cryptography.
	 */
	uint32_t check_zmask[4096 / sizeof(uint32_t)]; /* Heck, a whole page? Yes indeed. */

	if (zmask == NULL) {
		printf("Error: calling kp_run_with_specific_zmask() with a null pointer!\n\r");
		goto err;
	}

	/* Set breakpoint on the NOP instruction just following
	 * the random draw of lambda & its reduction. We can find the
	 * specific opcode address thx to ecc_addr.h (generated in
	 * hdl/ecc_curve_iram when doing 'make' in that folder).
	 */
	ip_ecc_set_breakpoint(DEBUG_ECC_IRAM_RANDOM_LAMBDA_ADDR, 0);

	/* Transmit the [k]P run command to the IP. */
	IPECC_EXEC_PT_KP();

	/* Poll register R_DBG_STATUS until it shows IP is halted
	 * in debug mode.
	 */
	IPECC_POLL_UNTIL_DEBUG_HALTED();

	/* IPECC IS HALTED */
	/* Get the PC & state from IPECC_R_DBG_STATUS */
	dbgpc = IPECC_GET_PC();
	dbgstate = IPECC_GET_FSM_STATE();

	/* Check that PC matchs the expected opcode of .setupL */
	if (dbgpc != DEBUG_ECC_IRAM_RANDOM_LAMBDA_ADDR) {
		printf("Error in kp_run_with_specific_zmask(): breakpoint was expected on opcode "
				"0x%03x\n\r", DEBUG_ECC_IRAM_RANDOM_LAMBDA_ADDR);
		printf("      and instead it is on 0x%03x\n\r", dbgpc);
		goto err;
	}
	if (dbgstate != IPECC_DEBUG_STATE_SETUP) {
		printf("Error in kp_run_with_specific_zmask(): should be in state %d\n\r", IPECC_DEBUG_STATE_SETUP);
		printf("      and instead in state (%d, decode this number using file <ecc_states.h>)\n\r", dbgstate);
		goto err;
	}

#ifdef KP_CHECK_ZMASK
	/* Get & display initial random value drawn as Z-mask
	 */
	ip_debug_read_all_limbs(IPECC_LARGE_NB_LAMBDA_ADDR, check_zmask);

	printf("Initially drawn random:     Zmask = 0x");
	for (i=IPECC_DBG_GET_W() - 1 ; i>=0; i--) {
		printf("%04x", check_zmask[i]);
	}
	printf("\n\r");
#endif

	w = DIV(IPECC_GET_NN_MAX() + 4, IPECC_DBG_GET_WW());

	/* Ignore possible error return case for ge_pow_of_2 here. */
	ge_pow_of_2(w, &n);

	for (i = 0; i < (int)IPECC_DBG_GET_W(); i++) {
		/* Set proper address for current limb.
		 * The address of 'lambda' variable (the random Z-masking large number in the
		 * memory of large numbers) is given in file <ecc_vars.h>, which is automatically
		 * generated by the Makefile in ecc_curve_iram/.
		 */
		IPECC_DBG_SET_FP_WRITE_ADDR((IPECC_LARGE_NB_LAMBDA_ADDR * n) + i);
		/* Now set the value of the limb.
		 * This also triggers the write itself.
		 */
		IPECC_DBG_SET_FP_WRITE_DATA(zmask[i]);
	}

#ifdef KP_CHECK_ZMASK
	/* Check that value was correctly set into memory of large nbs.
	 */
	ip_debug_read_all_limbs(IPECC_LARGE_NB_LAMBDA_ADDR, check_zmask);

	printf("Read-back after modif:      Zmask = 0x");
	for (i=IPECC_DBG_GET_W() - 1 ; i>=0; i--) {
		printf("%04x", check_zmask[i]);
	}
	printf("\n\r");

	/* set new breakpoint just before ZR01 is masked */
	ip_ecc_set_breakpoint(DEBUG_ECC_IRAM_CHECK0_ZMASK_ADDR, 0);
	IPECC_RESUME();

	IPECC_POLL_UNTIL_DEBUG_HALTED();
	/* get value of ZR01 before masking */
	ip_debug_read_all_limbs(IPECC_LARGE_NB_ZR01_ADDR, check_zmask);
	printf("Checked in memory (before): ZR01  = 0x");
	for (i=IPECC_DBG_GET_W() - 1 ; i>=0; i--) {
		printf("%04x", check_zmask[i]);
	}
	printf("\n\r");

	/* set new breakpoint just after ZR01 has been masked */
	ip_ecc_set_breakpoint(DEBUG_ECC_IRAM_CHECK1_ZMASK_ADDR, 0);
	IPECC_RESUME();

	IPECC_POLL_UNTIL_DEBUG_HALTED();
	/* get value of ZR01 before masking */
	ip_debug_read_all_limbs(IPECC_LARGE_NB_ZR01_ADDR, check_zmask);
	printf("Checked in memory (after):  ZR01  = 0x");
	for (i=IPECC_DBG_GET_W() - 1 ; i>=0; i--) {
		printf("%04x", check_zmask[i]);
	}
	printf("\n\r");

#endif

	IPECC_REMOVE_BREAKPOINT(0);
	IPECC_RESUME();

	return 0;
err:
	return -1;
}
#endif /* KP_SET_ZMASK */

/*
 * Commands execution (point operation)
 *
 * The default behaviour should be to call ip_ecc_exec_command() in 'blocking'
 * mode (the software driver will poll the BUSY WAIT bit until it is cleared
 * by the hardware). When in debug mode setting 'blocking' to 0 allowsa to
 * debug monitor the operation, using e.g breakpoints.
 */
static inline int ip_ecc_exec_command(ip_ecc_command cmd, int *flag, kp_trace_info_t* ktrc, uint32_t* zmask)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Execute the command */
	switch(cmd){
		case PT_ADD:{
			IPECC_EXEC_PT_ADD();
			break;
		}
		case PT_DBL:{
			IPECC_EXEC_PT_DBL();
			break;
		}
		case PT_KP:{
#ifdef KP_TRACE
			if (ktrc == NULL) {
				/* If debug ptr is null, this means no debug trace is required,
				 * so run the command immediately.
				 */
				IPECC_EXEC_PT_KP();
			} else {
				/* Since the ptr is not null, this means debug trace must happen,
				 * and some config is required before running the [k]P command,
				 * which is done by kp_debug_trace().
				 */
				if (kp_debug_trace(ktrc)) {
					goto err;
				};
			}
#else
  #ifdef KP_SET_ZMASK
			/* sanity check */
			if (zmask == NULL) {
				IPECC_EXEC_PT_KP();
			} else {
				/* Since 'zmask' ptr is not null, this means it contains a valid
				 * large number to be used as the initial "random" masking for Z
				 * coordinate.
				 *
				 * Some config is required before running the [k]P command,
				 * which is done by kp_run_with_specific_zmask().
				 */
				if (kp_run_with_specific_zmask(zmask)) {
					goto err;
				};
			}
			(void)ktrc; /* To avoid unused parameter warning from gcc */
  #else
			IPECC_EXEC_PT_KP();
			(void)ktrc; /* To avoid unused parameter warning from gcc */
			(void)zmask; /* To avoid unused parameter warning from gcc */
  #endif
#endif
			break;
		}
		case PT_CHK:{
			IPECC_EXEC_PT_CHK();
			break;
		}
		case PT_EQU:{
			IPECC_EXEC_PT_EQU();
			break;
		}
		case PT_OPP:{
			IPECC_EXEC_PT_OPP();
			break;
		}
		case PT_NEG:{
			IPECC_EXEC_PT_NEG();
			break;
		}
		default:{
			goto err;
		}
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Check for error */
	if(ip_ecc_check_error(NULL)){
		goto err;
	}

	/* Get a flag if necessary */
	if(flag != NULL){
		switch(cmd){
			case PT_CHK:{
				(*flag) = IPECC_GET_ONCURVE();
				break;
			}
			case PT_EQU:{
				(*flag) = IPECC_GET_EQU();
				break;
			}
			case PT_OPP:{
				(*flag) = IPECC_GET_OPP();
				break;
			}
			default:{
				goto err;
			}
		}
	}

	return 0;
err:
	return -1;
}

/* Is the IP in 'debug' or 'production' mode? */
static inline int ip_ecc_is_debug(bool* debug)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Ask the IP register. */
	if (IPECC_IS_DEBUG())
	{
		*debug = true;
	} else {
		*debug = false;
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	return 0;
}

static inline int ip_ecc_is_prod(bool* prod)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Ask the IP register. */
	if (IPECC_IS_PROD())
	{
		*prod = true;
	} else {
		*prod = false;
	}

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	return 0;
}

/* To get hardware capabilities from the IP */
static inline int ip_ecc_get_capabilities(bool* debug, bool* shuffle, bool* nndyn, bool* axi64, uint32_t* nnmax)
{
	if (IPECC_IS_DEBUG())
	{
		*debug = true;
	} else {
		*debug = false;
	}
	if (IPECC_IS_SHUFFLING_SUPPORTED())
	{
		*shuffle = true;
	} else {
		*shuffle = false;
	}
	if (IPECC_IS_DYNAMIC_NN_SUPPORTED())
	{
		*nndyn = true;
	} else {
		*nndyn = false;
	}
	if (IPECC_IS_W64())
	{
		*axi64 = true;
	} else {
		*axi64 = false;
	}
	*nnmax = IPECC_GET_NN_MAX();

	return 0;
}

/* Get the three version nb of the IP */
static inline int ip_ecc_get_version_tags(uint32_t* maj, uint32_t* min, uint32_t* ptc)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Get all version numbers from IP register. */
	*maj = IPECC_GET_MAJOR_VERSION();
	*min = IPECC_GET_MINOR_VERSION();
	*ptc = IPECC_GET_PATCH_VERSION();

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	return 0;
}

int ip_ecc_attack_set_cfg_0(bool naive, bool nocollisioncr)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Transmit configuration to low-level routine */
	IPECC_ATTACK_SET_HW_CFG(naive, nocollisioncr);

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	return 0;
}

/* Enable masking of kappa & kappa' by shift-registers embedded in the hardware.
 */
int ip_ecc_attack_enable_nnrndsf()
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Transmit configuration to low-level routine */
	IPECC_ATTACK_ENABLE_NNRNDSF();

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	return 0;
}

/* Disable masking of kappa & kappa' by shift-registers embedded in the hardware.
 */
int ip_ecc_attack_disable_nnrndsf()
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Transmit configuration to low-level routine */
	IPECC_ATTACK_DISABLE_NNRNDSF();

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	return 0;
}

int ip_ecc_attack_set_clock_div_out(int div, int divmm)
{
	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Transmit configuration to low-level routine */
	IPECC_ATTACK_SET_CLOCK_DIVOUT(div, divmm);

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	return 0;
}

#if 0
/* Function to get the random output of the RAW FIFO */
static inline int ip_ecc_get_random(uint8_t *out, uint32_t out_sz)
{
	uint32_t read = 0, addr;
	uint8_t bit;

	/* Wait until the IP is not busy */
	IPECC_BUSY_WAIT();

	/* Read in a loop the asked size */
	while(read != (8 * out_sz)){
		uint32_t i;
		/* Wait until our FIFO is full */
		IPECC_TRNG_RAW_FIFO_FULL_BUSY_WAIT();
		/* Read all our data */
		addr = 0;
		for(i = 0; i < IPECC_GET_TRNG_RAW_SZ(); i++){
			IPECC_TRNG_READ_FIFO_RAW(addr, &bit);
			if((read % 8) == 0){
				out[(read / 8)] = 0;
			}
			out[(read / 8)] |= (bit << (read % 8));
			addr++;
			read++;
			if(read == (8 * out_sz)){
				break;
			}
		}
	}

	return 0;
err:
	return -1;
}
#endif

static volatile uint8_t hw_driver_setup_state = 0;

static inline int driver_setup(void)
{
	bool debug;

	if(!hw_driver_setup_state){
		/* Ask the lower layer for a setup */
		if(hw_driver_setup((volatile uint8_t**)&ipecc_baddr, NULL /*(volatile uint8_t**)&ipecc_pseudotrng_baddr)*/)) {
			goto err;
		}
		/* Reset the IP for a clean state */
		IPECC_SOFT_RESET();

		/* Enable TRNG post-processing
		 *
		 * This is for the case where the IP is in DEBUG mode (not to be done otherwise
		 * as an error UNKNOWN_REG would be issued).
		 *
		 * NOTE:
		 *   We can make this call even before setting 'hw_driver_setup_state' to 1
		 *   below, because neither ip_ecc_is_debug() nor ip_ecc_trng_postproc_enable()
		 *   call driver_setup()
		 *   (so no risk of recursive deadlock).
		 */
		ip_ecc_is_debug(&debug);
		if (debug) {
			ip_ecc_trng_postproc_enable();
		}

#if 0
		/* Reset the pseudo TRNG device to empty its FIFO of pseudo raw random bytes */
		IPECC_PSEUDOTRNG_SOFT_RESET();
#endif

		/* We are in the initialized state */
		hw_driver_setup_state = 1;
	}

	return 0;
err:
	return -1;
}

/*********************************************
 **  Driver API (top-layer exported functions)
 *********************************************/

/* Reset the hardware */
int hw_driver_reset(void)
{
	/* Reset the IP for a clean state */
	IPECC_SOFT_RESET();

	return 0;
}

/* To know if the IP is in 'debug' or 'production' mode */
int hw_driver_is_debug(bool* debug)
{
	if(driver_setup()){
		goto err;
	}
	if (ip_ecc_is_debug(debug)){
		goto err;
	}
	return 0;
err:
	return -1;
}
int hw_driver_is_prod(bool* prod)
{
	if(driver_setup()){
		goto err;
	}
	if (ip_ecc_is_prod(prod)){
		goto err;
	}
	return 0;
err:
	return -1;
}

/* To get hardware capabilities from the IP
 */
int hw_driver_get_capabilities(bool* debug, bool* shuffle, bool* nndyn, bool* axi64, uint32_t* nnmax)
{
	if(driver_setup()){
		goto err;
	}
	if (ip_ecc_get_capabilities(debug, shuffle, nndyn, axi64, nnmax)){
		goto err;
	}
	return 0;
err:
	return -1;
}

	/* Get major version of the IP */
int hw_driver_get_version_tags(uint32_t* maj, uint32_t* min, uint32_t* patch)
{
	if(driver_setup()){
		goto err;
	}
	if (ip_ecc_get_version_tags(maj, min, patch)){
		goto err;
	}
	return 0;
err:
	return -1;
}

/* To halt the IP - This freezes execution of microcode
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_halt_DBG(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capabilities */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_debug_halt()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* To set and activate a new breakpoint
 *
 *   Note: the breakpoint is enabled for whatever the state
 *         the IP FSM is in when the microcode hits the address.
 *         Please c.f comment help of ip_ecc_set_breakpoint().
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_set_breakpoint_DBG(uint32_t addr, uint32_t id)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_set_breakpoint(addr, id)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* To remove/disable a breakpoint
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_remove_breakpoint_DBG(uint32_t id)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_remove_breakpoint(id)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Have IP to execute a certain nb of opcodes in microcode
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_run_opcodes_DBG(uint32_t nbops)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_run_opcodes(nbops)) {
		goto err;
	}

	/* Note: it is the caller's responsability to further test that the IP
	 * is again halted, hence meaning that the 'nbops' opcodes have been
	 * run - use API function hw_driver_is_debug_halted_DBG()
	 */

	return 0;
err:
	return -1;
}

/* Same as hw_driver_run_opcodes_DBG() but single step
 */
int hw_driver_single_step_DBG()
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_single_step()) {
		goto err;
	}

	/* Note: it is the caller's responsability to further test that the IP
	 * is again halted, hence meaning that the one opcode have been
	 * run - use API function hw_driver_is_debug_halted_DBG()
	 */

	return 0;
err:
	return -1;
}

/* Resume execution of microcode
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_resume_DBG()
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_resume()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Arm trigger function
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_arm_trigger_DBG()
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_arm_trigger()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disarm/disable trigger function
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_disarm_trigger_DBG()
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_disarm_trigger()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Set configuration of UP trigger detection
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_set_trigger_up_DBG(uint32_t time)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_set_trigger_up(time)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Set configuration of DOWN trigger detection
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_set_trigger_down_DBG(uint32_t time)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_set_trigger_down(time)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Patch a single opcode in the microcode.
 * 
 *   'opcode_msb' must contain the upper 32-bit part of the opcode in case
 *   size of opcodes is larger than 32-bit (and in this case 'opsz' must = 2).
 *
 *   'opcode_lsb' must contain the lower 32-bit part of the opcode in case
 *   size of the opcode is larger than 32-bit, OR the opcode word in case
 *   the size is 32-bit or less (and in this case 'opsz' must = 1).
 *
 * (in debug mode only).
 */
int hw_driver_patch_one_opcode_DBG(uint32_t address, uint32_t opcode_msb, uint32_t opcode_lsb, uint32_t opsz)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	ip_ecc_patch_one_opcode(address, opcode_msb, opcode_lsb, opsz);

	return 0;
err:
	return -1;
}

/* Patch a portion or the whole of microcode image
 *
 * (in debug mode only).
 */
int hw_driver_patch_microcode_DBG(uint32_t* buf, uint32_t nbops, uint32_t opsz)
{
	if(driver_setup()){
		goto err;
	}
	
	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_patch_microcode(buf, nbops, opsz)) {
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Set configuration of TRNG
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_configure_trng_DBG(int debias, uint32_t ta, uint32_t cycles)
{
	if(driver_setup()){
		goto err;
	}
	
	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_configure_trng(debias, ta, cycles)) {
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Reset the raw random bits FIFO
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_reset_trng_raw_fifo_DBG(void)
{
	if(driver_setup()){
		goto err;
	}
	
	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_reset_trng_raw_fifo()) {
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Reset all the internal random number FIFOs
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_reset_trng_irn_fifos_DBG(void)
{
	if(driver_setup()){
		goto err;
	}
	
	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_reset_trng_irn_fifos()) {
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable the TRNG post-processing logic that pulls bytes from the
 * raw random source.
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_trng_post_proc_disable_DBG(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_trng_postproc_disable()){
		goto err;
	}
	return 0;
err:
	return -1;
}

/* Enable TRNG post-processing logic
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_trng_post_proc_enable_DBG(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_trng_postproc_enable()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* TRNG complete bypass, also specifying deterministic bit value
 * to use instead of random bits.
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_bypass_full_trng_DBG(uint32_t instead_bit)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_bypass_full_trng(instead_bit)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Remove TRNG complete bypass, restoring nominal behaviour
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_dont_bypass_trng_DBG(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_dont_bypass_trng()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Force a deterministic effect (all 1's) of the NNRND instruction
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_nnrnd_deterministic_DBG(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_trng_nnrnd_deterministic()){
		goto err;
	}
	return 0;
err:
	return -1;
}

/* Restore nominal action of NNRND instruction, undoing action
 * of previous hw_driver_nnrnd_deterministic_DBG().
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_nnrnd_not_deterministic_DBG(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_trng_nnrnd_not_deterministic()){
		goto err;
	}
	return 0;
err:
	return -1;
}

/* To select the random source of which to read the diagnostics
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_select_trng_diag_source_DBG(uint32_t id)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_select_trng_diag_source(id)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Get one bit from the raw random FIFO
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_read_one_raw_random_bit_DBG(uint32_t addr, uint32_t* rawbit)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_read_one_raw_random_bit(addr, rawbit)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Write one word in the IP memory of large numbers.
 *
 *       LIMITATION : words (limbs of large nbs) must be of size
 *                    less than 32 bits (ww < 32 or equal) other-
 *                    wise an error is raised.
 *       *******************************************************
 *
 * This function writes one limb (of size ww-bit) of one large number
 * (which are made of 'w' limbs) in the IP memory of large numbers.
 *
 *   'addr' is the address in the memory (not the index of the large
 *   number) in other words it is the address of the limb/word itself.
 *
 *   'limb' is the data value to write.
 *
 * If ww > 32, then one limb cannot fit into a single machine word
 * (assuming a 32-bit host). An error is returned (and no action is
 * taken) in this case.
 *
 * The value of 'ww' is obtained inside the function by reading
 * the IP debug capabilities registers.
 *
 * Can only be called when in debug mode.
 */
int hw_driver_write_word_in_lgnbmem_DBG(uint32_t addr, uint32_t limb)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_write_word_in_lgnbmem(addr, limb)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Write one limb of one large nb in the IP memory of large numbers.
 *
 *   This function is much like hw_driver_write_word_in_lgnbmem_DBG().
 *   The difference is simply that it takes as parameter, instead
 *   of the address in memory, the index of the large nb and the
 *   index of the limb, to automatically compute the address where
 *   to write.
 *
 *       LIMITATION : words (limbs of large nbs) must be of size
 *                    less than 32 bits (ww < 32 or equal) other-
 *                    wise an  error is raised.
 *       *******************************************************
 *
 * This function writes one limb (of size ww-bit) of one large number
 * (which are made of 'w' limbs) in the memory of large numbers in the IP.
 *
 *   'i' is the index of the large number (there are typically 32 of them)
 *   so typically 'i' should be in the range 0 - 31.
 *   
 *   'j' is the index of the ww-bit limb in the large nb.
 *
 *       The address accessed in the IP memory of large nbs therefore is
 *       (i * n) + j. The value of 'n' is obtained inside the function
 *       from the IP capabilities registers.
 *
 *   'limb' is the data value to write.
 *
 * If ww > 32, then one limb cannot fit into a single machine word
 * (assuming a 32-bit host). An error is returned (and nothing done)
 * in this case.
 *
 * The value of 'ww' is obtained inside the function by reading
 * the IP debug capabilities registers.
 *
 * Can only be called when in debug mode.
 */
int hw_driver_write_limb_DBG(int32_t i, uint32_t j, uint32_t limb)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_write_limb(i, j, limb)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Write one large number in the IP memory of large numbers.
 *
 *       LIMITATION : words (limbs of large nbs) must be of size
 *                    less than 32 bits (ww < 32or  equal), other-
 *                    wise an error is raised.
 *       *******************************************************
 *
 * This function writes a complete large nb (made of 'w' words,
 * each of size 'ww'-bit) in the memory of large numbers in the IP.
 *
 *   'i' is the index of the large nb in memory. There are typically
 *   32 large nbs that the IP can store in its memory (see parameter
 *   'nblargenb' in source file ecc_customize.vhd) hence 'addr'
 *   typically should be in range 0 - 31.
 *
 *   'limbs' is a pointer to a buffer containing the large nb to write.
 *   Therefore it must store 'w' valid limbs, with limb[0] being the
 *   least significant value of the larne nb and limb[ww - 1] the most
 *   significant one.
 *
 *   Ex.: Assume the following large number X:
 *
 *            X = 0x_DEAD_0000_0000_0000_0000_0000_0000_BEEF (nn = 128 bits).
 *
 *        Assume ww = 16, then w = 9 (remember that w = ceil( (nn + 4) / ww))
 *        In this case, X must be passed to the function using:
 *
 *           limb[0] = 0xBEEF
 *           ...
 *           limb[7] = 0xDEAD
 *           limb[8] = 0.
 *
 * If ww > 32, then one limb cannot fit into a single machine word
 * (assuming a 32-bit host). An error is returned (and no action
 * taken) in this case.
 *
 * The values of 'w' & 'ww' are obtained inside the function by reading
 * the IP debug capabilities registers.
 *
 * Can only be called when in debug mode.
 */
int hw_driver_write_largenb_DBG(uint32_t i, uint32_t* limbs)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_write_largenb(i, limbs)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Read one word from the IP memory of large numbers.
 *
 *       LIMITATION : words (limbs of large nbs) must be of size
 *                    less than 32 bits (ww < 32 or equal) other-
 *                    wise an error is raised.
 *       ********************************************************
 *
 * This function reads one limb (of size ww-bit) of one large number
 * (which are made of 'w' limbs) from the IP memory of large numbers.
 *
 *   'addr' is the address in the memory (not the index of the large
 *   number) in other words it is the address of the limb/word itself.
 *
 *   'limb' is a pointer where to transfer the value read.
 *
 * If ww > 32, then one limb cannot fit into a single machine word
 * (assuming a 32-bit host). An error is returned (and no action done)
 * in this case.
 *
 * The value of 'ww' is obtained inside the function by reading
 * the IP debug capabilities registers.
 *
 * Can only be called when in debug mode.
 */
int hw_driver_read_word_from_lgnbmem_DBG(uint32_t addr, uint32_t* limb)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_read_word_from_lgnbmem(addr, limb)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Read one limb of one large nb from the IP memory of large numbers.
 *
 *   This function is much like hw_driver_read_word_from_lgnbmem_DBG().
 *   The difference is simply that it takes as parameter, instead
 *   of the address in memory, the index of the large nb and the
 *   index of the limb, to automatically compute the address where
 *   to read from.
 *
 *       LIMITATION : words (limbs of large nbs) must be of size
 *                    less than 32 bits (ww < 32 or equal) other-
 *                    wise an error is raised.
 *       *******************************************************
 *
 * This function reads one limb (of size ww-bit) of one large number
 * (which are made of 'w' limbs) from the IP memory of large numbers.
 *
 *   'i' is the index of the large number (there are typically 32 of them)
 *   so typically 'i' should be in the range 0 - 31.
 *   
 *   'j' is the index of the ww-bit limb in the large nb.
 *
 *       The address accessed in the IP memory of large nbs therefore is
 *       (i * n) + j. The value of 'n' is obtained inside the function
 *       from the IP capabilities registers.
 *
 *   'limb' is a pointer where to transfer the value read.
 *
 * If ww > 32, then one limb cannot fit into a single machine word
 * (assuming a 32-bit host). An error is returned (and nothing done)
 * in this case.
 *
 * The value of 'ww' is obtained inside the function by reading
 * the IP debug capabilities registers.
 *
 * Can only be called when in debug mode.
 */
int hw_driver_read_limb_DBG(int32_t i, uint32_t j, uint32_t* limb)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_read_limb(i, j, limb)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Read one large number from the IP memory of large numbers.
 *
 *       LIMITATION : words (limbs of large nbs) must be of size
 *                    less than 32 bits (ww < 32 or equal) other-
 *                    wise an error is raised.
 *       *******************************************************
 *
 * This function reads a complete large nb (made of 'w' words,
 * each of size 'ww'-bit) from the IP memory of large numbers.
 *
 *   'i' is the index of the large nb in memory. There are typically
 *   32 large nbs that the IP can store in its memory (see parameter
 *   'nblargenb' in source file ecc_customize.vhd) hence 'addr'
 *   typically should be in range 0 - 31.
 *
 *   'limbs' is a pointer to a buffer where to transfer the large nb
 *   to read. Therefore it must have an allocated size of at least
 *   'w' limbs. Function will transfer into limb[0] the least signi-
 *   ficant value of the larne nb, and into limb[ww - 1] the most
 *   significant one.
 *
 *   Ex.: Assume the following large number to read from memory is X:
 *
 *            X = 0x_DEAD_0000_0000_0000_0000_0000_0000_BEEF (nn = 128 bits).
 *
 *        Assume ww = 16, then w = 9 (remember that w = ceil( (nn + 4) / ww))
 *        In this case, upon completion ot the function, the caller
 *        will get:
 *
 *           limb[0] = 0xBEEF
 *           ...
 *           limb[7] = 0xDEAD
 *           limb[8] = 0.
 *
 * If ww > 32, then one limb cannot fit into a single machine word
 * (assuming a 32-bit host). An error is returned (and no action
 * taken) in this case.
 *
 * The values of 'w' & 'ww' are obtained inside the function by reading
 * the IP debug capabilities registers.
 *
 * Can only be called when in debug mode.
 */
int hw_driver_read_largenb_DBG(uint32_t i, uint32_t *limbs)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_read_largenb(i, limbs)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Enable XY-shuffling
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_enable_xyshuf(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Call to low-level routine */
	if(ip_ecc_enable_xyshuf()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable XY-shuffling
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_disable_xyshuf_DBG(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	if(ip_ecc_disable_xyshuf()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Enable 'on-the-fly masking of the scalar by AXI interface'
 * countermeasure.
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_enable_aximsk(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Call to low-level routine */
	if(ip_ecc_enable_aximsk()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable 'on-the-fly masking of the scalar by AXI interface'
 * countermeasure.
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_disable_aximsk_DBG(void)
{
	if(driver_setup()){
		goto err;
	}

	/* Call to low-level routine */
	if(ip_ecc_disable_aximsk()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Enable token feature
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_enable_token_DBG()
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_enable_token()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable token feature
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_disable_token_DBG()
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_disable_token()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Attack features: set a specific level of side-channel resistance */
int hw_driver_attack_set_level(int level)
{
	uint32_t jumpop;

	if(driver_setup()){
		goto err;
	}
	
	if ((level != 0) && (level != 1) && (level != 2) && (level != 3)) {
		log_print("In hw_driver_attack_set_level(): only levels 0 (min security) to 3 (max) are defined\n\r");
		goto err;
	}

	switch (level) {

		case 0:
			/* Level 0: minimum security case, i.e NAIVE (non constant time) implementation.
			 *
			 * This is performed through the following steps:
			 *   1. Write register W_ATTACK_CFG (with 0x011 = no-nnrndsf | naive )
			 *   2. Microcode needs to be patched:
			 *        2.1. phi0 and phi1 are cleared w/ NNCLR instead of randomized w/ NNRND
			 *             (this is in adpa.s)
			 *        2.2. kap0 and kap1 need to be 1-bit left-shifted instead of doing the
			 *             TESTPAR that was sampling the first bit of phi
			 *             (this is also in adpa.s)
			 *        2.3. In setup.s, instead of calling .dozdblL, we jump to .zdbl_not_alwaysL
			 *
			 *   3. Furthermore, as side-effect, we must also disable the on-the-fly masking of k
			 *      by the AXI interface.
			 *
			 *   4. Disable the three kappa, kappa' & phi shift-registers
			 */
			/* The 0x21 in the MSPart of the opcode below stands for "J" or simply "jump".
			 * The real proper thing would be to construct unequivocally the opcode word
			 * using a C header file automatically exported from VHDL source (ecc_pkg.vhd).
			 */
			jumpop = 0x21000000UL + ECC_IRAM_ZDBL_NOT_ALWAYS_ADDR; /*0x1fa*/
			/* Step 1 */
			ip_ecc_attack_set_cfg_0(true, false); /* can't generate any error */
			/* Step 2.1 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_RANDOM_PHI0_ADDR /*0x04c*/, 0, 0x51007fea, 1);    /* NNCLR            phi0 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_RANDOM_PHI1_ADDR /*0x04d*/, 0, 0x51007feb, 1);    /* NNCLR            phi1 */
			/* Step 2.2 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_SAMPLE0_KAPLSB_ADDR /*0x073*/, 0, 0x1400300c, 1); /* NNSLL    kap0    kap0 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_SAMPLE1_KAPLSB_ADDR /*0x074*/, 0, 0x1480340d, 1); /* NNSLL,X  kap1    kap1 */
			/* Step 2.3 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_JUMP_DOUBLE_ADDR /*0x083*/, 0, jumpop, 1);        /* J   .zdbl_not_alwaysL */
			/* Step 3 */
			ip_ecc_disable_aximsk(); /* can't generate any error */
			/* Step 4 */
			ip_ecc_attack_disable_nnrndsf(); /* can't generate any error */
			log_print("hw_driver_attack_set_level(): set attack level -0-\n\r");
			/*
			 * Note: in level 0, it still makes sense to activate following
			 *       coutermeasures:
			 *         - periodic Z-remask
			 *             (see API functions hw_driver_enable_zremask_and_set_period()
			 *                              & hw_driver_disable_zremask())
			 *         - memory shuffling
			 *             (see API functions hw_driver_enable_shuffling()
			 *                              & hw_driver_disable_shuffling())
			 *         - blinding
			 *             (see API functions hw_driver_enable_blinding_and_set_size()
			 *                              & hw_driver_disable_blinding())
			 *
			 * so the definition of level 0 is not "unequivocal" by itself.
			 */
			break;

		case 1:
			/* Level 1: constant time execution, the least one should do when considering
			 *          side-channel attacks & physical observation.
			 *
			 * Function calls below are quite understandable when compared w/ what is described
			 * for level 0.
			 */
			/* The 0x26 in the MSPart of the opcode below stands for "JL" or "jump-and-link"
			 * (aka CALL).
			 */
			jumpop = 0x26000000UL + DEBUG_ECC_IRAM_DOZDBL_ADDR; /*0x19b*/
			ip_ecc_disable_aximsk(); /* can't generate any error */
			ip_ecc_attack_set_cfg_0(false, true); /* can't generate any error */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_RANDOM_PHI0_ADDR, 0, 0x51007fea, 1);    /* NNCLR            phi0 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_RANDOM_PHI1_ADDR, 0, 0x51007feb, 1);    /* NNCLR            phi1 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_SAMPLE0_KAPLSB_ADDR, 0, 0x16003022, 1); /* TESTPARs kap0  1 %kap */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_SAMPLE1_KAPLSB_ADDR, 0, 0x00000000, 1); /* NOP                   */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_JUMP_DOUBLE_ADDR, 0, jumpop, 1);        /* JL           .dozdblL */
			ip_ecc_attack_enable_nnrndsf(); /* can't generate any error */
			log_print("hw_driver_attack_set_level(): set attack level -1-\n\r");
			/*
			 * Note: it still makes sense to activate the following coutermeasures:
			 *         - the same 3 as for level 0 (Z-remask, mem shuffling, blinding)
			 *         - XY-shuffling
			 *
			 * so the definition of level 1 neither is "unequivocal" by itself.
			 */
			break;

		case 2:
			/* Level 2: a little bit more hardware security, we implement Anti address-bit DPA.
			 */
			jumpop = 0x26000000UL + DEBUG_ECC_IRAM_DOZDBL_ADDR; /*0x19b*/
			ip_ecc_disable_aximsk(); /* can't generate any error */
			ip_ecc_attack_set_cfg_0(false, true); /* can't generate any error */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_RANDOM_PHI0_ADDR, 0, 0x1500000a, 1);    /* NNRND            phi0 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_RANDOM_PHI1_ADDR, 0, 0x1500000b, 1);    /* NNRND            phi1 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_SAMPLE0_KAPLSB_ADDR, 0, 0x16003022, 1); /* TESTPARs kap0  1 %kap */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_SAMPLE1_KAPLSB_ADDR, 0, 0x00000000, 1); /* NOP                   */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_JUMP_DOUBLE_ADDR, 0, jumpop, 1);        /* JL           .dozdblL */
			log_print("hw_driver_attack_set_level(): set attack level -2-\n\r");
			/*
			 * Note: same as above, the following coutermeasures can still be enabled:
			 *         - the same 4 as for level 1
			 *         - masking of kappa & kappa' using in-logic-fabric shift-registers.
			 *
			 * so the definition of level 2 neither is "unequivocal" by itself.
			 */
			break;

		case 3:
			/* Level 3: quite satisfying hardware security, on-the-fly patching is done
			 *          on instructions of the microcode that manipulate the coordinates
			 *          [XY]R[01] of the two sensitive points, so as to ensure that exec-
			 *          ution of ZADDU & ZADDC routines are perfectly symmetric as regard
			 *          to the address of operands they manipulate (those theoretically
			 *          cannot be used anymore to distinguish between values of the two
			 *          bits kappa_i and kappa'_i during one step of the scalar right-to-
			 *          left loop).
			 */
			jumpop = 0x26000000UL + DEBUG_ECC_IRAM_DOZDBL_ADDR; /*0x19b*/
			ip_ecc_disable_aximsk(); /* can't generate any error */
			ip_ecc_attack_set_cfg_0(false, false); /* can't generate any error */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_RANDOM_PHI0_ADDR, 0, 0x1500000a, 1);    /* NNRND            phi0 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_RANDOM_PHI1_ADDR, 0, 0x1500000b, 1);    /* NNRND            phi1 */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_SAMPLE0_KAPLSB_ADDR, 0, 0x16003022, 1); /* TESTPARs kap0  1 %kap */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_SAMPLE1_KAPLSB_ADDR, 0, 0x00000000, 1); /* NOP                   */
			ip_ecc_patch_one_opcode(DEBUG_ECC_IRAM_JUMP_DOUBLE_ADDR, 0, jumpop, 1);        /* JL           .dozdblL */
			log_print("hw_driver_attack_set_level(): set attack level -3-\n\r");
			/*
			 * Note: same as above, the 5 following coutermeasures can still be enabled
			 * as for level 2, hence definition of level 3 neither is "unequivocal" by
			 * itself.
			 */
			break;

		default: 
			break;
	}

	return 0;
err:
	return -1;
}

/* To get some more capabilties than offered by hw_driver_get_capabilities()
 *
 * (only in debug mode, as some of the infos might give inside to the internal
 * features of the IP which are not at all required for its nominal (secure)
 * use.
 */
int hw_driver_get_more_capabilities_DBG(uint32_t* ww, uint32_t* nbop,
		uint32_t* opsz, uint32_t* rawramsz, uint32_t* irnshw)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_get_more_capabilities(ww, nbop, opsz, rawramsz, irnshw)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* To determine if the IP is currently debug-halted
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_is_debug_halted_DBG(bool* halted)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_is_debug_halted(halted)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Is the IP currently halted on a breakpoint hit?
 * (and if it is, return the breakpoint ID).
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken).
 */
int hw_driver_halted_breakpoint_hit_DBG(bool* hit, uint32_t* id)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_halted_breakpoint_hit(hit, id)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Get the value of Program Counter
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_get_pc_DBG(uint32_t* pc)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_get_pc(pc)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* To get the FSM state the IP is currently in.
 *
 *    The state is returned as a relatively short character string
 *    (copied in 'state') which allocated size must be passed using
 *    'sz'. No more than 'sz' bytes will be copied by the function.
 *    However allow sufficient space, e.g 32 bytes to get enough
 *    info.
 *
 *    See file <ecc_states.h> in folder hdl/common/ecc_curve_iram/
 *    for a list of all state identifiers (this is one of the files
 *    which are automatically generated when running 'make' in that
 *    folder).
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_get_fsm_state_DBG(char* state, uint32_t sz)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_get_fsm_state(state, sz)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* To get value of point-operation time counter.
 *
 *     Each time a point-based operation is started (including [k]P
 *     computation) an internal counter is started and incremented at
 *     each cycle of the main clock. Reading this counter allows to
 *     measure computation duration of point operations.
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_get_time_DBG(uint32_t* clk_cycles)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* call to low-level routine */
	if (ip_ecc_get_time(clk_cycles)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* To assess the production throughput of TRNG raw random bits.
 *
 *     This function measures the time it takes to fill-up
 *     the RAW random FIFO of the TRNG. It first disables
 *     the post-processing function of the TRNG (so that not to
 *     have any read empyting the FIFO), then reset (empty) it,
 *     then poll debug status until it shows that the FIFO is
 *     full. When it is, the value in register R_DBG_RAWDUR
 *     yiels the nb of cycles (of the main clock) the filling-up
 *     process took.
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_get_trng_raw_fifo_filling_time_DBG(uint32_t* duration)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* Call to low level equivalent function */
	if (ip_ecc_get_trng_raw_fifo_filling_time(duration)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Returns the state of the TRNG FIFO of raw random bits.
 *
 *   *full   = true is the FIFO is full
 *   *nbbits = nb of raw random bits currently
 *             buffered in the FIFO.
 *
 * It is recommended to wait for the IP to be in idle state
 * (no running computation) or to be debug-halted before
 * calling the function.
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_get_trng_raw_fifo_state_DBG(bool* full, uint32_t* nbbits)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* Call to low level equivalent function */
	if (ip_ecc_get_trng_raw_fifo_state(full, nbbits)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Read the whole content of the TRNG FIFO of raw random bits
 *
 *   'buf' must point to a buffer whose allocated size must
 *   be sufficient to store the whole content of the FIFO,
 *   the size of which shall be obtained through a call to
 *   function hw_driver_get_more_capabilities_DBG()
 *   (see that function)
 *
 * The function temporarily disables the post-processing unit
 * of TRNG from reading bits from the FIFO. Hence it is strongly
 * advised to wait for the IP to be in idle state (no computation)
 * or debug-halted before calling this funtion.
 *
 * (only allowed in debug mode, otherwise an error
 * is returned, with no action taken)..
 */
int hw_driver_get_content_of_trng_raw_random_fifo_DBG(char* buf, uint32_t* nbbits)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* Call to low level equivalent function */
	if (ip_ecc_get_content_of_trng_raw_random_fifo(buf, nbbits)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* To get an estimation of the frequency of clocks in the IP ('clk' & 'clkmm')
 *
 *     Parameter 'sec' is the time to wait for estimation (in seconds).
 *
 *     The larger the value of 'sec', the better the estimation
 *     (3 seconds is enough, max allowed is 10 (error otherwise)).
 */
int hw_driver_get_clocks_freq_DBG(uint32_t* mhz, uint32_t* mhz_mm, uint32_t sec)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	if (sec > 10) {
		goto err;
	}

	/* Sample the start counters */
	if (ip_ecc_get_clocks_freq(mhz, mhz_mm, sec)) {
		goto err;
	}

	return 0;
err:
	return -1;
}

/* To get all the TRNG diagnostic infos in one API call
 */
int hw_driver_get_trng_diagnostics_DBG(trng_diagcnt_t* tdg)
{
	if(driver_setup()){
		goto err;
	}

	/* Test debug capability */
	if (IPECC_IS_PROD())
	{
		goto err;
	}

	/* Call to low level equivalent function */
	if (ip_ecc_get_trng_diagnostics(tdg)){
		goto err;
	}

	return 0;
err:
	return -1;
}

int hw_driver_attack_enable_nnrndsf(void)
{
	if(driver_setup()){
		goto err;
	}
	if (ip_ecc_attack_enable_nnrndsf()){
		goto err;
	}
	return 0;
err:
	return -1;
}

int hw_driver_attack_disable_nnrndsf(void)
{
	if(driver_setup()){
		goto err;
	}
	if (ip_ecc_attack_disable_nnrndsf()){
		goto err;
	}
	return 0;
err:
	return -1;
}

/* Attack features: set clk & clkmm division & out feature.
 * (set div to 0 to switch off clock 'clk',
 *  set divmm to 0 to switch off clock 'clkmm').
 */
int hw_driver_attack_set_clock_div_out(int div, int divmm) /* set to 0 to switch off clock */
{
	if(driver_setup()){
		goto err;
	}

	if (ip_ecc_attack_set_clock_div_out(div, divmm)){
		goto err;
	}
	return 0;
err:
	return -1;
}

/* Set the curve parameters a, b, p and q.
 *
 * All size arguments (*_sz) must be given in bytes.
 *
 * Please read and take into consideration the 'NOTE:' mentionned
 * at the top of the prototype file <hw_accelerator_driver.h>
 * about the formatting and size of large numbers.
 *
 * Note: if software does not intend to later use the blinding
 * countermeasure, then the order 'q' of the curve is not mandatory
 * (using e.g an arbitrary number for argument 'q' and setting 'q_sz'
 * the same as 'p_sz').
 *
 * Note however that the IP could have been synthesized with an
 * active hardware-locked blinding countermeasure, which, if the IP
 * was further synthesized in production (secure) mode, won't be
 * disengageable by software. In this situation, since every scalar
 * multiplication will be run by the IP with active blinding, 'q'
 * and 'q_sz' arguments should be rigorously set.
 */
int hw_driver_set_curve(const uint8_t *a, uint32_t a_sz, const uint8_t *b, uint32_t b_sz,
       		        const uint8_t *p, uint32_t p_sz, const uint8_t *q, uint32_t q_sz)
{
	if(driver_setup()){
		goto err;
	}
	/* We set the dynamic NN size value to be the max
	 * of P and Q size
	 */
	if(p_sz > q_sz){
		if(ip_ecc_set_nn_bit_size(8 * p_sz)){
			goto err;
		}
	}
	else{
		if(ip_ecc_set_nn_bit_size(8 * q_sz)){
			goto err;
		}
	}

	/* Set a, b, p, q */
	if(ip_ecc_write_bignum(p, p_sz, EC_HW_REG_P)){
		goto err;
	}
	if(ip_ecc_write_bignum(a, a_sz, EC_HW_REG_A)){
		goto err;
	}
	if(ip_ecc_write_bignum(b, b_sz, EC_HW_REG_B)){
		goto err;
	}
	if(ip_ecc_write_bignum(q, q_sz, EC_HW_REG_Q)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Activate the blinding for scalar multiplication.
 *
 * Argument 'blinding_size' must be given in bits, and must be
 * strictly less than the value of 'nn' currently set in the
 * hardware (hance 'nn' - 1 is the largest authorized value).
 *
 * Otherwise error 'ERR_BLN' is raised in R_STATUS register.
 *
 * A value of 0 for input argument 'blinding_size' is counter-
 * intuitive and will be held as meaning to disable the blinding
 * countermeasure (consider using instead  explicit function
 * hw_driver_disable_blinding()).
 */
int hw_driver_enable_blinding_and_set_size(uint32_t blinding_size)
{
	if(driver_setup()){
		goto err;
	}

	if(ip_ecc_enable_blinding_and_set_size(blinding_size)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable the blinding for scalar multiplication.
 */
int hw_driver_disable_blinding(void)
{
	if(driver_setup()){
		goto err;
	}

	if(ip_ecc_disable_blinding()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Activate the shuffling for scalar multiplication */
int hw_driver_enable_shuffling(void)
{
	if(driver_setup()){
		goto err;
	}

	if(ip_ecc_enable_shuffling()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable the shuffling for scalar multiplication */
int hw_driver_disable_shuffling(void)
{
	if(driver_setup()){
		goto err;
	}

	if(ip_ecc_disable_shuffling()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Activate and configure the periodic Z-remasking countermeasure
 * (the 'period' arguement is expressed in number of bits of the scalar */
int hw_driver_enable_zremask_and_set_period(uint32_t period)
{
	if(driver_setup()){
		goto err;
	}

	if(ip_ecc_enable_zremask_and_set_period(period)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Disable the periodic Z-remasking countermeasure for scalar multiplication */
int hw_driver_disable_zremask(void)
{
	if(driver_setup()){
		goto err;
	}

	if(ip_ecc_disable_zremask()){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Check if an affine point (x, y) is on the curve currently defined
 * in the IP (this curve was previously set in the hardware using
 * function hw_driver_set_curve() - c.f above).
 *
 * All size arguments (*_sz) must be given in bytes.
 *
 * Please read and take into consideration the 'NOTE:' mentionned
 * at the top of the prototype file <hw_accelerator_driver.h>
 * about the formatting and size of large numbers.
 */
int hw_driver_is_on_curve(const uint8_t *x, uint32_t x_sz, const uint8_t *y, uint32_t y_sz,
                     	  int *on_curve)
{
	int inf_r0, inf_r1;

	if(driver_setup()){
		goto err;
	}

	/* Preserve our inf flags in a constant time fashion */
	if(ip_ecc_get_r0_inf(&inf_r0)){
		goto err;
	}
	if(ip_ecc_get_r1_inf(&inf_r1)){
		goto err;
	}

	/* Write our R0 register */
	if(ip_ecc_write_bignum(x, x_sz, EC_HW_REG_R0_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y, y_sz, EC_HW_REG_R0_Y)){
		goto err;
	}

	/* Restore our inf flags in a constant time fashion */
	if(ip_ecc_set_r0_inf(inf_r0)){
		goto err;
	}
	if(ip_ecc_set_r1_inf(inf_r1)){
		goto err;
	}

	/* Check if it is on curve */
	if(ip_ecc_exec_command(PT_CHK, on_curve, NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Check if affine points (x1, y1) and (x2, y2) are equal.
 *
 * All size arguments (*_sz) must be given in bytes.
 *
 * Please read and take into consideration the 'NOTE:' mentionned
 * at the top of the prototype file <hw_accelerator_driver.h>
 * about the formatting and size of large numbers.
 */
int hw_driver_eq(const uint8_t *x1, uint32_t x1_sz, const uint8_t *y1, uint32_t y1_sz,
       	    	 const uint8_t *x2, uint32_t x2_sz, const uint8_t *y2, uint32_t y2_sz,
                 int *is_eq)
{
	int inf_r0, inf_r1;

	if(driver_setup()){
		goto err;
	}

	/* Preserve our inf flags in a constant time fashion */
	if(ip_ecc_get_r0_inf(&inf_r0)){
		goto err;
	}
	if(ip_ecc_get_r1_inf(&inf_r1)){
		goto err;
	}

	/* Write our R0 register */
	if(ip_ecc_write_bignum(x1, x1_sz, EC_HW_REG_R0_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y1, y1_sz, EC_HW_REG_R0_Y)){
		goto err;
	}
	/* Write our R1 register */
	if(ip_ecc_write_bignum(x2, x2_sz, EC_HW_REG_R1_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y2, y2_sz, EC_HW_REG_R1_Y)){
		goto err;
	}

	/* Restore our inf flags in a constant time fashion */
	if(ip_ecc_set_r0_inf(inf_r0)){
		goto err;
	}
	if(ip_ecc_set_r1_inf(inf_r1)){
		goto err;
	}

	/* Check if it the points are equal */
	if(ip_ecc_exec_command(PT_EQU, is_eq, NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Check if affine points (x1, y1) and (x2, y2) are opposite.
 *
 * All size arguments (*_sz) must be given in bytes.
 *
 * Please read and take into consideration the 'NOTE:' mentionned
 * at the top of the prototype file <hw_accelerator_driver.h>
 * about the formatting and size of large numbers.
 */
int hw_driver_opp(const uint8_t *x1, uint32_t x1_sz, const uint8_t *y1, uint32_t y1_sz,
                  const uint8_t *x2, uint32_t x2_sz, const uint8_t *y2, uint32_t y2_sz,
               	  int *is_opp)
{
	int inf_r0, inf_r1;

	if(driver_setup()){
		goto err;
	}

	/* Preserve our inf flags in a constant time fashion */
	if(ip_ecc_get_r0_inf(&inf_r0)){
		goto err;
	}
	if(ip_ecc_get_r1_inf(&inf_r1)){
		goto err;
	}

	/* Write our R0 register */
	if(ip_ecc_write_bignum(x1, x1_sz, EC_HW_REG_R0_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y1, y1_sz, EC_HW_REG_R0_Y)){
		goto err;
	}
	/* Write our R1 register */
	if(ip_ecc_write_bignum(x2, x2_sz, EC_HW_REG_R1_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y2, y2_sz, EC_HW_REG_R1_Y)){
		goto err;
	}

	/* Restore our inf flags in a constant time fashion */
	if(ip_ecc_set_r0_inf(inf_r0)){
		goto err;
	}
	if(ip_ecc_set_r1_inf(inf_r1)){
		goto err;
	}


	/* Check if the points are opposite */
	if(ip_ecc_exec_command(PT_OPP, is_opp, NULL)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Check if the infinity point flag is set in the hardware for
 * point at index idx.
 *
 * Argument 'index' must be either 0, identifying point R0, or 1,
 * identifying point R1.
 */
int hw_driver_point_iszero(uint8_t idx, int *iszero)
{
	if(driver_setup()){
		goto err;
	}

	switch(idx){
		case 0:{
			/* Get our R0 register infinity flag */
			if(ip_ecc_get_r0_inf(iszero)){
				goto err;
			}
			break;
		}
		case 1:{
			/* Get our R1 register infinity flag */
			if(ip_ecc_get_r1_inf(iszero)){
				goto err;
			}
			break;
		}
		default:{
			/* Index not supported */
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/* Set the infinity point flag in the hardware for point
 * at index idx. This tells hardware to hold the corresponding
 * point (R0 or R1) as being the null point (aka point at infinity).
 *
 * Any values of the affine coordinates the hardware was currently
 * holding for that point will become irrelevant, either they were
 * resulting from a previous computation or transmitted by software.
 *
 * Argument 'index' must be either 0, identifying point R0, or 1,
 * identifying point R1.
 */
int hw_driver_point_zero(uint8_t idx)
{
	if(driver_setup()){
		goto err;
	}

	switch(idx){
		case 0:{
			/* Write our R0 register infinity flag */
			if(ip_ecc_set_r0_inf(1)){
				goto err;
			}
			break;
		}
		case 1:{
			/* Write our R1 register infinity flag */
			if(ip_ecc_set_r1_inf(1)){
				goto err;
			}
			break;
		}
		default:{
			/* Index not supported */
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/* Unset the infinity point flag in the hardware for
 * point at index idx. This tells hardware to hold the corresponding
 * point (R0 or R1) as NOT being the null point (aka point at infinity).
 *
 * The affine coordinates the hardware was currently holding for that
 * point will then become relevant, either they were resulting from a
 * previous computation or transmitted by software.
 *
 * Further note that transmitting coordinates to the hardware for one
 * of particular points R0 or R1 (using ip_ecc_write_bignum()) automatically
 * set that point as being not null, just as hw_driver_point_unzero()
 * would do.
 *
 * Argument 'index' must be either 0, identifying point R0, or 1,
 * identifying point R1.
 */
int hw_driver_point_unzero(uint8_t idx)
{
	if(driver_setup()){
		goto err;
	}

	switch(idx){
		case 0:{
			/* Write our R0 register infinity flag */
			if(ip_ecc_set_r0_inf(0)){
				goto err;
			}
			break;
		}
		case 1:{
			/* Write our R1 register infinity flag */
			if(ip_ecc_set_r1_inf(0)){
				goto err;
			}
			break;
		}
		default:{
			/* Index not supported */
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/* Return (out_x, out_y) = -(x, y) i.e the opposite of the
 * intput point.
 *
 * All size arguments (*_sz) must be given in bytes.
 *
 * Please read and take into consideration the 'NOTE:' mentionned
 * at the top of the prototype file <hw_accelerator_driver.h>
 * about the formatting and size of large numbers.
 */
int hw_driver_neg(const uint8_t *x, uint32_t x_sz, const uint8_t *y, uint32_t y_sz,
                  uint8_t *out_x, uint32_t *out_x_sz, uint8_t *out_y, uint32_t *out_y_sz)
{
	int inf_r0, inf_r1;
	uint32_t nn_sz;

	if(driver_setup()){
		goto err;
	}

	/* Preserve our inf flags in a constant time fashion */
	if(ip_ecc_get_r0_inf(&inf_r0)){
		goto err;
	}
	if(ip_ecc_get_r1_inf(&inf_r1)){
		goto err;
	}

	/* Write our R0 register */
	if(ip_ecc_write_bignum(x, x_sz, EC_HW_REG_R0_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y, y_sz, EC_HW_REG_R0_Y)){
		goto err;
	}

	/* Restore our inf flags in a constant time fashion */
	if(ip_ecc_set_r0_inf(inf_r0)){
		goto err;
	}
	if(ip_ecc_set_r1_inf(inf_r1)){
		goto err;
	}

	/* Execute our NEG command */
	if(ip_ecc_exec_command(PT_NEG, NULL, NULL)){
		goto err;
	}

	/* Get back the result from R1 */
	nn_sz = ip_ecc_nn_bytes_from_bits_sz(ip_ecc_get_nn_bit_size());
	if(((*out_x_sz) < nn_sz) || ((*out_y_sz) < nn_sz)){
		goto err;
	}
	(*out_x_sz) = (*out_y_sz) = nn_sz;
	if(ip_ecc_read_bignum(out_x, (*out_x_sz), EC_HW_REG_R1_X)){
		goto err;
	}
	if(ip_ecc_read_bignum(out_y, (*out_y_sz), EC_HW_REG_R1_Y)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Return (out_x, out_y) = 2 * (x, y), i.e the double of the
 * input point.
 *
 * All size arguments (*_sz) must be given in bytes.
 *
 * Please read and take into consideration the 'NOTE:' mentionned
 * at the top of the prototype file <hw_accelerator_driver.h>
 * about the formatting and size of large numbers.
 */
int hw_driver_dbl(const uint8_t *x, uint32_t x_sz, const uint8_t *y, uint32_t y_sz,
                  uint8_t *out_x, uint32_t *out_x_sz, uint8_t *out_y, uint32_t *out_y_sz)
{
	int inf_r0, inf_r1;
	uint32_t nn_sz;

	if(driver_setup()){
		goto err;
	}

	/* Preserve our inf flags in a constant time fashion */
	if(ip_ecc_get_r0_inf(&inf_r0)){
		goto err;
	}
	if(ip_ecc_get_r1_inf(&inf_r1)){
		goto err;
	}

	/* Write our R0 register */
	if(ip_ecc_write_bignum(x, x_sz, EC_HW_REG_R0_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y, y_sz, EC_HW_REG_R0_Y)){
		goto err;
	}

	/* Restore our inf flags in a constant time fashion */
	if(ip_ecc_set_r0_inf(inf_r0)){
		goto err;
	}
	if(ip_ecc_set_r1_inf(inf_r1)){
		goto err;
	}

	/* Execute our DBL command */
	if(ip_ecc_exec_command(PT_DBL, NULL, NULL)){
		goto err;
	}

	/* Get back the result from R1 */
	nn_sz = ip_ecc_nn_bytes_from_bits_sz(ip_ecc_get_nn_bit_size());
	if(((*out_x_sz) < nn_sz) || ((*out_y_sz) < nn_sz)){
		goto err;
	}
	(*out_x_sz) = (*out_y_sz) = nn_sz;
	if(ip_ecc_read_bignum(out_x, (*out_x_sz), EC_HW_REG_R1_X)){
		goto err;
	}
	if(ip_ecc_read_bignum(out_y, (*out_y_sz), EC_HW_REG_R1_Y)){
		goto err;
	}

	return 0;
err:
	return -1;
}


/* Return (out_x, out_y) = (x1, y1) + (x2, y2), i.e perform addition
 * of the two input points.
 *
 * All size arguments (*_sz) must be given in bytes.
 *
 * Please read and take into consideration the 'NOTE:' mentionned
 * at the top of the prototype file <hw_accelerator_driver.h>
 * about the formatting and size of large numbers.
 */
int hw_driver_add(const uint8_t *x1, uint32_t x1_sz, const uint8_t *y1, uint32_t y1_sz,
                  const uint8_t *x2, uint32_t x2_sz, const uint8_t *y2, uint32_t y2_sz,
                  uint8_t *out_x, uint32_t *out_x_sz, uint8_t *out_y, uint32_t *out_y_sz)
{
	int inf_r0, inf_r1;
	uint32_t nn_sz;

	if(driver_setup()){
		goto err;
	}

	/* Preserve our inf flags in a constant time fashion */
	if(ip_ecc_get_r0_inf(&inf_r0)){
		goto err;
	}
	if(ip_ecc_get_r1_inf(&inf_r1)){
		goto err;
	}

	/* Write our R0 register */
	if(ip_ecc_write_bignum(x1, x1_sz, EC_HW_REG_R0_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y1, y1_sz, EC_HW_REG_R0_Y)){
		goto err;
	}
	/* Write our R1 register */
	if(ip_ecc_write_bignum(x2, x2_sz, EC_HW_REG_R1_X)){
		goto err;
	}
	if(ip_ecc_write_bignum(y2, y2_sz, EC_HW_REG_R1_Y)){
		goto err;
	}

	/* Restore our inf flags in a constant time fashion */
	if(ip_ecc_set_r0_inf(inf_r0)){
		goto err;
	}
	if(ip_ecc_set_r1_inf(inf_r1)){
		goto err;
	}

	/* Execute our ADD command */
	if(ip_ecc_exec_command(PT_ADD, NULL, NULL)){
		goto err;
	}

	/* Get back the result from R1 */
	nn_sz = ip_ecc_nn_bytes_from_bits_sz(ip_ecc_get_nn_bit_size());
	if(((*out_x_sz) < nn_sz) || ((*out_y_sz) < nn_sz)){
		goto err;
	}
	(*out_x_sz) = (*out_y_sz) = nn_sz;
	if(ip_ecc_read_bignum(out_x, (*out_x_sz), EC_HW_REG_R1_X)){
		goto err;
	}
	if(ip_ecc_read_bignum(out_y, (*out_y_sz), EC_HW_REG_R1_Y)){
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Return (out_x, out_y) = scalar * (x, y), i.e perform the scalar 
 * multiplication of the input point by the input scalar.
 *
 * All size arguments (*_sz) must be given in bytes.
 *
 * Please read and take into consideration the 'NOTE:' mentionned
 * at the top of the prototype file <hw_accelerator_driver.h>
 * about the formatting and size of large numbers.
 */
int hw_driver_mul(const uint8_t *x, uint32_t x_sz, const uint8_t *y, uint32_t y_sz,
                  const uint8_t *scalar, uint32_t scalar_sz,
                  uint8_t *out_x, uint32_t *out_x_sz, uint8_t *out_y, uint32_t *out_y_sz,
									kp_trace_info_t* ktrc, uint32_t* zmask)
{
	int inf_r0, inf_r1;
	uint32_t nn_sz;

	/* 32768 bits are more than enough for any practical
	 * use of elliptic curve cryptography.
	 */
	uint8_t token[4096] = {0, }; /* Heck, a whole page? Yes indeed. */

	if(driver_setup()){
		log_print("In hw_driver_mul(): Error in driver_setup()\n\r");
		goto err;
	}

	/* Nb of bytes corresponding to current value of 'nn' in the IP.
	 */
	nn_sz = ip_ecc_nn_bytes_from_bits_sz(ip_ecc_get_nn_bit_size());

	/* Check that the current value of 'nn' does not exceed the size
	 * allocated to the token on the stack.
	 */
	if(ip_ecc_nn_bytes_from_bits_sz(ip_ecc_get_nn_bit_size()) > 4096){
		log_print("In hw_driver_mul(): Error in ip_ecc_nn_bytes_from_bits_sz()\n\r");
		goto err;
	}

	/* Preserve our inf flags in a constant time fashion */
	if(ip_ecc_get_r0_inf(&inf_r0)){
		log_print("In hw_driver_mul(): Error in ip_ecc_get_r0_inf()\n\r");
		goto err;
	}
	if(ip_ecc_get_r1_inf(&inf_r1)){
		log_print("In hw_driver_mul(): Error in ip_ecc_get_r1_inf()\n\r");
		goto err;
	}

	/* Get the random one-shot token */
	if (ip_ecc_get_token(token, nn_sz)){
		log_print("In hw_driver_mul(): Error in ip_ecc_get_token()\n\r");
		goto err;
	}

	/* Write our scalar register with the scalar k */
	if(ip_ecc_write_bignum(scalar, scalar_sz, EC_HW_REG_SCALAR)){
		log_print("In hw_driver_mul(): Error in ip_ecc_write_bignum()\n\r");
		goto err;
	}
	/* Write our R1 register with the point to be multiplied */
	if(ip_ecc_write_bignum(x, x_sz, EC_HW_REG_R1_X)){
		log_print("In hw_driver_mul(): Error in ip_ecc_write_bignum()\n\r");
		goto err;
	}
	if(ip_ecc_write_bignum(y, y_sz, EC_HW_REG_R1_Y)){
		log_print("In hw_driver_mul(): Error in ip_ecc_write_bignum()\n\r");
		goto err;
	}

	/* Restore our inf flags in a constant time fashion */
	if(ip_ecc_set_r0_inf(inf_r0)){
		log_print("In hw_driver_mul(): Error in ip_ecc_set_r0_inf()\n\r");
		goto err;
	}
	if(ip_ecc_set_r1_inf(inf_r1)){
		log_print("In hw_driver_mul(): Error in ip_ecc_set_r1_inf()\n\r");
		goto err;
	}

	/* Execute our [k]P command */
	if(ip_ecc_exec_command(PT_KP, NULL, ktrc, zmask)){
		log_print("In hw_driver_mul(): Error in ip_ecc_exec_command()\n\r");
		goto err;
	}

	/* Get back the result from R1 */
	if(((*out_x_sz) < nn_sz) || ((*out_y_sz) < nn_sz)){
		log_print("In hw_driver_mul(): *out_x_sz = %d\n\r", *out_x_sz);
		log_print("In hw_driver_mul(): *out_y_sz = %d\n\r", *out_y_sz);
		log_print("In hw_driver_mul(): nn_sz = %d\n\r", nn_sz);
		log_print("In hw_driver_mul(): Error in sizes' comparison\n\r");
		goto err;
	}
	(*out_x_sz) = (*out_y_sz) = nn_sz;
	if(ip_ecc_read_bignum(out_x, (*out_x_sz), EC_HW_REG_R1_X)){
		log_print("In hw_driver_mul(): Error in ip_ecc_read_bignum()\n\r");
		goto err;
	}
	if(ip_ecc_read_bignum(out_y, (*out_y_sz), EC_HW_REG_R1_Y)){
		log_print("In hw_driver_mul(): Error in ip_ecc_read_bignum()\n\r");
		goto err;
	}

	/* Unmask the [k]P result coordinates with the one-shot token */
	if (ip_ecc_unmask_with_token(out_x, (*out_x_sz), token, nn_sz, out_x, out_x_sz)) {
		log_print("In hw_driver_mul(): Error in ip_ecc_unmask_with_token()\n\r");
		goto err;
	}
	if (ip_ecc_unmask_with_token(out_y, (*out_y_sz), token, nn_sz, out_y, out_y_sz)) {
		log_print("In hw_driver_mul(): Error in ip_ecc_unmask_with_token()\n\r");
		goto err;
	};

	/* Clear the token */
	ip_ecc_clear_token(token, nn_sz);

	return 0;
err:
	return -1;
}

/* Set the small scalar size in the hardware.
 *
 * The 'small scalar size' feature is provided by the IP in order
 * to provide a computation speed-up for really small scalar.
 *
 * This is a "one shot" feature, meaning the 'nn' parameter is
 * still recorded by the IP and will become applicable again
 * as soon as the scalar multiplication following the call to
 * function hw_driver_set_small_scalar_size() is done.
 *
 * Hence hw_driver_set_small_scalar_size() must be called
 * each time the feature is needed.
 *
 * Obviously this feature only concerns the scalar multiplication.
 * */
int hw_driver_set_small_scalar_size(uint32_t bit_sz)
{
	if(driver_setup()){
		goto err;
	}

	/* NOTE: sanity check on this size should be performed by
	 * the hardware (e.g. is this size exceeds the nn size, and
	 * so on). So no need to sanity check anything here. */
	IPECC_SET_SMALL_SCALAR_SIZE(bit_sz);

	return 0;
err:
	return -1;
}

/**********************************************************/

#else
/*
 * Dummy definition to avoid the empty translation unit ISO C warning
 */
typedef int dummy;
#endif /* WITH_EC_HW_ACCELERATOR */

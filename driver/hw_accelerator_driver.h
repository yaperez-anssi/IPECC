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

#ifndef __HW_ACCELERATOR_DRIVER_H__
#define __HW_ACCELERATOR_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>

#if defined(WITH_EC_HW_ACCELERATOR)

/* Hardware/external accelerator driver abstraction
 *
 * NOTE: big numbers are in BIG ENDIAN format, and their size is in bytes. No particular
 * hypothesis must be taken on the address or size alignment of the buffers, or on the zero padding.
 *
 * For instance, the representation of the big number 0xabcdef can be either { 0xab, 0xcd, 0xef } on three
 * bytes, or {0x00, 0x00, 0xab, 0xcd, 0xef } on five bytes.
 */

#ifdef KP_TRACE
#include <stdbool.h>
#endif

/* Supported command types */
typedef enum {  
        PT_ADD = 0,
        PT_DBL = 1,
        PT_CHK = 2,
        PT_EQU = 3,
        PT_OPP = 4,
        PT_KP  = 5,
        PT_NEG = 6,
} ip_ecc_command;

/**********************
 *    Nominal API     *
 **********************/

/* Reset the hardware */
int hw_driver_reset(void);

/* To know if the IP is in 'HW secure' or 'HW unsecure' mode */
int hw_driver_is_hwunsecure(bool*);
int hw_driver_is_hwsecured(bool*);

/* Get all three version nbs of the IP (major, minor & patch) */
int hw_driver_get_version_tags(uint32_t*, uint32_t*, uint32_t*);

/* Set the curve parameters a, b, p and q */
int hw_driver_set_curve(const uint8_t *a, uint32_t a_sz, const uint8_t *b, uint32_t b_sz,
			const uint8_t *p, uint32_t p_sz, const uint8_t *q, uint32_t q_sz);

/* Activate the blinding for scalar multiplication */
int hw_driver_enable_blinding_and_set_size(uint32_t blinding_size);

/* Disable the blinding for scalar multiplication */
int hw_driver_disable_blinding(void);

/* Activate the shuffling for scalar multiplication */
int hw_driver_enable_shuffling(void);

/* Disable the shuffling for scalar multiplication */
int hw_driver_disable_shuffling(void);

/* Activate and configure the periodic Z-remasking countermeasure
 * (the 'period' argument is expressed in number of bits of the scalar */
int hw_driver_enable_zremask_and_set_period(uint32_t period);

/* Disable the periodic Z-remasking countermeasure for scalar multiplication */
int hw_driver_disable_zremask(void);

/* Debug feature: disable the XY-shuffling countermeasure */
int hw_driver_disable_xyshuf_DBG(void);

/* Debug feature: re-enable the XY-shuffling countermeasure */
int hw_driver_enable_xyshuf(void);

/* Check if an affine point (x, y) is on the curve that has been previously set in the hardware */
int hw_driver_is_on_curve(const uint8_t *x, uint32_t x_sz, const uint8_t *y, uint32_t y_sz,
			  int *on_curve);

/* Check if affine points (x1, y1) and (x2, y2) are equal */
int hw_driver_eq(const uint8_t *x1, uint32_t x1_sz, const uint8_t *y1, uint32_t y1_sz,
		 const uint8_t *x2, uint32_t x2_sz, const uint8_t *y2, uint32_t y2_sz,
		 int *is_eq);

/* Check if affine points (x1, y1) and (x2, y2) are opposite */
int hw_driver_opp(const uint8_t *x1, uint32_t x1_sz, const uint8_t *y1, uint32_t y1_sz,
		  const uint8_t *x2, uint32_t x2_sz, const uint8_t *y2, uint32_t y2_sz,
		  int *is_opp);

/* Check if the infinity point flag is set in the hardware for
 * point at index idx
 */
int hw_driver_point_iszero(uint8_t idx, int *iszero);

/* Set the infinity point flag in the hardware for
 * point at index idx
 */
int hw_driver_point_zero(uint8_t idx);

/* Unset the infinity point flag in the hardware for
 * point at index idx
 */
int hw_driver_point_unzero(uint8_t idx);

/* Return (out_x, out_y) = -(x, y) */
int hw_driver_neg(const uint8_t *x, uint32_t x_sz, const uint8_t *y, uint32_t y_sz,
		  uint8_t *out_x, uint32_t *out_x_sz, uint8_t *out_y, uint32_t *out_y_sz);

/* Return (out_x, out_y) = 2 * (x, y) */
int hw_driver_dbl(const uint8_t *x, uint32_t x_sz, const uint8_t *y, uint32_t y_sz,
                  uint8_t *out_x, uint32_t *out_x_sz, uint8_t *out_y, uint32_t *out_y_sz);


/* Return (out_x, out_y) = (x1, y1) + (x2, y2) */
int hw_driver_add(const uint8_t *x1, uint32_t x1_sz, const uint8_t *y1, uint32_t y1_sz,
		  const uint8_t *x2, uint32_t x2_sz, const uint8_t *y2, uint32_t y2_sz,
                  uint8_t *out_x, uint32_t *out_x_sz, uint8_t *out_y, uint32_t *out_y_sz);

#ifdef KP_TRACE
typedef struct {
	uint32_t r0z;
	uint32_t r1z;
	uint32_t kap;
	uint32_t kapp;
	uint32_t zu;
	uint32_t zc;
	uint32_t jnbbit;
} kp_exp_flags_t;
#endif

/* The following 'kp_trace_info' structure allows any calling program (stat. linked with
 * the driver) to get a certain number of IP internal states/infos collected during a [k]P
 * computation through breakpoints and step-by-step execution (this includes e.g values of
 * a few random numbers/masks, coordinates of intermediate points, etc).
 */
typedef struct {
	/* Main security parameter nn */
	uint32_t nn;
	/* Random values (along with a valig flag for each) */
	uint32_t* lambda;
	bool lambda_valid;
	uint32_t* phi0;
	bool phi0_valid;
	uint32_t* phi1;
	bool phi1_valid;
	uint32_t* alpha;
	bool alpha_valid;
	uint32_t* kap0msk;
	bool kap0msk_valid;
	uint32_t* kap1msk;
	bool kap1msk_valid;
	uint32_t* kapP0msk;
	bool kapP0msk_valid;
	uint32_t* kapP1msk;
	bool kapP1msk_valid;
	uint32_t* phi0msk;
	bool phi0msk_valid;
	uint32_t* phi1msk;
	bool phi1msk_valid;
	/* Nb of trace steps (roughly the nb of opcodes for this [k]P run) */
	uint32_t nb_steps;
	/* Temporary value of XR0, YR0, XR1 and YR1 */
	uint32_t* nb_xr0;
	uint32_t* nb_yr0;
	uint32_t* nb_xr1;
	uint32_t* nb_yr1;
	uint32_t* nb_zr01;
	/* A huge char buffer to printf all required infos. */
	char* msg;
	uint32_t msgsz;
	uint32_t msgsz_max;
} kp_trace_info_t;

typedef struct {
	/* "AXI" */
	uint32_t aximin;
	uint32_t aximax;
	uint32_t axiok;
	uint32_t axistarv;
	/* "EFP" */
	uint32_t efpmin;
	uint32_t efpmax;
	uint32_t efpok;
	uint32_t efpstarv;
	/* "CRV" */
	uint32_t crvmin;
	uint32_t crvmax;
	uint32_t crvok;
	uint32_t crvstarv;
	/* "SHF" */
	uint32_t shfmin;
	uint32_t shfmax;
	uint32_t shfok;
	uint32_t shfstarv;
	/* "RAW" */
	uint32_t rawmin;
	uint32_t rawmax;
	uint32_t rawok;
	uint32_t rawstarv;
} trng_diagcnt_t;

/* The size of the statically allocated buffer that field
 * 'msgsz_max' of struct 'kp_trace_info_t' above should not
 * exceed. */
#define KP_TRACE_PRINTF_SZ   (16*1024*1024)    /* 16 MB */

/* Return (out_x, out_y) = scalar * (x, y) */
int hw_driver_mul(const uint8_t *x, uint32_t x_sz, const uint8_t *y, uint32_t y_sz,
		  const uint8_t *scalar, uint32_t scalar_sz,
		  uint8_t *out_x, uint32_t *out_x_sz, uint8_t *out_y, uint32_t *out_y_sz,
			uint32_t* kp_time, uint32_t* zmask, kp_trace_info_t* ktrc);

/* Set the small scalar size in the hardware */
int hw_driver_set_small_scalar_size(uint32_t bit_sz);

/* To get hardware capabilities from the IP */
int hw_driver_get_capabilities(bool* secure, bool* shuffle, bool* nndyn, bool* axi64, uint32_t* nnmax);

/* To determine if the IP is in mode "HW unsecure"
 *
 * Watch-out/reminder: this is not a dynamic mode but a static one:
 * IP was synthesized either with 'hwsecure' = TRUE or 'hwsecure' = FALSE
 * in static config file ecc_customize.vhd.
 *
 * These two functions allow the calling software to know if the IP was
 * synthesized with 'hwsecure' = FALSE or not.
 */
int hw_driver_is_hw_secure(bool*);   /* if HW secure, the bool. pted to by arg-ptr is set to TRUE. */
int hw_driver_is_hw_unsecure(bool*); /* if HW secure, the bool. pted to by arg-ptr is set to FALSE. */

/******************
 *    DEBUG API   *    (meaning: IP was synthesized with 'hwsecure' = FALSE in ecc_customize.vhd)
 *******************/

/* To halt the IP - This freezes execution of microcode
 * (only in HW unsecure mode) */
int hw_driver_halt_DBG(void);

/* To set and activate a new breakpoint
 * (only in HW unsecure mode) */
int hw_driver_set_breakpoint_DBG(uint32_t, uint32_t);

/* To remove/disable a breakpoint
 * (only in HW unsecure mode) */
int hw_driver_remove_breakpoint_DBG(uint32_t);

/* Have IP to execute a certain nb of opcodes in microcode
 * (only in HW unsecure mode).
 */
int hw_driver_run_opcodes_DBG(uint32_t);

/* Same as hw_driver_run_opcodes_DBG() but single step
 */
int hw_driver_single_step_DBG(void);

/* Resume execution of microcode
 * (only in HW unsecure mode).
 */
int hw_driver_resume_DBG(void);

/* Arm trigger function
 * (only in HW unsecure mode)
 */
int hw_driver_arm_trigger_DBG(void);

/* Disarm/disable trigger function
 * (only in HW unsecure mode)
 */
int hw_driver_disarm_trigger_DBG(void);

/* Set configuration of UP trigger detection
 * (only in HW unsecure mode)
 */
int hw_driver_set_trigger_up_DBG(uint32_t);

/* Set configuration of DOWN trigger detection
 * (only in HW unsecure mode)
 */
int hw_driver_set_trigger_down_DBG(uint32_t);

/* Patch a single opcode in the microcode.
 * (only in HW unsecure mode).
 */
int hw_driver_patch_one_opcode_DBG(uint32_t, uint32_t, uint32_t, uint32_t);

/* Patch a portion or the whole of microcode image
 * (only in HW unsecure mode).
 */
int hw_driver_patch_microcode_DBG(uint32_t*, uint32_t, uint32_t);

/* Set configuration of TRNG
 * (only in HW unsecure mode)
 */
int hw_driver_configure_trng_DBG(int, uint32_t, uint32_t);

/* Reset the raw random bits FIFO
 * (only in HW unsecure mode)
 */
int hw_driver_reset_trng_raw_fifo_DBG(void);

/* Reset all the internal random number FIFOs
 * (only in HW unsecure mode)
 */
int hw_driver_reset_trng_irn_fifos_DBG(void);

/* Enable the TRNG post-processing logic
 *
 * Note: a call is required if the IP is in Debug mode, otherwise
 *       the TRNG won't ever provide a single byte to its post-processing
 *       function (see driver_setup() function in source file
 *       hw_accelereator_driver_ipecc.c, which enforces this action).
 *
 * (only in HW unsecure mode)
 */
int hw_driver_trng_post_proc_enable_DBG(void);

/* Disable the TRNG post-processing logic
 * (only in HW unsecure mode)
 */
int hw_driver_trng_post_proc_disable_DBG(void);

/* Complete bypass the TRNG function (both entropy source,
 * post-processing, and server)
 * (only in HW unsecure mode)
 */
int hw_driver_bypass_full_trng_DBG(uint32_t bit);

/* Remove complete bypass the TRNG function & restore normal behaviour
 *
 * (only in HW unsecure mode)
 */
int hw_driver_dont_bypass_trng_DBG();

/* Force a deterministic effect (all 1's) of the NNRND instruction
 */
int hw_driver_nnrnd_deterministic_DBG(void);

/* Restore nominal action of NNRND instruction, undoing action of
 * previous hw_driver_nnrnd_deterministic_DBG().
 */
int hw_driver_nnrnd_not_deterministic_DBG(void);

/* To select the random source of which to read the diagnostics
 * (only in HW unsecure mode).
 */
int hw_driver_select_trng_diag_source_DBG(uint32_t);

/* Get one bit from the raw random FIFO
 * (only in HW unsecure mode).
 */
int hw_driver_read_one_raw_random_bit_DBG(uint32_t, uint32_t*);

/* Write one word in the IP memory of large numbers.
 */
int hw_driver_write_word_in_lgnbmem_DBG(uint32_t, uint32_t);

/* Write one limb of one large nb in the IP memory of large numbers.
 */
int hw_driver_write_limb_DBG(int32_t, uint32_t, uint32_t);

/* Write one large number in the IP memory of large numbers.
 */
int hw_driver_write_largenb_DBG(uint32_t, uint32_t*);

/* Read one word from the IP memory of large numbers.
 */
int hw_driver_read_word_from_lgnbmem_DBG(uint32_t, uint32_t*);

/* Read one limb of one large nb from the IP memory of large numbers.
 */
int hw_driver_read_limb_DBG(int32_t, uint32_t, uint32_t*);

/* Read one large number from the IP memory of large numbers.
 */
int hw_driver_read_largenb_DBG(uint32_t, uint32_t*);

/* Enable XY-shuffling
 * (only in HW unsecure mode)
 */
int hw_driver_enable_xyshuf(void);

/* Disable XY-shuffling
 * (only in HW unsecure mode) */
int hw_driver_disable_xyshuf_DBG(void);

/* Enable 'on-the-fly masking of the scalar by AXI interface'
 * countermeasure.
 * (only in HW unsecure mode).
 */
int hw_driver_enable_aximsk(void);

/* Disable 'on-the-fly masking of the scalar by AXI interface'
 * countermeasure.
 * (only in HW unsecure mode).
 */
int hw_driver_disable_aximsk_DBG(void);

/* Enable token feature
 * (only in HW unsecure mode).
 */
int hw_driver_enable_token_DBG(void);

/* Disable token feature
 * (only in HW unsecure mode).
 */
int hw_driver_disable_token_DBG(void);

/* To get some more capabilties than offered by hw_driver_get_capabilities()
 * (only in HW unsecure mode).
 */
int hw_driver_get_more_capabilities_DBG(uint32_t*, uint32_t*, uint32_t*, uint32_t*, uint32_t*);

/* To determine if the IP is currently debug-halted
 * (only in HW unsecure mode)
 */
int hw_driver_is_debug_halted_DBG(bool*);

/* Is the IP currently halted on a breakpoint hit?
 * (and if it is, return the breakpoint ID).
 * (only in HW unsecure mode)
 */
int hw_driver_halted_breakpoint_hit_DBG(bool*, uint32_t*);

/* Get the value of Program Counter.
 * (only in HW unsecure mode).
 */
int hw_driver_get_pc_DBG(uint32_t*);

/* To get the FSM state the IP is currently in.
 * (only in HW unsecure mode).
 */
int hw_driver_get_fsm_state_DBG(char*, uint32_t);

/* To get value of point-operation time counter.
 * (only in HW unsecure mode).
 */
int hw_driver_get_time_DBG(uint32_t*);

/* To assess the production throughput of TRNG raw random bits.
 * (only in HW unsecure mode).
 */
int hw_driver_get_trng_raw_fifo_filling_time_DBG(uint32_t*);

/* Returns the state of the TRNG FIFO of raw random bits
 * (is it full and nb of currently stored bits).
 * (only in HW unsecure mode).
 */
int hw_driver_get_trng_raw_fifo_state_DBG(bool*, uint32_t*);

/* Read the whole content of the TRNG FIFO of raw random bits
 * (only in HW unsecure mode).
 */
int hw_driver_get_content_of_trng_raw_random_fifo_DBG(char*, uint32_t*);

/* To estimate from software the IP clock frequenciies
 * (only in HW unsecure mode).
 */
int hw_driver_get_clocks_freq_DBG(uint32_t*, uint32_t*, uint32_t);

/* To get all the TRNG diagnostic infos in one API call
 * (only in HW unsecure mode) */
int hw_driver_get_trng_diagnostics_DBG(trng_diagcnt_t*);

/* Attack features: set a specific level of side-channel resistance */
int hw_driver_attack_set_level(int level);

/* Attack features: enable countermeasure using hardware shift-regs to mask kappa & kappa' */
int hw_driver_attack_enable_nnrndsf(void);

/* Attack features: disable countermeasure using hardware shift-regs to mask kappa & kappa' */
int hw_driver_attack_disable_nnrndsf(void);

/* Attack features: clocks division & out feature */
int hw_driver_attack_set_clock_div_out(int, int);

/*
 * Error/printf formating
 */
#ifdef TERM_CTRL_AND_COLORS
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KORA  "\033[93m"
#define KUNK  "\033[91m"
#define KVIO  "\033[38;5;199m"
#define KERASELINE "\033[2K"
#define KMVUP1LINE "\033[1A"
#define KBOLD "\033[1m"
#define KNOBOLD "\033[22m"
#define KCURSORVIS "\033[?25h"
#define KCURSORINVIS "\033[?25l"
#else
#define KNRM  ""
#define KRED  ""
#define KGRN  ""
#define KYEL  ""
#define KBLU  ""
#define KMAG  ""
#define KCYN  ""
#define KWHT  ""
#define KORA  ""
#define KUNK  "\033[91m"
#define KVIO  ""
#define KERASELINE ""
#define KMVUP1LINE ""
#define KBOLD ""
#define KNOBOLD ""
#define KCURSORVIS ""
#define KCURSORINVIS ""
#endif /* TERM_COLORS */

#define KERR  KUNK
#define KINF  KORA

#endif /* !WITH_EC_HW_ACCELERATOR */

#endif /* __HW_ACCELERATOR_DRIVER_H__ */

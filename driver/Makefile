# VHD_DIR must be set with the ABSOLUTE path where IPECC microcode
# has been built
VHD_DIR=../hdl/common/ecc_curve_iram

# CONFIG #############
ARM_CC ?= arm-linux-gnueabihf-gcc
CFLAGS = -Wall -Wextra -Wpedantic -Wno-stringop-truncation -O3 -g3 -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -static
CFLAGS += -DWITH_EC_HW_DEBUG

# ####################################################################################################
# Additional compilation flags (some experimental)
#
# Comment the following line if your terminal doesn't support ANSI Escape Codes
# (to colorize and control cursor movement on screen)
CFLAGS += -DTERM_CTRL_AND_COLORS
#
# Uncomment the first following line to get [k]P trace debug feature
# (this is obviously a highly unsecure feature that is only available if the IP
# was synthesized in HW unsecure mode).
# Also uncomment the second following line if you want the debug trace infos to be
# printed on-the-fly to the console (if you have one on your hw target), otherwise
# these infos are simply sprintf'ed into a buffer.
# Note: if compiling with -DKP_TRACE_CONSOLE you MUST also compile with -DKP_TRACE
#       so uncomment the 2nd line only if also uncomment the first one.
#CFLAGS += -DKP_TRACE
#CFLAGS += -DKP_TRACE_CONSOLE
#
# Uncomment the first following line to force the first random Z-masking value (this is
# experimental)
# Also uncomment the second following line to check that the Z-mask you wish to set has
# actually replace the random one.
#
# Note 1: if compiling with -DKP_CHECK_ZMASK you MUST also compile with -DKP_SET_ZMASK
#         so uncomment the 2nd line only if also uncomment the first one.
#
# Note 2: for now one can't both have debug-trace and the Z-mask setting, so either use
#         -DKP_TRACE or -DKP_SET_ZMASK (you won't get any error, simply the KP_TRACE will
#         take precedence).
#CFLAGS += -DKP_SET_ZMASK
#CFLAGS += -DKP_CHECK_ZMASK
#
# ####################################################################################################


C_FILES = hw_accelerator_driver_ipecc_platform.c hw_accelerator_driver_ipecc.c
C_FILES_LINUX = $(C_FILES) linux/ecc-test-linux.c linux/curve.c linux/kp.c linux/ptops.c linux/pttests.c
C_FILES_STDOL = $(C_FILES) stdalone/ecc-test-stdl.c


# TARGETS ############
all: ecc-test-linux-uio ecc-test-linux-devmem ecc-test-stdalone


$(VHD_DIR)/ecc_addr.h $(VHD_DIR)/ecc_vars.h $(VHD_DIR)/ecc_states.h $(VHD_DIR)/ecc_platform.h:
	@if [ -z "$(VHD_DIR)" ] ; then \
		echo -e "\033[93m *Error*: makefile variable VHD_DIR is not defined.\033[0m" ; \
		echo -e "\033[92m\033[4mSolution\033[24m: Edit ./Makefile and set VHD_DIR to point to the folder path\033[0m" ; \
		echo -e "\033[92m          where the IPECC microcode was built. Alternatively you can\033[0m" ; \
		echo -e "\033[92m          inline definition e.g 'make VHD_DIR=', but whatever the way\033[0m" ; \
		echo -e "\033[92m          don't use a relative path, use only an absolute one.\033[0m" ; \
		exit -1; \
		fi
	@if [ ! -x $(VHD_DIR) ] ; then \
		echo -e "\033[93m*Error*: makefile variable VHD_DIR points to a folder without access permissions.\033[0m" ; \
		echo -e "\033[38;5;226m\033[4mDetails\033[24m: Folder '$(VHD_DIR)' doesn't have the 'x' permission\033[0m" ; \
		exit -1; \
		fi
	@for h in ecc_addr.h ecc_vars.h ecc_states.h ecc_platform.h ; do \
		if [ ! -r $(VHD_DIR)/$${h} ] ; then \
			cd $(VHD_DIR) && $(MAKE) -C ./ && cd -; \
		fi \
	done

ecc-test-linux-uio: $(VHD_DIR)/ecc_addr.h $(VHD_DIR)/ecc_vars.h $(VHD_DIR)/ecc_states.h $(VHD_DIR)/ecc_platform.h $(C_FILES_LINUX) linux/ecc-test-linux.h
	$(ARM_CC) $(CFLAGS) -I$(VHD_DIR) -DWITH_EC_HW_ACCELERATOR -DWITH_EC_HW_UIO $(C_FILES_LINUX) -o ecc-test-linux-uio

ecc-test-linux-devmem: $(VHD_DIR)/ecc_addr.h $(VHD_DIR)/ecc_vars.h $(VHD_DIR)/ecc_states.h $(VHD_DIR)/ecc_platform.h $(C_FILES_LINUX) linux/ecc-test-linux.h
	$(ARM_CC) $(CFLAGS) -I$(VHD_DIR) -DWITH_EC_HW_ACCELERATOR -DWITH_EC_HW_DEVMEM $(C_FILES_LINUX) -o ecc-test-linux-devmem

ecc-test-stdalone: $(VHD_DIR)/ecc_addr.h $(VHD_DIR)/ecc_vars.h $(VHD_DIR)/ecc_states.h $(VHD_DIR)/ecc_platform.h $(C_FILES_STDOL) stdalone/ecc-test-stdl.h
	$(ARM_CC) $(CFLAGS) -I$(VHD_DIR) -DWITH_EC_HW_ACCELERATOR -DWITH_EC_HW_STANDALONE $(C_FILES_STDOL) -o ecc-test-stdalone

clean:
	@rm -f ecc-test-linux-uio ecc-test-linux-devmem ecc-test-stdalone

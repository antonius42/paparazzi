# Hey Emacs, this is a -*- makefile -*-
#
# AHRS_H_X
# AHRS_H_Y
# AHRS_H_Z
#

USE_MAGNETOMETER ?= 1
AHRS_ALIGNER_LED ?= none

AHRS_CFLAGS  = -DUSE_AHRS
AHRS_CFLAGS += -DUSE_AHRS_ALIGNER

ifeq (,$(findstring $(USE_MAGNETOMETER),0 FALSE))
  AHRS_CFLAGS += -DUSE_MAGNETOMETER
endif

ifneq ($(AHRS_ALIGNER_LED),none)
  AHRS_CFLAGS += -DAHRS_ALIGNER_LED=$(AHRS_ALIGNER_LED)
endif

AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_quat_wrapper.h\"
AHRS_SRCS   += subsystems/ahrs.c
AHRS_SRCS   += subsystems/ahrs/ahrs_int_cmpl_quat.c
AHRS_SRCS   += subsystems/ahrs/ahrs_int_cmpl_quat_wrapper.c
AHRS_SRCS   += subsystems/ahrs/ahrs_aligner.c

ap.CFLAGS += $(AHRS_CFLAGS)
ap.srcs += $(AHRS_SRCS)

nps.CFLAGS += $(AHRS_CFLAGS)
nps.srcs += $(AHRS_SRCS)

test_ahrs.CFLAGS += $(AHRS_CFLAGS)
test_ahrs.srcs += $(AHRS_SRCS)

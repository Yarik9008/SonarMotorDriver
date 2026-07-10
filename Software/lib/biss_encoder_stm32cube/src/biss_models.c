/**
 * @file biss_models.c
 */

#include "biss_encoder/biss_models.h"

const biss_frame_cfg_t BISS_LENZ_IRS_17BIT = {
    .frame_bytes      = 6U,
    .scd_bits         = 32U,
    .position_bits    = 24U,
    .resolution_bits  = 17U,
    .crc_bits         = 6U,
    .error_ok_high    = 1U,
    .warning_ok_high  = 1U,
};

const biss_frame_cfg_t BISS_LENZ_IRS_18BIT = {
    .frame_bytes      = 6U,
    .scd_bits         = 32U,
    .position_bits    = 24U,
    .resolution_bits  = 18U,
    .crc_bits         = 6U,
    .error_ok_high    = 1U,
    .warning_ok_high  = 1U,
};

/**
 * @file biss_models.h
 * @brief Пресеты конфигурации кадра для семейства LENZ IRS.
 */

#ifndef BISS_ENCODER_MODELS_H
#define BISS_ENCODER_MODELS_H

#include "biss_encoder/biss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** LENZ IRS-I34 / I50 / I60 — 17 бит (131072 counts/rev). */
extern const biss_frame_cfg_t BISS_LENZ_IRS_17BIT;

/** LENZ IRS-I70 / I80 / I90 — 18 бит (262144 counts/rev). */
extern const biss_frame_cfg_t BISS_LENZ_IRS_18BIT;

#ifdef __cplusplus
}
#endif

#endif /* BISS_ENCODER_MODELS_H */

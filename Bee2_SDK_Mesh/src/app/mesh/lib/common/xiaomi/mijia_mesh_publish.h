/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mijia_mesh_publish.h
* @brief    Head file for mijia mesh publish
* @details  Data types and external functions declaration.
* @author   hector_huang
* @date     2019-09-09
* @version  v1.0
* *************************************************************************************
*/
#ifndef _MIJIA_MESH_PUBLISH_H_
#define _MIJIA_MESH_PUBLISH_H_

#include "mesh_api.h"

BEGIN_DECLS

/**
 * @addtogroup MIJIA_MESH_PUBLISH
 * @{
 */


/**
 * @defgroup MIJIA_MESH_PUBLISH_API mijia mesh publish api
 * @brief Functions declaration
 * @{
 */

/**
 * @brief initialize mijia mesh publish
 * @retval TRUE: initialize success
 * @retval FALSE: initialize failed
 */
bool mi_publish_init(void);

/**
 * @brief set model publish parameters
 * @param[in] pmodel: model need to set publish parameter
 * @param[in] address: publish address
 * @param[in] app_key_idx_g: app global key index used in publication
 */
void mi_model_publish_set(mesh_model_t *pmodel, uint16_t address, uint16_t app_key_idx_g);

/**
 * @brief start periodic publish
 */
bool mi_publish_start(void);

/**
 * @brief stop periodic publish
 */
void mi_publish_stop(void);

/**
 * @brief set periodic publsih interval
 * @param interval: publish interval, unit is minutes
 */
void mi_publish_interval_set(uint32_t interval);

/**
 * @brief start model publish timer
 * @param[in] pmodel_info: pointer to model information context that need to publish
 * @retval TRUE: start model publish success
 * @retval FALSE: start model publish failed
 */
bool mi_publish_single_start(mesh_model_info_t *pmodel_info);

/**
 * @brief process publish tick timeout
 */
void mi_process_publish_single_timeout(plt_timer_t ptimer);

/**
 * @brief process publish timeout
 */
void mi_process_publish_timeout(void);

/** @} */
/** @} */


END_DECLS


#endif /* _GENERIC_TRANSITION_TIME_H_ */



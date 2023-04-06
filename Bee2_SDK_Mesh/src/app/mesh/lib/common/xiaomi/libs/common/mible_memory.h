/*
 * mible_memory.h
 *
 *  Created on: 2021��3��8��
 *      Author: mi
 */

#ifndef MIJIA_BLE_LIBS_COMMON_MIBLE_MEMORY_H_
#define MIJIA_BLE_LIBS_COMMON_MIBLE_MEMORY_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "mible_log.h"

void *mible_malloc(size_t size);
void mible_free(void *ptr);

#endif /* MIJIA_BLE_LIBS_COMMON_MIBLE_MEMORY_H_ */

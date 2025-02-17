/*
 * eventlog.h
 *
 *  Created on: 14 февр. 2025 г.
 *      Author: bryki
 */

#ifndef INC_EVENTLOG_H_
#define INC_EVENTLOG_H_

#include <stddef.h>
#include <stdint.h>

typedef enum eventlog_status_t
{
    LOG_OK = 0, // 0: Successful
    LOG_NO_MEM,
    LOG_ERROR
} eventlog_status_t;

#define EVENTLOG_CACHE_SIZE     256

eventlog_status_t eventlog_init(uint16_t entry_size);
eventlog_status_t eventlog_write(void *entry);
eventlog_status_t eventlog_read(uint32_t entry_num, void *entry);
eventlog_status_t eventlog_flush(void);

uint32_t eventlog_getFreeMemory(void);
uint32_t eventlog_getTotalMemory(void);
#endif /* INC_EVENTLOG_H_ */

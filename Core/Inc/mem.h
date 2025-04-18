/**
 ************************************************************************************************
 * @file    mem.h
 * @brief   part of project "hc-fluo"
 * @author  Zaikin Denis (ZD), Smirnov Alexander (SA)
 * @copyright Copyright (C) 2023
 ************************************************************************************************
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------------*/
typedef enum mem_status_t
{
    MEM_OK = 0, // 0: Successful
    MEM_ERROR,  // 1: R/W Error
    MEM_WRPRT,  // 2: Write Protected
    MEM_NOTRDY, // 3: Not Ready / Busy
    MEM_PARERR  // 4: Invalid Parameter
} mem_status_t;

typedef enum mem_ioctl_cmd_t
{
    MEM_IOCTL_CTRL_SYNC = 0, // Complete pending write process
    MEM_IOCTL_PAGES_COUNT,   // Get page size
    MEM_IOCTL_PAGE_SIZE,     // Get page size
    MEM_IOCTL_SECTOR_SIZE,   // Get erase block size
    MEM_IOCTL_TOTAL_SIZE,
    MEM_IOCTL_BURN,          // Write chache to memory
    MEM_IOCTL_ERASE_CHIP,
	MEM_IOCTL_POWERDOWN,
	MEM_IOCTL_RELEASE
} mem_ioctl_cmd_t;

/* ---------------------------------------------------------------------------------------------------------*/

mem_status_t mem_init();
mem_status_t mem_status();
mem_status_t mem_read_page(uint8_t* buff, uint32_t page, size_t count);
mem_status_t mem_write_page(uint8_t* buff, uint32_t page, size_t count);
mem_status_t mem_ioctl(mem_ioctl_cmd_t cmd, void* data);
mem_status_t mem_deinit();

/* ---------------------------------------------------------------------------------------------------------*/

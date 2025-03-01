/*
 * eventlog.c
 *
 *  Created on: 14 февр. 2025 г.
 *      Author: bryki
 */
#include "eventlog.h"
#include "mem.h"
#include <stdio.h>
#include <string.h>

typedef struct eventlog_t
{
    uint32_t entry_size;
    uint32_t max_entries;
    uint32_t next_free_ptr;
    uint32_t page_size;
} eventlog_t;

eventlog_t event_log = {0};
uint8_t eventlog_cache[EVENTLOG_CACHE_SIZE]; // память не динамическая, поэтому статический буфер с заранее известным размером создал


eventlog_status_t eventlog_init(uint16_t entry_size)
{
    eventlog_status_t status = LOG_ERROR;
    uint32_t i;
    uint16_t cache_ptr;
    uint32_t page_ptr;
    uint32_t *ptr32;

    if(mem_init() == MEM_OK)
    {
        // вычисляем сколько записей влезет в память
        mem_ioctl(MEM_IOCTL_TOTAL_SIZE, &i);

        event_log.entry_size = entry_size;
        event_log.max_entries = i/event_log.entry_size;
        event_log.next_free_ptr = 0;

        mem_ioctl(MEM_IOCTL_PAGE_SIZE, &event_log.page_size);
        // ищем в памяти первую незанятую ячейку (какое условие свободности?)
        for(event_log.next_free_ptr = 0; event_log.next_free_ptr < event_log.max_entries; )
        {
            // вычисляем индексы на следующую пустую область в кэше и на страницу
            page_ptr = event_log.next_free_ptr / event_log.page_size;

            mem_read_page(eventlog_cache, page_ptr, 1);

            for(cache_ptr = 0; cache_ptr < event_log.page_size; cache_ptr += event_log.entry_size)
            {
                // ну такое себе условие свободности
                ptr32 = (uint32_t *)(&eventlog_cache[cache_ptr]);
                if(*ptr32 == 0xFFFFFFFF)
                    return LOG_OK;

                event_log.next_free_ptr += event_log.entry_size;
            }
        }

        status = LOG_NO_MEM;
    }

    return status;
}

eventlog_status_t eventlog_write(void *entry)
{
    // по заполнению памяти ничего не делаем (по новой не пишем)
    if(event_log.next_free_ptr >= event_log.max_entries)
        return LOG_NO_MEM;

    // вычисляем индексы на следующую пустую область в кэше и на страницу
    uint16_t cache_ptr = event_log.next_free_ptr % event_log.page_size;
    uint32_t page_ptr = event_log.next_free_ptr / event_log.page_size;

    //если указатель на данные в кэше 0, то там ничего нет,
    // пожтому читаем данные в кэш (нужен, т.к. минимум можно записать 256 байт в w25)
    if(cache_ptr == 0)
    {
        mem_read_page(eventlog_cache, page_ptr, 1);
    }

    // копируем в кэш новую запись
    memcpy(eventlog_cache + cache_ptr, (uint8_t *)entry, event_log.entry_size);

    cache_ptr += event_log.entry_size;

    // как только кэш страницы заполняется, пишем в память
    if(cache_ptr >= event_log.page_size)
    {
        mem_write_page((uint8_t *)eventlog_cache, page_ptr, 1);
    }

    event_log.next_free_ptr += event_log.entry_size;

    return LOG_OK;
}

eventlog_status_t eventlog_read(uint32_t entry_num, void *entry)
{
    uint32_t page_ptr = entry_num / 256;
    uint32_t entry_ptr = entry_num % 256;

    mem_read_page(eventlog_cache, page_ptr, 1);

    memcpy((uint8_t *)entry, &eventlog_cache[entry_ptr], 16);

    return LOG_OK;
}

eventlog_status_t eventlog_flush(void)
{
    event_log.next_free_ptr = 0;

    //erase all
    return (mem_ioctl(MEM_IOCTL_ERASE_CHIP, 0) == MEM_OK) ? LOG_OK : LOG_ERROR;
}

uint32_t eventlog_getFreeMemory(void)
{
    return event_log.next_free_ptr;
}

uint32_t eventlog_getTotalMemory(void)
{
    return event_log.max_entries;
}

#if 0
uint8_t board_GetNextFreeEntry(void)
{
    uint8_t found = 0;
    uint16_t i;
    //поиск свободной запсиси
    for(i = 0; (i < 16384) && (!found); i++)
    {
        mem_read_page((uint8_t *)&cache, i, 1);

        for(entry_ptr = 0; entry_ptr < 16; entry_ptr++)
        {
            if(cache.entries[entry_ptr].crc16 == 0xFFFF)
            {
                eeprom_page_ptr = i;
                found = 1;
                break;
            }
        }
    }

    return found;
}

uint8_t board_WriteEntry(eeprom_entry_t *entry)
{
    if(eeprom_page_ptr == 16384)
        return 0;

    if(entry_ptr == 0)
    {
        mem_read_page((uint8_t *)&cache, eeprom_page_ptr, 1);
    }

    memcpy(&(cache.entries[entry_ptr++]), entry, sizeof(eeprom_entry_t));

    if(entry_ptr == 16)
    {
        if(eeprom_page_ptr < 16384)
        {
            mem_write_page((uint8_t *)&cache, eeprom_page_ptr, 1);
            //write
            eeprom_page_ptr++;
        }
        entry_ptr = 0;
    }

    return 1;
}
#endif

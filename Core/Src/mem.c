/**
 ************************************************************************************************
 * @file    mem.c
 * @brief   part of project "hc-fluo"
 * @author  Zaikin Denis (ZD), Smirnov Alexander (SA)
 * @copyright Copyright (C) 2023
 *
 *
 *
 * ** ZD25Q64B **
 * 64M Ver.B SPI NOR FLASH
 *
 * https://datasheet.lcsc.com/lcsc/2106070104_Zetta-ZD25Q64BSIGT_C2687407.pdf
 *
 ************************************************************************************************
 */
#include "main.h"
#include "mem.h"
#include "spi.h"

#include <stdbool.h>
#include <string.h>

/* ---------------------------------------------------------------------------------------------------------*/
typedef struct mem_dev_t
{
    uint8_t manufact_id; // Manufacturer ID
    uint8_t device_id;   // Device ID
    uint8_t capacity_id; // Capacity ID
    uint8_t memory_id;   // Memory ID
    //
    uint32_t flash_size;      // Total size in bytes
    uint16_t page_size;       // Min program size in bytes
    uint16_t sector_size;     // Min erase size in bytes
    uint16_t pages_in_sector; // Pages quantity in sector
    uint16_t timeout;         // Operation timeout in ms
} mem_dev_t;

/* ---------------------------------------------------------------------------------------------------------*/
typedef enum mem_cmd_t
{
    MEM_CMD_WRITE_EN       = 0x06, // Write Enable
    MEM_CMD_WRITE_EN_VSR   = 0x50, // Write Enable For Volatile Status Register
    MEM_CMD_WRITE_DIS      = 0x04, // Write Disable
    MEM_CMD_READ_SR1       = 0x05, // Read Status Register-1
    MEM_CMD_READ_SR2       = 0x35, // Read Status Register-2
    MEM_CMD_WRITE_SR1      = 0x01, // Write Status Register-1
    MEM_CMD_WRITE_SR2      = 0x31, // Write Status Register-2
    MEM_CMD_READ_DATA      = 0x03, // Read Data
    MEM_CMD_READ_FAST_DATA = 0x0B, // Fast Read Data
    MEM_CMD_WRITE_PAGE     = 0x02, // Page Program
    MEM_CMD_ERASE_SECTOR   = 0x20, // Sector Erase(4KB)
    MEM_CMD_ERASE_BLOCK32  = 0x52, // Block Erase(32KB)
    MEM_CMD_ERASE_BLOCK64  = 0xD8, // Block Erase(64KB)
    MEM_CMD_ERASE_CHIP     = 0x60, // Chip Erase
    MEM_CMD_READ_JEDEC     = 0x9f, // Read JEDEC ID
} mem_cmd_t;

typedef union mem_sr1_t
{
    struct
    {
        bool busy : 1;  /* Erase or Write in Progress */
        bool wel : 1;   /* Write Enable Latch */
        uint8_t bp : 3; /* Block Protect */
        bool tb : 1;    /* Top/Bottom Write Protect  */
        bool sec : 1;   /* Sector Protect */
        bool srp0 : 1;  /* Status Register Protect 0 */
    };
    uint8_t reg;
} mem_sr1_t;

/* ---------------------------------------------------------------------------------------------------------*/

static const mem_dev_t dev_table[] = {
    // ZD25Q64B
    {
        .manufact_id = 0xBA,
        .device_id   = 0x16,
        .capacity_id = 0x17,
        .memory_id   = 0x32,
        //
        .flash_size      = 0x800000,
        .page_size       = 0x100,
        .sector_size     = 0x1000,
        .pages_in_sector = 16,
        .timeout         = 1000,
    },

    {
        .manufact_id = 0xEF,
        .device_id   = 0x00,
        .capacity_id = 0x16,
        .memory_id   = 0x40,
        //
        .flash_size      = 0x400000,
        .page_size       = 0x100,
        .sector_size     = 0x1000,
        .pages_in_sector = 16,
        .timeout         = 1000,
    }
};

static const mem_dev_t* dev = NULL;

/* ---------------------------------------------------------------------------------------------------------*/

static volatile uint32_t mem_is_init = 0;

static volatile int32_t current_sector = -1;
static volatile bool sector_modify     = false;
static uint8_t mem_cache[ 1024 * 4 ]   = {0};

/* ---------------------------------------------------------------------------------------------------------*/
static mem_status_t mem_erase_sector(uint32_t addr);
static mem_status_t mem_read_data(uint32_t addr, uint8_t* data, size_t len);
static mem_status_t mem_write_data(uint32_t addr, uint8_t* data, size_t len);
static mem_status_t mem_wait_rdy(uint32_t timeout);

static mem_status_t mem_send_cmd(mem_cmd_t cmd);
static mem_status_t mem_status_unsafe();

static mem_status_t mem_update_cache(int32_t new_sector);
static mem_status_t mem_operation(uint8_t* buf, uint32_t page, size_t count, bool write);

static inline void mem_delay(uint32_t ms) { HAL_Delay(ms); }

/* ---------------------------------------------------------------------------------------------------------*/
void __attribute__((weak)) mem_mux_create() {}
void __attribute__((weak)) mem_mux_take() {}
void __attribute__((weak)) mem_mux_give() {}

/* ---------------------------------------------------------------------------------------------------------*/

mem_status_t mem_init()
{
    if (mem_is_init > 0)
    {
        mem_is_init++;
        return MEM_OK;
    }

    spi_cs_deactivate();
    spi_init();
    mem_mux_create();

    mem_status_t status = MEM_ERROR;

    // Read JEDEC ID
    mem_mux_take();
    {
        spi_cs_activate();
        {
            uint8_t jdec[ 4 ] = {MEM_CMD_READ_JEDEC, 0, 0, 0};

            if (spi_tx_rx(jdec, jdec, 4) == SPI_OK)
            {
                for (uint32_t i = 0; i < sizeof(dev_table) / sizeof(mem_dev_t); i++) //-V1008
                {
                    if (jdec[ 1 ] == dev_table[ i ].manufact_id &&                   //
                        jdec[ 2 ] == dev_table[ i ].memory_id &&                     //
                        jdec[ 3 ] == dev_table[ i ].capacity_id)
                    {
                        dev    = &dev_table[ i ];
                        status = MEM_OK;
                        break;
                    }
                }
            }
        }
        spi_cs_deactivate();
    }
    mem_mux_give();

    if (status == MEM_OK) mem_is_init++;

    return status;
}

mem_status_t mem_status()
{
    mem_mux_take();
    mem_status_t status = mem_status_unsafe();
    mem_mux_give();
    return status;
}

mem_status_t mem_read_page(uint8_t* buff, uint32_t page, size_t count)
{
    if (mem_is_init == 0) return MEM_ERROR;
    if ((page + count) > (dev->flash_size / dev->page_size) || (buff == NULL)) return MEM_PARERR;

    mem_mux_take();
    //mem_status_t status = mem_operation(buff, page, count, false);
    mem_status_t status = mem_read_data(page * dev->page_size, buff, dev->page_size);
    mem_mux_give();
    return status;
}

mem_status_t mem_write_page(uint8_t* buff, uint32_t page, size_t count)
{
    if (mem_is_init == 0) return MEM_ERROR;
    if ((page + count) > (dev->flash_size / dev->page_size) || (buff == NULL)) return MEM_PARERR;

    mem_mux_take();
    //mem_status_t status = mem_operation(buff, page, count, true);
    mem_status_t status = mem_write_data(page * dev->page_size, buff, dev->page_size);
    mem_mux_give();
    return status;
}

mem_status_t mem_ioctl(mem_ioctl_cmd_t cmd, void* data)
{
    if (mem_is_init == 0) return MEM_ERROR;

    mem_status_t status = MEM_OK;

    mem_mux_take();
    {
        switch (cmd)
        {
            case MEM_IOCTL_SECTOR_SIZE: *(( uint32_t* )data) = dev->sector_size; break;
            case MEM_IOCTL_PAGES_COUNT: *(( uint32_t* )data) = dev->flash_size / dev->page_size; break;
            case MEM_IOCTL_PAGE_SIZE: *(( uint32_t* )data) = dev->page_size; break;
            case MEM_IOCTL_TOTAL_SIZE: *(( uint32_t* )data) = dev->flash_size; break;

            case MEM_IOCTL_BURN: status = mem_update_cache(current_sector); break;

            case MEM_IOCTL_CTRL_SYNC:
            default: break;
        }
    }
    mem_mux_give();

    return status;
}

mem_status_t mem_deinit()
{
    mem_mux_take();
    {
        if (mem_is_init > 0) mem_is_init--;
    }
    mem_mux_give();
    return MEM_OK;
}

/* ---------------------------------------------------------------------------------------------------------*/

static mem_status_t mem_update_cache(int32_t sector)
{
    mem_status_t status = MEM_OK;

    if (sector_modify && (sector != -1))
    {
        status = mem_erase_sector(current_sector * dev->sector_size);

        for (size_t i = 0; i < dev->pages_in_sector && status == MEM_OK; i++)
        {
            uint32_t page_offset = i * dev->page_size;
            if (mem_write_data((current_sector * dev->sector_size) + page_offset, // address
                               mem_cache + page_offset,                           // data
                               dev->page_size) != MEM_OK)
            {
                status = MEM_ERROR;
                break;
            }
        }
        sector_modify = false;
    }

    if (status == MEM_OK && (current_sector != sector))
    {
        current_sector = sector;

        status = mem_read_data(sector * dev->sector_size, mem_cache, dev->sector_size);
    }
    return status;
}


static mem_status_t __attribute__((optimize("O0"))) mem_operation(uint8_t* buf, uint32_t page, size_t count, bool write)
{
    if (mem_is_init == 0) return MEM_ERROR;
    if ((page + count) > (dev->flash_size / dev->page_size) || (buf == NULL)) return MEM_PARERR;

    mem_status_t status = MEM_OK;

    int32_t sectorn = page / dev->pages_in_sector; // number operating sector

    for (size_t npage = 0; npage < count && status == MEM_OK;)
    {
        //  if sector has changed
        if (current_sector != sectorn) status = mem_update_cache(sectorn);

        // page number in sector
        uint32_t pn_in_sector = page & (dev->pages_in_sector - 1);

        // pages count for operation
        uint32_t pagec_op = (count - npage);

        pagec_op = (pn_in_sector + pagec_op) > dev->pages_in_sector //
                       ? (dev->pages_in_sector - pn_in_sector)      //
                       : pagec_op;

        size_t cahce_n   = dev->page_size * pn_in_sector; // byte number of cahce
        size_t buff_n    = dev->page_size * npage;        // byte number of buffer
        size_t data_size = dev->page_size * pagec_op;     // bytes quantity for operation

        if (write)                                        //-V1051
        {
            sector_modify = true;
            memcpy(&mem_cache[ cahce_n ], &buf[ buff_n ], data_size);
        }
        else
        {
            memcpy(&buf[ buff_n ], &mem_cache[ cahce_n ], data_size); //
        }

        // pages of next sector if nessary
        page += pagec_op;
        npage += pagec_op;
        sectorn++;
    }

    return status;
}

static mem_status_t mem_status_unsafe()
{
    if (mem_is_init == 0) return MEM_ERROR;

    mem_status_t status = MEM_OK;

    spi_cs_activate();
    {
        uint8_t data[ 2 ] = {[0] = MEM_CMD_READ_SR1}; //-V1009
        if (spi_tx_rx(data, data, 2) == SPI_OK)
        {
            mem_sr1_t sr1 = {.reg = data[ 1 ]};

            if (sr1.sec || sr1.tb || sr1.bp) { status = MEM_WRPRT; }
            else if (sr1.busy) { status = MEM_NOTRDY; }
        }
    }
    spi_cs_deactivate();

    return status;
}

static mem_status_t mem_send_cmd(mem_cmd_t cmd)
{
    uint8_t data[] = {cmd};
    mem_status_t status;

    spi_cs_activate();
    {
        status = spi_tx(data, 1) == SPI_OK ? MEM_OK : MEM_ERROR;
    }
    spi_cs_deactivate();

    return status == MEM_OK ? mem_wait_rdy(dev->timeout) : status;
}

static mem_status_t mem_read_data(uint32_t addr, uint8_t* data, size_t len)
{
    if (addr > dev->flash_size || data == NULL) { return MEM_PARERR; }

    mem_status_t status = MEM_OK;

    spi_cs_activate();
    {
        uint32_t cmd = __REV(addr) | MEM_CMD_READ_DATA;
        if (spi_tx(&cmd, sizeof(cmd)) != SPI_OK || // send address
            spi_rx(data, len) != SPI_OK)           // read data
        {
            status = MEM_ERROR;
        }
    }
    spi_cs_deactivate();

    return status;
}

static mem_status_t mem_write_data(uint32_t addr, uint8_t* data, size_t len)
{
    if (addr > dev->flash_size || data == NULL || len > 256) { return MEM_PARERR; }
    // needed before write
    mem_status_t status = mem_send_cmd(MEM_CMD_WRITE_EN);

    spi_cs_activate();
    {
        uint32_t cmd = __REV(addr) | MEM_CMD_WRITE_PAGE;
        if (status != MEM_OK ||                    // write enable
            spi_tx(&cmd, sizeof(cmd)) != SPI_OK || // send address
            spi_tx(data, len) != SPI_OK)           // write data
        {
            status = MEM_ERROR;
        }
    }
    spi_cs_deactivate();

    status = mem_wait_rdy(dev->timeout);

    return status;
}

static mem_status_t mem_erase_sector(uint32_t addr)
{
    if (addr > dev->flash_size) { return MEM_PARERR; }

    mem_status_t status = mem_send_cmd(MEM_CMD_WRITE_EN);

    spi_cs_activate();
    {
        uint32_t cmd = __REV(addr) | MEM_CMD_ERASE_SECTOR;
        if (status != MEM_OK || // write enable
            spi_tx(&cmd, 4) != SPI_OK)
        {
            status = MEM_ERROR;
        }
    }
    spi_cs_deactivate(); // Erase page on cs deactivation

    status = mem_wait_rdy(dev->timeout);

    return status;
}

static mem_status_t mem_wait_rdy(uint32_t timeout)
{
    mem_status_t status = MEM_NOTRDY;

    while (status == MEM_NOTRDY && timeout)
    {
        mem_delay(1);
        status = mem_status_unsafe();
        timeout--;
    }

    return status;
}

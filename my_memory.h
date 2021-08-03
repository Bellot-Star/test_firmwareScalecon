#ifndef _MYMEMORY_H
#define _MYMEMORY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_fstorage_sd.h"

/**< flash base address : changable in range of (0x3e000 ~ 0x10000) >*/
#define FSDB_BASE_ADDR  0x42000

/**< header size : do not change >*/
#define FSDB_HDR_SIZE   4096

/**< body address : do not change >*/
#define FSDB_BODY_ADDR  FSDB_BASE_ADDR + FSDB_HDR_SIZE

/**< the number of pages : storage size = PAGECNT * PAGESIZE = 132kB >*/
#define FSDB_PAGECNT    33
#define FSDB_PAGESIZE   4096

/**< block size : writable unit size >*/
#define FSDB_BLOCKSIZE  4

/**< track size : data unit size to be stored in memory (ADC data * CHANNELS = 40B) >*/
#define FSDB_TRACKSIZE  40

/**< the numbers of tracks : track size should be a multiple of PAGESIZE ( PAGESIZE / TRACKSIZE = 102 ) >*/
#define FSDB_TRACKCNT   102

typedef enum
{
    DATA_FMT_NONE = 0,
    DATA_FMT_HEX = 'h',
    DATA_FMT_STR = 's'
} data_fmt_t;

/**< seeking address : readable and writable seeking address from FSDB_BODY_ADDR >*/
extern uint32_t g_page_addr;

/**< initialize functions >*/
void init_fstorage();

/**< erase database of flash storage : page_addr = 0, erase all data in body area >*/
void reset_fsdb_contents();

void fstorage_write(uint32_t addr, uint8_t* data, uint32_t len);
void fstorage_erase(uint32_t addr, uint32_t pages_cnt);
void fstorage_read(uint32_t addr, uint32_t len, uint8_t* pData, data_fmt_t fmt);

/**< read seeking address from header >*/
void read_fsdb_harder();

/**< print body data on log console >*/
void print_fsdb_body(uint32_t addr, uint32_t len);

/**< print header info on log console >*/
void print_fsdb_header();

#endif
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_error.h"
#include "app_util.h"

#include "my_memory.h"
#include "nrf_pwr_mgmt.h"

uint32_t g_page_addr = 0;

static uint8_t txbuf[255] = {0};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x3e000,
//    .end_addr   = 0x3ffff,
    .end_addr   = 0xfffff,
};

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}


static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.\n");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.\n",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.\n",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}


static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========\n");
    NRF_LOG_INFO("erase unit: \t%d bytes\n",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes\n",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================\n");
}


void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        nrf_pwr_mgmt_run();
    }
}

static uint32_t m_data          = 0xBADC0FFE;
static char     m_hello_world[] = "hello world";

void fstorage_read(uint32_t addr, uint32_t len, uint8_t* pData, data_fmt_t fmt)
{
    ret_code_t rc;
    uint8_t    data[256] = {0};

    if (len > sizeof(data))
    {
        len = sizeof(data);
    }

    /* Read data. */
    rc = nrf_fstorage_read(&fstorage, addr, data, len);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n",
                        nrf_strerror_get(rc));
        return;
    }

    switch (fmt)
    {
        case DATA_FMT_HEX:
        {
            /* Print bytes. */
            NRF_LOG_HEXDUMP_INFO(data, len);
        } break;

        case DATA_FMT_STR:
        {
            NRF_LOG_INFO("%s\n", data);
        } break;

        default:
            break;
    } 
    if(pData != NULL)
        memcpy(pData, data, len);
}

void fstorage_erase(uint32_t addr, uint32_t pages_cnt)
{
    ret_code_t rc = nrf_fstorage_erase(&fstorage, addr, pages_cnt, NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_erase() returned: %s\n", nrf_strerror_get(rc));
    }
    wait_for_flash_ready(&fstorage);
}


void test_fstorage()
{
    ret_code_t rc;
    NRF_LOG_INFO("test fstorage\n");
    NRF_LOG_INFO("flash end address : 0x%x\n", nrf5_flash_end_addr_get());
    
    fstorage_erase(FSDB_BODY_ADDR, 1);
//
    NRF_LOG_INFO("Writing \"%x\" to flash.\n", m_data);
    rc = nrf_fstorage_write(&fstorage, FSDB_BODY_ADDR, &m_hello_world, sizeof(m_hello_world), NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(&fstorage);
    NRF_LOG_INFO("Done.\n");

    NRF_LOG_INFO("reading from fstorage...");
    fstorage_read(FSDB_BODY_ADDR, 100, NULL, DATA_FMT_HEX);
    wait_for_flash_ready(&fstorage);
}

void reset_fsdb_contents()
{
    ret_code_t rc;
    NRF_LOG_INFO("erase fstorage...");
    g_page_addr = 0;
    fstorage_erase(FSDB_BASE_ADDR, 2);
    
    rc = nrf_fstorage_write(&fstorage, FSDB_BASE_ADDR, &g_page_addr, sizeof(g_page_addr), NULL);
    wait_for_flash_ready(&fstorage);
    NRF_LOG_INFO("Done.\n");

    APP_ERROR_CHECK(rc);

//    fstorage_read(FSDB_BASE_ADDR, 4, NULL, DATA_FMT_HEX);
//    wait_for_flash_ready(&fstorage);
//
//    fstorage_read(FSDB_BODY_ADDR, 40, NULL, DATA_FMT_HEX);
//    wait_for_flash_ready(&fstorage);

    NRF_LOG_INFO("erase done");
}

void print_fsdb_header()
{
    NRF_LOG_INFO("[FSDB] harder printing..");
    fstorage_read(FSDB_BASE_ADDR, 4, NULL, DATA_FMT_HEX);
    wait_for_flash_ready(&fstorage);
    NRF_LOG_INFO("\n");
}

void print_fsdb_body(uint32_t addr, uint32_t len)
{
    NRF_LOG_INFO("[FSDB] data printing..");
    fstorage_read(FSDB_BODY_ADDR+addr, len, NULL, DATA_FMT_HEX);
    wait_for_flash_ready(&fstorage);
    NRF_LOG_INFO("\n");
}

void init_fstorage()
{
    ret_code_t rc;
    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_sd;
    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);
    print_flash_info(&fstorage);
}

void read_fsdb_harder()
{
    NRF_LOG_INFO("[FSDB] reading header...");
    fstorage_read(FSDB_BASE_ADDR, sizeof(g_page_addr), (uint8_t*)&g_page_addr, DATA_FMT_NONE);
    wait_for_flash_ready(&fstorage);
    NRF_LOG_INFO("      stored data size : %d\n",  g_page_addr);
    g_page_addr = g_page_addr - (g_page_addr % FSDB_PAGESIZE);
    fstorage_erase(FSDB_BODY_ADDR + g_page_addr, 1);
}

void fstorage_write(uint32_t addr, uint8_t* data, uint32_t len)
{
  ret_code_t rc;
  rc = nrf_fstorage_write(&fstorage, addr, data, len, NULL);
  APP_ERROR_CHECK(rc);
  wait_for_flash_ready(&fstorage);
}

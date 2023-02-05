#include "flash_logging.h"

#include "string.h"

#include "global_variables.h"
#include "w25qxx.h"
#include "utils.h"
#include "j_packets.h"
#include "j_packet_send_callback.h"

#ifdef MCU

void configure_logging() {
    if (flash_page_addr == 0) {
        can_do_logging = W25qxx_IsEmptyPage(0, 0, 256);
    } else {
        can_do_logging = true;
    }
}


void write_flash_bytes(uint8_t *buf, uint32_t num_bytes) {
    const uint32_t page_size = 256;
    uint32_t remaining_bytes = num_bytes;

    while (remaining_bytes > 0) {
        uint32_t bytes_to_write = MIN(remaining_bytes, page_size - flash_page_offset);
        
        W25qxx_WritePage(buf, flash_page_addr, flash_page_offset, bytes_to_write);

        flash_page_offset += bytes_to_write;
        if (flash_page_offset >= page_size) {
            flash_page_offset = 0;
            flash_page_addr += 1;
        }

        buf += bytes_to_write;
        remaining_bytes -= bytes_to_write;
    }
}

void flash_erase() {
    W25qxx_EraseChip();

    flash_page_offset = 0;
    flash_page_addr = 0;
}

void do_flash_log(char *buf) {
    if (can_do_logging && is_armed) {
        write_flash_bytes((uint8_t *)buf, strlen(buf));
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    }
}

void dump_flash_log() {
    int page = 0;
    while (!W25qxx_IsEmptyPage(page, 0, 256)) {
        uint8_t res_buf[256];
        W25qxx_ReadPage((uint8_t *)res_buf, page, 0, 256);
        j_packet_send(0x00, (void *)res_buf, 256, 1, J_PACKET_SIZE, j_packet_send_callback);

        HAL_Delay(10);

        page += 1;
    }
}

void del_flash_log() {
    send_jpacket_info(0x00, "Erasing...\n", 64);
    flash_erase();
    send_jpacket_info(0x00, "Done erasing\n", 64);
}


#else // MCU

void configure_logging() { }
void write_flash_bytes(uint8_t *buf, uint32_t num_bytes) { }
void flash_erase() { }
void do_flash_log(char *buf) { }
void dump_flash_log() { }
void del_flash_log() { }

#endif // MCU
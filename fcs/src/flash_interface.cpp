#include "flash_interface.h"

#include "string.h"

#include "global_variables.h"
#include "w25qxx.h"
#include "utils.h"
#include "j_packets.h"
#include "j_packet_send_callback.h"


#ifdef MCU

void configure_logging() {
    if (flash_page_addr < initial_page) {
        flash_page_addr = initial_page;
    }
    
    if (flash_page_addr == initial_page) {
        can_do_logging = W25qxx_IsEmptyPage(initial_page, 0, 256);
    } else {
        can_do_logging = true;
    }
}


void write_flash_bytes(uint8_t *buf, uint32_t num_bytes) {
    const uint32_t page_size = 256;
    uint32_t remaining_bytes = num_bytes;

    while (remaining_bytes > 0) {
        if (flash_page_addr >= 65535) return;
        
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
    flash_page_addr = initial_page;
}

void do_flash_log(char *buf) {
    if (can_do_logging && is_armed) {
        write_flash_bytes((uint8_t *)buf, strlen(buf));
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    }
}

void dump_flash_log() {
    int page = initial_page;
    while (!W25qxx_IsEmptyPage(page, 0, 256)) {
        if (flash_page_addr >= 65535) return;
        
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
    save_pid_flash(true);

    send_jpacket_info(0x00, "Done erasing\n", 64);
}

void save_pid_flash(bool is_sector_erased) {
    float buf[64] = { 0 };

    buf[0] = alt_sp_pid.k_p;
    buf[1] = alt_sp_pid.k_i;
    buf[2] = alt_sp_pid.k_d;

    buf[3] = alt_rate_pid.k_p;
    buf[4] = alt_rate_pid.k_i;
    buf[5] = alt_rate_pid.k_d;

    buf[6] = yaw_sp_pid.k_p;
    buf[7] = yaw_sp_pid.k_i;
    buf[8] = yaw_sp_pid.k_d;

    buf[9]  = yaw_rate_pid.k_p;
    buf[10] = yaw_rate_pid.k_i;
    buf[11] = yaw_rate_pid.k_d;

    buf[12] = x_body_pid.k_p;
    buf[13] = x_body_pid.k_i;
    buf[14] = x_body_pid.k_d;

    buf[15] = y_body_pid.k_p;
    buf[16] = y_body_pid.k_i;
    buf[17] = y_body_pid.k_d;

    buf[18] = roll_pid.k_p;
    buf[19] = roll_pid.k_i;
    buf[20] = roll_pid.k_d;

    buf[21] = pitch_pid.k_p;
    buf[22] = pitch_pid.k_i;
    buf[23] = pitch_pid.k_d;

    buf[24] = roll_rate_pid.k_p;
    buf[25] = roll_rate_pid.k_i;
    buf[26] = roll_rate_pid.k_d;

    buf[27] = pitch_rate_pid.k_p;
    buf[28] = pitch_rate_pid.k_i;
    buf[29] = pitch_rate_pid.k_d;

    if (!is_sector_erased) W25qxx_EraseSector(0);

    W25qxx_WritePage((uint8_t *)buf, 0, 0, sizeof(buf));

    send_jpacket_info(0x00, "Saved\n", 64);
}

void load_pid_flash(bool send_info) {
    if (W25qxx_IsEmptyPage(0, 0, 256)) {
        if (send_info) send_jpacket_info(0x00, "No config\n", 64);
        return;
    }
    
    float buf[64] = { 0 };
    W25qxx_ReadPage((uint8_t *)buf, 0, 0, sizeof(buf));

    alt_sp_pid.k_p = buf[0];
    alt_sp_pid.k_i = buf[1];
    alt_sp_pid.k_d = buf[2];

    alt_rate_pid.k_p = buf[3];
    alt_rate_pid.k_i = buf[4];
    alt_rate_pid.k_d = buf[5];

    yaw_sp_pid.k_p = buf[6];
    yaw_sp_pid.k_i = buf[7];
    yaw_sp_pid.k_d = buf[8];

    yaw_rate_pid.k_p = buf[9];
    yaw_rate_pid.k_i = buf[10];
    yaw_rate_pid.k_d = buf[11];

    x_body_pid.k_p = buf[12];
    x_body_pid.k_i = buf[13];
    x_body_pid.k_d = buf[14];

    y_body_pid.k_p = buf[15];
    y_body_pid.k_i = buf[16];
    y_body_pid.k_d = buf[17];

    roll_pid.k_p = buf[18];
    roll_pid.k_i = buf[19];
    roll_pid.k_d = buf[20];

    pitch_pid.k_p = buf[21];
    pitch_pid.k_i = buf[22];
    pitch_pid.k_d = buf[23];

    roll_rate_pid.k_p = buf[24];
    roll_rate_pid.k_i = buf[25];
    roll_rate_pid.k_d = buf[26];

    pitch_rate_pid.k_p = buf[27];
    pitch_rate_pid.k_i = buf[28];
    pitch_rate_pid.k_d = buf[29];

    if (send_info) send_jpacket_info(0x00, "Loaded\n", 64);
}



#else // MCU

void configure_logging() { }
void write_flash_bytes(uint8_t *buf, uint32_t num_bytes) { }
void flash_erase() { }
void do_flash_log(char *buf) { }
void dump_flash_log() { }
void del_flash_log() { }
void save_pid_flash(bool is_sector_erased) { }
void load_pid_flash(bool send_info) { }

#endif // MCU

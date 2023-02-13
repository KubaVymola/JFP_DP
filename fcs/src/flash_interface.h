#ifndef FLASHLOGGING_H
#define FLASHLOGGING_H

#include <stdint.h>

const int initial_page = 16;

void configure_logging();
void write_flash_bytes(uint8_t *buf, uint32_t num_bytes);
void flash_erase();
void do_flash_log(char *buf);
void dump_flash_log();
void del_flash_log();

void save_pid_flash(bool is_sector_erased);
void load_pid_flash(bool send_info);

#endif

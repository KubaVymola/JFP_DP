#ifndef FLASHLOGGING_H
#define FLASHLOGGING_H

#include <stdint.h>

void configure_logging();
void write_flash_bytes(uint8_t *buf, uint32_t num_bytes);
void flash_erase();
void do_flash_log(char *buf);
void dump_flash_log();
void del_flash_log();

#endif

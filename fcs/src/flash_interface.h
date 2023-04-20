//==============================================================================
// flash_interface.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef FLASHLOGGING_H
#define FLASHLOGGING_H

#include <stdint.h>

void configure_logging();
void write_flash_bytes(uint8_t *buf, uint32_t num_bytes);
void flash_erase();
void do_flash_log(char *buf);
void dump_flash_log();
void del_flash_log();

void save_pid_flash(bool is_sector_erased);
void load_pid_flash(bool send_info);

#endif

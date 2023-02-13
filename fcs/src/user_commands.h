#ifndef USERCOMMANDS_H
#define USERCOMMANDS_H

#include "pid.h"

void parse_user_command();
void parse_pid_config(pid_state_t& pid);
void print_pid_config();

#endif

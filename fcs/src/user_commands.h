#ifndef USERCOMMANDS_H
#define USERCOMMANDS_H

#include "pid.h"

/**
 * When a full command in received on channel 0x00 into cmd_string (the cmd_string ends with newline)
 * this function parses the cmd_string and performs a given action
*/
void parse_user_command();

/**
 * Gets called by parse_user_command if the first word of user command is one of the following:
 * alt_sp, alt_rate, yaw_sp, yaw_rate, x_body, y_body, roll, pitch, roll_rate, pitch_rate
*/
void parse_pid_config(pid_state_t& pid);

/**
 * Gets called when cmd_string is 'print_pid'
*/
void print_pid_config();

#endif

#include "user_commands.h"

#include "string.h"
#include "global_variables.h"
#include "flash_interface.h"
#include "utils.h"

void parse_user_command() {
    if (cmd_string[strlen(cmd_string) - 1] != '\n') return;

    cmd_string[strlen(cmd_string) - 1] = '\0';  // trim string
    char *token = strtok(cmd_string, " ");

    if (strcmp("help", token) == 0) {
        send_jpacket_info(0x00, "--\n", 64);
        send_jpacket_info(0x00, "sayhi\narm\ndisarm\ndumplog\ndellog\n", 64);
        send_jpacket_info(0x00, "save_pid\nload_pid\nprint_pid\n", 64);
        send_jpacket_info(0x00, "alt_sp,alt_rate,yaw_sp,yaw_rate,x_body,y_body,roll,pitch,roll_rate,pitch_rate\n", 128);
        send_jpacket_info(0x00, "--\n", 64);
    }
    
    if (strcmp("sayhi", token) == 0) {
        send_jpacket_info(0x00, "I wont say hi\n", 64);
    }
    
    if (strcmp("arm", token) == 0) {
        is_armed = true;
        force_arm = true;
        configure_logging();
    }
    
    if (strcmp("disarm", token) == 0) {
        is_armed = false;
        force_arm = false;

    }
    
    if (strcmp("dumplog", token) == 0) {
        dump_flash_log();
    }

    if (strcmp("dellog", token) == 0) {
        del_flash_log();
    }

    /**
     * ==== Parse PID config ====
     */
    if (strcmp("alt_sp", token) == 0) {
        parse_pid_config(alt_sp_pid);
    }
    if (strcmp("alt_rate", token) == 0) {
        parse_pid_config(alt_rate_pid);
    }
    if (strcmp("yaw_sp", token) == 0) {
        parse_pid_config(yaw_sp_pid);
    }
    if (strcmp("yaw_rate", token) == 0) {
        parse_pid_config(yaw_rate_pid);
    }
    if (strcmp("x_body", token) == 0) {
        parse_pid_config(x_body_pid);
    }
    if (strcmp("y_body", token) == 0) {
        parse_pid_config(y_body_pid);
    }
    if (strcmp("roll", token) == 0) {
        parse_pid_config(roll_pid);
    }
    if (strcmp("pitch", token) == 0) {
        parse_pid_config(pitch_pid);
    }
    if (strcmp("roll_rate", token) == 0) {
        parse_pid_config(roll_rate_pid);
    }
    if (strcmp("pitch_rate", token) == 0) {
        parse_pid_config(pitch_rate_pid);
    }
    /**
     * ==== END Parse PID config ====
     */

    if (strcmp("save_pid", token) == 0) {
        save_pid_flash(false);
    }

    if (strcmp("load_pid", token) == 0) {
        load_pid_flash(true);
    }

    if (strcmp("print_pid", token) == 0) {
        print_pid_config();
    }

    memset(cmd_string, '\0', sizeof(cmd_string));
}

void parse_pid_config(pid_state_t& pid) {
    char *pid_coef_name = strtok(NULL, " ");
    char *pid_coef_val  = strtok(NULL, " ");

    if (pid_coef_name == nullptr) return;
    if (pid_coef_val  == nullptr) return;

    float value = atof(pid_coef_val);

    if (strcmp(pid_coef_name, "p") == 0) {
        pid.k_p = value;
    }
    if (strcmp(pid_coef_name, "i") == 0) {
        pid.k_i = value;
    }
    if (strcmp(pid_coef_name, "d") == 0) {
        pid.k_d = value;
    }
}

void print_pid_config() {
    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "alt_sp",   alt_sp_pid.k_p,   alt_sp_pid.k_i,   alt_sp_pid.k_d);
    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "alt_rate", alt_rate_pid.k_p, alt_rate_pid.k_i, alt_rate_pid.k_d);
    
    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "yaw_sp",   yaw_sp_pid.k_p,   yaw_sp_pid.k_i,   yaw_sp_pid.k_d);
    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "yaw_rate", yaw_rate_pid.k_p, yaw_rate_pid.k_i, yaw_rate_pid.k_d);

    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "x_body", x_body_pid.k_p, x_body_pid.k_i, x_body_pid.k_d);
    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "y_body", y_body_pid.k_p, y_body_pid.k_i, y_body_pid.k_d);

    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "roll",  roll_pid.k_p,  roll_pid.k_i,  roll_pid.k_d);
    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "pitch", pitch_pid.k_p, pitch_pid.k_i, pitch_pid.k_d);

    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "roll_rate",  roll_rate_pid.k_p,  roll_rate_pid.k_i,  roll_rate_pid.k_d);
    send_jpacket_info(0x00, "%-11s %.6f  %.6f  %.6f\n", 64, "pitch_rate", pitch_rate_pid.k_p, pitch_rate_pid.k_i, pitch_rate_pid.k_d);
}

#include "user_commands.h"

#include "string.h"
#include "global_variables.h"
#include "flash_logging.h"
#include "utils.h"

void parse_user_command() {
    if (cmd_string[strlen(cmd_string) - 1] != '\n') return;

    cmd_string[strlen(cmd_string) - 1] = '\0';  // trim string
    char *token = strtok(cmd_string, " ");

    if (strcmp("help", token) == 0) {
        send_jpacket_info(0x00, "--\nsayhi\nalt_sp <param>\narm\ndisarm\ndumplog\ndellog\n--\n", 64);
    }
    if (strcmp("sayhi", token) == 0) {
        send_jpacket_info(0x00, "I wont say hi\n", 64);
    }
    if (strcmp("alt_sp", token) == 0) {
        char *val = strtok(NULL, " ");
        alt_sp = atof(val);
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

    memset(cmd_string, '\0', sizeof(cmd_string));
}
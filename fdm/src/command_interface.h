#ifndef COMMANDINTERFACE_H
#define COMMANDINTERFACE_H

#include <stdint.h>
#include <poll.h>
#include <string>
#include <vector>

class CommandInterface {
public:
    CommandInterface() : socket_fd(-1) { }
    void init(uint16_t port);
    std::string update_socket_and_read_commands();
    void send_command_output(char *data, int size);
private:
    std::vector<pollfd> read_poll_fds;

    int socket_fd;
};

#endif

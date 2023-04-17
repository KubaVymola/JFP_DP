#ifndef COMMANDINTERFACE_H
#define COMMANDINTERFACE_H

#include <stdint.h>
#include <poll.h>
#include <string>
#include <vector>

/**
 * This class implements a TCP socket server, and it realizes the reading data from a writing data
 * to the server.
 * 
 * This class is used both by the SerialInterface, as well as SITLInterface
*/
class CommandInterface {
public:
    CommandInterface() : socket_fd(-1) { }

    /**
     * The function opens a socket on a given port and starts listening
    */
    void init(uint16_t port);

    /**
     * The function accepts new connections, polls existing connections for any data, and returns
     * the first pience of data as a string to a calling function, which is a method of
     * SerialInterface or SITLInterface
    */
    std::string update_socket_and_read_commands();

    /**
     * The function sends data from SerialInterface or SITLInterface over every open connection
    */
    void send_command_output(char *data, int size);
private:
    std::vector<pollfd> read_poll_fds;

    int socket_fd;
};

#endif

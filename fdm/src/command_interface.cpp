//==============================================================================
// command_interface.cpp
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "command_interface.h"

#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

void CommandInterface::init(uint16_t port) {
    read_poll_fds.clear();
    
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    in_addr addr_any = { INADDR_ANY };
    sockaddr_in srv_addr;
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_port = htons(port);
    srv_addr.sin_addr = addr_any;

    int opt = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        return;
    }

    if (bind(socket_fd, (const sockaddr *)&srv_addr, sizeof(srv_addr)) < 0) {
        return;
    }

    if (listen(socket_fd, 1) < 0) {
        return;
    }

    printf("CMD: Socket on port %d listening\n", port);

    read_poll_fds.push_back({ socket_fd, POLLIN });
}

std::string CommandInterface::update_socket_and_read_commands() {
    if (socket_fd < 0) return "";
    
    int poll_res = poll(read_poll_fds.data(), read_poll_fds.size(), 0);


    if (read_poll_fds[0].revents & POLLIN) {
        printf("CMD: New connection\n");

        sockaddr_in rsa;
        int rsa_size = sizeof(rsa);

        int client_fd = accept(socket_fd, (sockaddr *)&rsa, (socklen_t *)&rsa_size);
        if (client_fd < 0) return "";

        read_poll_fds.push_back({ client_fd, POLLIN });
    }

    for (std::vector<pollfd>::iterator it = read_poll_fds.begin() + 1;
         it != read_poll_fds.end();) {

        if (it->revents & POLLIN) {
            char buf[128];
            int len = read(it->fd, buf, sizeof(buf) - 1);

            if (len <= 0) {
                printf("CMD: Client closed\n");
                close(it->fd);
                read_poll_fds.erase(it);
                continue;
            }


            // Pass the read command to serial interface and to FCS
            buf[len] = '\0';            
            return std::string(buf);
        }


        ++it;
    }

    return "";
}

void CommandInterface::send_command_output(char *data, int size) {
    if (socket_fd < 0) return;
    
    for (std::vector<pollfd>::iterator it = read_poll_fds.begin() + 1;
         it != read_poll_fds.end();
         ++it) {

        write(it->fd, data, size);
    }
}

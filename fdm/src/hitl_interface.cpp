#include "hitl_interface.h"

#include "termios.h"
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions

void HITLInterface::hitl_init(sim_config_t& sim_config,
                              json *sim_data,
                              bool *continue_running) {
    
    // int serial_port = open(sim_config.hitl_path.c_str(), O_RDWR | O_NOCTTY);
    
    printf("Opening serial port: %s\n", sim_config.hitl_path.c_str());

    /**
     * Open in non-blocking mode - should work with /dev/tty.* on macos
     */
    int serial_port = open(sim_config.hitl_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    /**
     * Remove non-blocking mode from fd after opening
     */
    int flag = fcntl(serial_port, F_GETFL);
    fcntl(serial_port, F_SETFL, flag & ~O_NONBLOCK);

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return;
    }


    while (*continue_running) {
        char read_buf[64];
        
        const char *message = "Hello worlda\n";
        write(serial_port, message, strlen(message));
        
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
        if (num_bytes < 0) {
            printf("Error reading: %s\n", strerror(errno));
            return;
        }

        // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
        // print it to the screen like this!)
        printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf);
    }
}
                
void HITLInterface::parse_xml_config(sim_config_t& sim_config) {
    
}

void HITLInterface::handle_event(const std::string& event_name, json *sim_data) {
    if (event_name == "sim:before_iter") {

    }
}
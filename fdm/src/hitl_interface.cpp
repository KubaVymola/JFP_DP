#include "hitl_interface.h"

#include <errno.h>    // Error integer and strerror() function
#include <termios.h>  // Contains POSIX terminal control definitions

#ifdef __linux__
#include <sys/ioctl.h>
#endif

#include "timer.h"

#define LOBYTE(x)  ((uint8_t)(x & 0x00FFU))
#define HIBYTE(x)  ((uint8_t)((x & 0xFF00U) >> 8U))

#define MIN(x,y)   (((x)<(y))?(x):(y))

#define PACKET_HEADER_SIZE   6 /* 1B (channel) 1B (0x55) 2B (size) 2B (offset) */
#define PACKET_FOOTER_SIZE   1 /* 1B (0x00) */

int HITLInterface::round_down_to_multiple(int number, int multiple) {
    return number - number % multiple;
}

void HITLInterface::send_data_usb(int serial_port, int channel_number, void *data, uint16_t data_size, int item_size, int buffer_size) {
    uint16_t current_offset = 0;
    uint16_t maximum_data_in_packet = round_down_to_multiple(buffer_size - PACKET_HEADER_SIZE - PACKET_FOOTER_SIZE, item_size);
    
    uint8_t to_send[buffer_size];
    to_send[0] = channel_number;
    to_send[1] = 0x55; /* Simple packet validation */
    
    /**
     * Send out individual packets
     */
    while (current_offset < data_size) {
        uint16_t remaining_size = data_size - current_offset;
        uint16_t to_send_size = MIN(remaining_size, maximum_data_in_packet);

        to_send[2] = LOBYTE(to_send_size);
        to_send[3] = HIBYTE(to_send_size);
        to_send[4] = LOBYTE(current_offset);
        to_send[5] = HIBYTE(current_offset);
        
        memcpy(to_send + PACKET_HEADER_SIZE, (uint8_t *)data + current_offset, to_send_size);

        to_send[PACKET_HEADER_SIZE + to_send_size] = '\0';

        int num_bytes = write(serial_port, to_send, PACKET_HEADER_SIZE + to_send_size + PACKET_FOOTER_SIZE);
        tcdrain(serial_port);
        
        // {
        //     Timer timer;
        //     tcdrain(serial_port);
        // }

        current_offset += to_send_size;
    }
}

void HITLInterface::receive_data_usb(int serial_port, json *sim_data) {
    char buf[1024];
    int len = read(serial_port, buf, sizeof(buf));

    /**
     * n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
     */
    if (len < 0) {
        // printf("Error reading: %s\n", strerror(errno));
        return;
    }

    char *current_data = buf;

    while (current_data - buf + PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE <= len) {
        uint8_t channel_number  =  current_data[0];
        uint16_t data_size   = (current_data[3] << 8) | current_data[2];
        uint16_t data_offset = (current_data[5] << 8) | current_data[4];

        /**
         * Incomplete packet received
         */
        if (current_data - buf + PACKET_HEADER_SIZE + data_size + PACKET_FOOTER_SIZE > len) break;

        /**
         * Invalid packet received
         */
        if (channel_number > 0x02
        || current_data[1] != 0x55
        || current_data[PACKET_HEADER_SIZE + data_size] != '\0'
        || data_size > 64
        || data_offset > 512) {
            current_data++;
            continue;
        }

        // TODO handle correct channels
        // TODO add callback that sends data, data size and channel number

        if (channel_number == 0x00) {
            write(STDOUT_FILENO, current_data + PACKET_HEADER_SIZE, data_size);
            printf("\n");
        }

        /**
         * Receive data to sim_data
         */
        if (channel_number == 0x02) {
            float *new_data = (float *)(current_data + PACKET_HEADER_SIZE);
            int data_items_offset = data_offset / sizeof(float);
            
            for (int i = 0; i < data_size / sizeof(float); ++i) {
                (*sim_data)[to_jsbsim_properties[i + data_items_offset]] = new_data[i];
            }
        }


        current_data += PACKET_HEADER_SIZE + data_size + PACKET_FOOTER_SIZE;
    }
}


void HITLInterface::hitl_init(sim_config_t &sim_config,
                              json *sim_data) {
    
    printf("Opening serial port: %s\n", sim_config.hitl_path.c_str());

    /**
     * Open in non-blocking mode - should work with /dev/tty.* on macos
     */
    hitl_device = open(sim_config.hitl_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    /**
     * // Remove non-blocking mode from fd after opening
     */
    // int flag = fcntl(hitl_device, F_GETFL);
    // fcntl(hitl_device, F_SETFL, flag & ~O_NONBLOCK);

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(hitl_device, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return;
    }

    tty.c_cflag &= ~PARENB;         // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;         // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;          // Clear all bits that set the data size
    tty.c_cflag |= CS8;             // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                         // Disable echo
    tty.c_lflag &= ~ECHOE;                                                        // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                       // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                         // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                       // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);  // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    // tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    // tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // cfmakeraw(&tty);

    // Save tty settings, also checking for error
    if (tcsetattr(hitl_device, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return;
    }
    
#ifdef __linux__
    /**
     * ? On linux reduce USB latency from 16 ms to about 2 ms
     * https://www.projectgus.com/2011/10/notes-on-ftdi-latency-with-arduino/
     */
    struct serial_struct serinfo;
    ioctl(hitl_device, TIOCGSERIAL, &serinfo);
    serinfo.flags |= ASYNC_LOW_LATENCY;
    ioctl(hitl_device, TIOCSSERIAL, &serinfo);
#endif

    parse_xml_config(sim_config);    
}

void HITLInterface::parse_xml_config(sim_config_t &sim_config) {
    using namespace tinyxml2;

    XMLDocument craft_doc(true, COLLAPSE_WHITESPACE);
    craft_doc.LoadFile(sim_config.craft_config_path.c_str());

    XMLElement *root_elem = craft_doc.FirstChildElement("fdm_config");
    XMLElement *fcs_elem = root_elem->FirstChildElement("fcs_interface");

    /**
     * Get all properties that go from FCS to sim_data
     */
    XMLElement *to_jsb_elem = fcs_elem->FirstChildElement("to_jsbsim");
    XMLElement *to_jsb_prop_elem = to_jsb_elem->FirstChildElement("property");
    for (; to_jsb_prop_elem != nullptr; to_jsb_prop_elem = to_jsb_prop_elem->NextSiblingElement("property")) {
        to_jsbsim_properties.push_back(to_jsb_prop_elem->GetText());
    }
    
    /**
     * Get all properties that go from sim_data to FCS
     */
    XMLElement *from_jsb_elem = fcs_elem->FirstChildElement("from_jsbsim");

    XMLElement *from_jsb_prop_elem = from_jsb_elem->FirstChildElement("property");
    for (; from_jsb_prop_elem != nullptr; from_jsb_prop_elem = from_jsb_prop_elem->NextSiblingElement("property")) {
        from_jsbsim_properties.push_back(from_jsb_prop_elem->GetText());
    }
}

void HITLInterface::handle_event(const std::string &event_name, json *sim_data) {
    if (event_name == "sim:before_iter") {
        // Non-blocking call, might receive nothing - that's OK
        receive_data_usb(hitl_device, sim_data);
    }

    if (event_name == "sim:after_iter") {
        float data[from_jsbsim_properties.size()];

        int i = 0;
        for (const std::string& to_fcs_property : from_jsbsim_properties) {
            data[i++] = (float)sim_data->value<double>(to_fcs_property, 0.0);
            // printf("%s: %f\n", to_fcs_property.c_str(), sim_data->value<double>(to_fcs_property, 0.0));
        }
        
        // printf("items: %d, size: %d\n", from_jsbsim_properties.size(), sizeof(data));
        
        send_data_usb(hitl_device, 0x02, data, sizeof(data), sizeof(float), 64);
    }
}
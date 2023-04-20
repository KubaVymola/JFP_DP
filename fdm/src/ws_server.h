//==============================================================================
// ws_server.h
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef WSSERVER_H
#define WSSERVER_H

#include <string>
#include <functional>
#include <set>
#include <mutex>
#include <condition_variable>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;

/**
 * Gives a meaning to an action.
 * 
 * "SUBSCRIBE" action_type marks a new client.
 * "UNSUBSCRIBE" action_type marks a disconnection of a client.
 * If action_type is "MESSAGE", the on_message_cb function gets called.
 * If action_type is "PUSH_MESSAGE", the given message is sent to all clients.
*/
enum action_type {
    SUBSCRIBE,
    UNSUBSCRIBE,
    MESSAGE,
    PUSH_MESSAGE,
};

struct action {
    action(action_type t, websocketpp::connection_hdl h): type(t), hdl(h) {}
    action(action_type t, websocketpp::connection_hdl h, std::string p)
        : type(t), hdl(h), payload(p) {}
    action(action_type t, std::string p): type(t), payload(p) {}

    action_type type;
    websocketpp::connection_hdl hdl;
    std::string payload;
};

/**
 * websocket_server class encapsulates a server used to send JSON data over websocket to a 3D
 * visualization application.
 * 
 * This server runs in a dedicated thread and it spawns two additional threads, which are used to
 * handle the server and to handle processing of the action queue (m_actions).
*/
class websocket_server {
public:
    websocket_server();
    void run(uint16_t port);
    void stop();
    void on_open(websocketpp::connection_hdl hdl);
    void on_close(websocketpp::connection_hdl hdl);
    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg);
    void send_message(std::string payload);
    void process_messages();
    void register_on_message_cb(void (*cb)(std::string));
private:
    typedef std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> con_list;

    server m_server;
    con_list m_connections;
    std::queue<action> m_actions;

    std::mutex m_action_lock;
    std::mutex m_connection_lock;
    std::condition_variable m_action_cond;

    void (*on_message_cb)(std::string) = nullptr;

    bool m_done;
};

#endif

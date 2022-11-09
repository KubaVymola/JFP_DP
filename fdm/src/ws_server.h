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

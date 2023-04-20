//==============================================================================
// ws_server.cpp
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "ws_server.h"

using std::placeholders::_1;
using std::placeholders::_2;

websocket_server::websocket_server()
    : m_done(false) {
    m_server.init_asio();
    
    m_server.clear_access_channels(websocketpp::log::alevel::all);

    m_server.set_open_handler(bind(&websocket_server::on_open, this, ::_1));
    m_server.set_close_handler(bind(&websocket_server::on_close, this, ::_1));
    m_server.set_message_handler(bind(&websocket_server::on_message, this, ::_1, ::_2));
}

void websocket_server::run(uint16_t port) {
    m_server.set_reuse_addr(true);
    m_server.listen(port);
    m_server.start_accept();

    try {
        std::thread asio_thread(&server::run, &m_server);
        std::thread processing_thread(&websocket_server::process_messages, this);

        asio_thread.join();
        printf("Asio stopped\n");
        
        processing_thread.join();
        printf("Processing thread stopped\n");
    } catch (const std::exception& e) {
        printf("Exception: %s\n", e.what());
    }
}

void websocket_server::stop() {
    printf("Stopping asio\n");
    
    m_server.stop_listening();

    {
        std::lock_guard<std::mutex> guard(m_connection_lock);

        con_list::iterator it;
        for (it = m_connections.begin(); it != m_connections.end(); ++it) {
            m_server.close(*it, 1000, "Server quitting");
        }

        m_connections.clear();
    }

    m_done = true;
    m_action_cond.notify_one();
    printf("Stopping processing thread\n");
}

void websocket_server::on_open(websocketpp::connection_hdl hdl) {
    {
        std::lock_guard<std::mutex> guard(m_action_lock);
        m_actions.push(action(SUBSCRIBE, hdl));
    }

    m_action_cond.notify_one();
}

void websocket_server::on_close(websocketpp::connection_hdl hdl) {
    {
        std::lock_guard<std::mutex> guard(m_action_lock);
        m_actions.push(action(UNSUBSCRIBE, hdl));
    }

    m_action_cond.notify_one();
}

void websocket_server::on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
    {
        std::lock_guard<std::mutex> guard(m_action_lock);
        m_actions.push(action(MESSAGE, hdl, msg->get_payload()));
    }

    m_action_cond.notify_one();
}

void websocket_server::send_message(std::string payload) {
    {
        std::lock_guard<std::mutex> guard(m_action_lock);
        m_actions.push(action(PUSH_MESSAGE, payload));
    }

    m_action_cond.notify_one();
}


void websocket_server::process_messages() {
    while (1) {
        std::unique_lock<std::mutex> lock(m_action_lock);

        while (m_actions.empty()) {
            m_action_cond.wait(lock);

            if (m_done) return;
        }

        action a = m_actions.front();
        m_actions.pop();

        if (a.type == SUBSCRIBE) {
            std::lock_guard<std::mutex> guard(m_connection_lock);
            m_connections.insert(a.hdl);
        } else if (a.type == UNSUBSCRIBE) {
            std::lock_guard<std::mutex> guard(m_connection_lock);
            m_connections.erase(a.hdl);
        }  else if (a.type == MESSAGE) {
            std::lock_guard<std::mutex> guard(m_connection_lock);

            if (on_message_cb != nullptr) on_message_cb(a.payload);
        } else if (a.type == PUSH_MESSAGE) {
            std::lock_guard<std::mutex> guard(m_connection_lock);

            con_list::iterator it;
            for (it = m_connections.begin(); it != m_connections.end(); ++it) {
                m_server.send(*it, a.payload, websocketpp::frame::opcode::TEXT);
            }
        } else {
            // Undefined
        }
    }
}

void websocket_server::register_on_message_cb(void (*cb)(std::string)) {
    this->on_message_cb = cb;
}

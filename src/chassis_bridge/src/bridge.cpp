#include <cstdint>
#include <chrono>
#include <thread>
#include <exception>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>

#include "chassis_bridge/types.hpp"
#include "chassis_bridge/container.hpp"
#include "chassis_bridge/connection.hpp"
#include "chassis_bridge/nodes.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    constexpr uint16_t connection_listen_port{60000};

    using rx_deque_item = std::shared_ptr<cb::types::underlying::rx::frame>;
    using tx_deque_item = std::shared_ptr<cb::types::underlying::tx::frame>;

    cb::container::ts::deque<rx_deque_item> receive_deque;
    cb::container::ts::deque<tx_deque_item> transmit_deque;

    std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
              << " [bridge main thread] spawning connection server thread from thread: " << std::this_thread::get_id() << std::endl;
    auto connection_server_thread{std::thread([&] {
        connection_server_start: try {
            asio::io_context io_context;
            cb::connection::tcp::server<cb::connection::tcp::session> server(
                io_context, connection_listen_port, &receive_deque, &transmit_deque
            );
            io_context.run();
        } catch (std::exception& e) {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << " [connection server thread] restarting connection server with exception: " << e.what() << std::endl;
            goto connection_server_start;
        }
    })};

    std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
              << " [bridge main thread] launching bridge node from thread: " << std::this_thread::get_id() << std::endl;
    auto bridge_node_handler_ptr{std::make_shared<cb::nodes::bridge>("chassis_bridge", &receive_deque, &transmit_deque)};

    connection_server_thread.join();
    bridge_node_handler_ptr->threads_join();

    return 0;
}

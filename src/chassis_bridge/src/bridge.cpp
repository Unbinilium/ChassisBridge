#include <cstdint>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "chassis_bridge/types.hpp"
#include "chassis_bridge/container.hpp"
#include "chassis_bridge/connection.hpp"
#include "chassis_bridge/nodes.hpp"
#include "chassis_bridge/utility.hpp"

int main(int argc, char* argv[]) {
    rclcpp::install_signal_handlers(rclcpp::SignalHandlerOptions::None);
    rclcpp::init(argc, argv);

    constexpr uint16_t connection_listen_port{60000};

    cb::container::ts::deque<std::shared_ptr<cb::types::underlying::rx::frame>> receive_deque;
    cb::container::ts::deque<std::shared_ptr<cb::types::underlying::tx::frame>> transmit_deque;

    std::cout << cb::utility::get_current_timestamp() 
              << " [bridge main thread] spawning connection server thread from thread: " << std::this_thread::get_id() << std::endl;
    auto connection_server_thread_handler_ptr{std::make_shared<cb::connection::tcp::server<cb::connection::tcp::session>>(
        connection_listen_port, &receive_deque, &transmit_deque
    )};
    std::cout << cb::utility::get_current_timestamp() 
              << " [bridge main thread] launching bridge node from thread: " << std::this_thread::get_id() << std::endl;
    auto bridge_node_handler_ptr{std::make_shared<cb::nodes::bridge>("bridge", &receive_deque, &transmit_deque)};

    rclcpp::on_shutdown([&] { cb::utility::terminate(connection_server_thread_handler_ptr, bridge_node_handler_ptr); });
    if (!cb::utility::spin(connection_server_thread_handler_ptr, bridge_node_handler_ptr)) rclcpp::shutdown();

    return 0;
}

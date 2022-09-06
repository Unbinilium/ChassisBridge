#pragma once

#include <memory>
#include <chrono>
#include <exception>
#include <utility>
#include <iostream>

#include <asio.hpp>

#include "chassis_bridge/protocol.hpp"
#include "chassis_bridge/types.hpp"
#include "chassis_bridge/container.hpp"

namespace cb::connection {
    namespace tcp {
        class session : public std::enable_shared_from_this<session> {
            using rx_deque_item = std::shared_ptr<cb::types::underlying::rx::frame>;
            using tx_deque_item = std::shared_ptr<cb::types::underlying::tx::frame>;
        public:
            session(asio::ip::tcp::socket                    socket,
                    cb::container::ts::deque<rx_deque_item>* receive_deque_ptr,
                    cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr
            ) : socket_(std::move(socket)),
                receive_deque_ptr_(receive_deque_ptr),
                transmit_deque_ptr_(transmit_deque_ptr) {
                    std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                              << " [tcp session] client connected from: " << socket_.remote_endpoint() << std::endl;
            }
            ~session() = default;

            void start() {
                do_read();
                do_write();
                do_write_heartbeat();
            }

        protected:
            virtual void on_reading_header(asio::ip::tcp::socket&) {}
            virtual void on_reading_body(asio::ip::tcp::socket&) {}
            virtual void on_reading_finished(asio::ip::tcp::socket&) {
                std::this_thread::yield();
                do_read();
            }
            virtual void on_writing_finished(asio::ip::tcp::socket&) {
                std::this_thread::yield();
                transmit_deque_ptr_->wait();
                do_write();
            }
            virtual void on_heartbeat_finished(asio::ip::tcp::socket&) {
                static auto start{std::chrono::system_clock::now()};
                auto end{std::chrono::system_clock::now()};
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(100ms - (end - start));
                start = end;
                do_write_heartbeat();
            }

        private:
            void do_read() {
                auto self(shared_from_this());
                auto header{std::make_shared<cb::types::underlying::head>()};
                socket_.async_read_some(
                    asio::buffer(reinterpret_cast<void*>(header.get()), sizeof(cb::types::underlying::head)),
                    [this, self, header = std::move(header)](std::error_code ec, std::size_t byte) {
                        auto rx_frame{std::make_shared<cb::types::underlying::rx::frame>()};
                        if (ec) [[unlikely]] {
                            std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                      << " [tcp session] read " << byte << " byte header failed: " << ec.message() << std::endl;
                            socket_.close();
                            std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                      << " [tcp session] session closed from: " << socket_.remote_endpoint() << std::endl;
                            return;
                        }
                        if (*header == rx_frame->header) {
                            std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                      << " [tcp session] read " << byte << " byte header: '" << *header << "'" << std::endl;
                            on_reading_header(socket_);
                            socket_.async_read_some(
                                asio::buffer(reinterpret_cast<void*>(&rx_frame->body), sizeof(cb::types::underlying::rx::data)),
                                [this, self, rx_frame = std::move(rx_frame)](std::error_code ec, std::size_t byte) {
                                    if (ec) [[unlikely]] {
                                        std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                                  << " [tcp session] read " << byte << " byte header failed: "  << ec.message() << std::endl;
                                        socket_.close();
                                        std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                                  << " [tcp session] session closed from: " << socket_.remote_endpoint() << std::endl;
                                        return;
                                    }
                                    std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                              << " [tcp session] read " << byte << " byte body"  << std::endl;
                                    on_reading_body(socket_);
                                    receive_deque_ptr_->push_back(std::move(rx_frame));
                                    on_reading_finished(socket_);
                            });
                        }
                });
            }

            void do_write() {
                if (transmit_deque_ptr_->empty()) on_writing_finished(socket_);
                auto self(shared_from_this());
                asio::async_write(socket_,
                    asio::buffer(reinterpret_cast<void*>(transmit_deque_ptr_->front().get()), sizeof(cb::types::underlying::tx::frame)),
                    [this, self](std::error_code ec, std::size_t byte) {
                        if (ec) [[unlikely]] {
                            std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                      << " [tcp session] write " << byte << " byte frame failed: "  << ec.message() << std::endl;
                            socket_.close();
                            std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                      << " [tcp session] session closed from: " << socket_.remote_endpoint() << std::endl;
                            return;
                        }
                        std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                  << " [tcp session] write " << byte << " byte frame"  << std::endl;
                        transmit_deque_ptr_->pop_front();
                        on_writing_finished(socket_);
                });
            }

            void do_write_heartbeat() {
                auto self(shared_from_this());
                static auto heartbeat{cb::types::underlying::tx::heartbeat()};
                asio::async_write(socket_,
                    asio::buffer(reinterpret_cast<void*>(&heartbeat), sizeof(cb::types::underlying::tx::heartbeat)),
                    [this, self](std::error_code ec, std::size_t byte) {
                        if (ec) [[unlikely]] {
                            std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                      << " [tcp session] write " << byte << " byte heartbeat failed: "  << ec.message() << std::endl;
                            socket_.close();
                            std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                      << " [tcp session] session closed from: " << socket_.remote_endpoint() << std::endl;
                            return;
                        }
                        on_heartbeat_finished(socket_);
                });
            }

            asio::ip::tcp::socket                    socket_;
            cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
            cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr_;
        };


        template <typename T = session>
        class server {
            using rx_deque_item = std::shared_ptr<cb::types::underlying::rx::frame>;
            using tx_deque_item = std::shared_ptr<cb::types::underlying::tx::frame>;
        public:
            server(asio::io_context&                        io_context,
                   uint16_t                                 port,
                   cb::container::ts::deque<rx_deque_item>* receive_deque_ptr,
                   cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr
            ) : acceptor_(io_context, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
                socket_(io_context),
                receive_deque_ptr_(receive_deque_ptr),
                transmit_deque_ptr_(transmit_deque_ptr) {
                std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                          << " [tcp server] spawned tcp connection server thread: " << std::this_thread::get_id() << std::endl;
                do_accept();
            }
            ~server() = default;

        protected:
            virtual void on_accepting_finished() {
                std::this_thread::yield();
                do_accept();
            }

        private:
            void do_accept() {
                acceptor_.async_accept(socket_, [this](std::error_code ec) {
                    if (ec) [[unlikely]] {
                        std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                  << " [tcp server] accept connection failed: " << ec.message() << std::endl;
                    } else {
                        std::cout << std::chrono::system_clock::now().time_since_epoch().count()
                                  << " [tcp server] accept connection success from: " << socket_.remote_endpoint() << std::endl;
                        std::make_shared<T>(std::move(socket_), receive_deque_ptr_, transmit_deque_ptr_)->start();
                    }
                    on_accepting_finished();
                });
            }

            asio::ip::tcp::acceptor                  acceptor_;
            asio::ip::tcp::socket                    socket_;
            cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
            cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr_;
        };
    }
}

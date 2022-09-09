#pragma once

#include <memory>
#include <thread>
#include <utility>
#include <exception>
#include <stop_token>
#include <iostream>

#include <asio.hpp>

#include "chassis_bridge/protocol.hpp"
#include "chassis_bridge/types.hpp"
#include "chassis_bridge/container.hpp"
#include "chassis_bridge/utility.hpp"

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
                    std::cout << cb::utility::get_current_timestamp()
                              << " [tcp session] client connected from: " << socket_.remote_endpoint() << std::endl;
            }
            ~session() = default;

            void start() {
                auto self{shared_from_this()};
                std::jthread([this, self] { do_read(self); }).detach();
                std::jthread([this, self] { do_write(self); }).detach();
            }

            void terminate() {
                if (socket_.is_open()) {
                    socket_.cancel();
                    socket_.close();
                }
                receive_deque_ptr_->notify_all();
                transmit_deque_ptr_->notify_all();
            }

        protected:
            virtual void on_reading_finished(asio::ip::tcp::socket&, std::shared_ptr<session> self) {
                if (socket_.is_open()) do_read(std::move(self));
            }

            virtual void on_writing_finished(asio::ip::tcp::socket&, std::shared_ptr<session> self) {
                transmit_deque_ptr_->wait();
                if (socket_.is_open()) do_write(std::move(self));
            }

        private:
            void do_read(std::shared_ptr<session> self) {
                auto header{std::make_shared<cb::types::underlying::head>()};
                asio::async_read(socket_,
                    asio::buffer(reinterpret_cast<void*>(header.get()), sizeof(cb::types::underlying::head)),
                    [this, self = std::move(self), header = std::move(header)](std::error_code ec, std::size_t byte) {
                        auto rx_frame{std::make_shared<cb::types::underlying::rx::frame>()};
                        [[unlikely]] if (ec) {
                            std::cout << cb::utility::get_current_timestamp()
                                      << " [tcp session] read " << byte << " byte header failed: " << ec.message() << std::endl;
                            std::cout << cb::utility::get_current_timestamp()
                                      << " [tcp session] closing session from: " << socket_.remote_endpoint() << std::endl;
                            terminate();
                        } else if (*header == rx_frame->header) asio::async_read(socket_,
                            asio::buffer(reinterpret_cast<void*>(&rx_frame->body), sizeof(cb::types::underlying::rx::data)),
                            [this, self = std::move(self), rx_frame = std::move(rx_frame)](std::error_code ec, std::size_t byte) {
                                [[unlikely]] if (ec) {
                                    std::cout << cb::utility::get_current_timestamp()
                                              << " [tcp session] read " << byte << " byte body failed: "  << ec.message() << std::endl;
                                    std::cout << cb::utility::get_current_timestamp()
                                              << " [tcp session] closing session from: " << socket_.remote_endpoint() << std::endl;
                                    terminate();
                                } else {
                                    receive_deque_ptr_->push_back(std::move(rx_frame));
                                    on_reading_finished(socket_, std::move(self));
                                }
                        });
                        else on_reading_finished(socket_, std::move(self));
                });
            }

            void do_write(std::shared_ptr<session> self) {
                if (transmit_deque_ptr_->empty()) on_writing_finished(socket_, std::move(self));
                else asio::async_write(socket_,
                    asio::buffer(reinterpret_cast<void*>(transmit_deque_ptr_->front().get()), sizeof(cb::types::underlying::tx::frame)),
                    [this, self = std::move(self)](std::error_code ec, std::size_t byte) {
                        [[unlikely]] if (ec) {
                            std::cout << cb::utility::get_current_timestamp()
                                      << " [tcp session] write " << byte << " byte frame failed: "  << ec.message() << std::endl;
                            std::cout << cb::utility::get_current_timestamp()
                                      << " [tcp session] closing session from: " << socket_.remote_endpoint() << std::endl;
                            terminate();
                        } else {
                            transmit_deque_ptr_->pop_front();
                            on_writing_finished(socket_, std::move(self));
                        }
                });
            }

            asio::ip::tcp::socket                    socket_;
            cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
            cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr_;
        };


        template <class Session = session>
        class server {
            using rx_deque_item = std::shared_ptr<cb::types::underlying::rx::frame>;
            using tx_deque_item = std::shared_ptr<cb::types::underlying::tx::frame>;
        public:
            server(const uint16_t                           port,
                   cb::container::ts::deque<rx_deque_item>* receive_deque_ptr,
                   cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr
            ) : io_context_({}),
                acceptor_(io_context_, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
                socket_(io_context_),
                receive_deque_ptr_(receive_deque_ptr),
                transmit_deque_ptr_(transmit_deque_ptr) {
                server_thread_ = std::jthread([this](std::stop_token st) {
                    server_start: try {
                        do_accept();
                        io_context_.run();
                    } catch (std::exception& e) {
                        std::cout << cb::utility::get_current_timestamp() 
                                  << " [tcp server] restarting connection server with exception: " << e.what() << std::endl;
                        if (st.stop_requested()) return;
                        io_context_.reset();
                        goto server_start;
                    }
                });
                std::cout << cb::utility::get_current_timestamp()
                          << " [tcp server] spawned tcp connection server thread: " << server_thread_.get_id() << std::endl;
            }

            ~server(){ terminate(); };

            void terminate() {
                if (server_thread_.joinable()) server_thread_.request_stop();
                if (socket_.is_open()) {
                    socket_.cancel();
                    socket_.close();
                }
                io_context_.stop();
            }

            bool spin() {
                if (server_thread_.joinable()) server_thread_.join();
                return false;
            }

        protected:
            virtual void on_accepting_finished() {
                if (server_thread_.get_stop_token().stop_requested()) terminate();
                else do_accept();
            }

        private:
            void do_accept() {
                acceptor_.async_accept(socket_, [this](std::error_code ec) {
                    [[unlikely]] if (ec) std::cout << cb::utility::get_current_timestamp()
                                                   << " [tcp server] accept connection failed: " << ec.message() << std::endl;
                    else {
                        std::cout << cb::utility::get_current_timestamp()
                                  << " [tcp server] accept connection success from: " << socket_.remote_endpoint() << std::endl;
                        std::make_shared<Session>(std::move(socket_), receive_deque_ptr_, transmit_deque_ptr_)->start();
                    }
                    on_accepting_finished();
                });
            }

            asio::io_context                         io_context_;
            asio::ip::tcp::acceptor                  acceptor_;
            asio::ip::tcp::socket                    socket_;
            std::jthread                             server_thread_;
            cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
            cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr_;
        };
    }
}

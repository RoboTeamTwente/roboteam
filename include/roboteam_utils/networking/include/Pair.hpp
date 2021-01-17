#ifndef __PAIR_HPP__
#define __PAIR_HPP__

#define ZMQ_BUILD_DRAFT_API 1
#include <zmqpp/zmqpp.hpp>
#include <stx/result.h>

namespace rtt::networking {
    template <size_t port>
    struct PairReceiver {
        zmqpp::context context;
        zmqpp::socket socket;

        PairReceiver()
            : socket{ context, zmqpp::socket_type::pair } {
            socket.connect("tcp://127.0.0.1:" + std::to_string(port));
            // The AI should read every tick, this is the only socket that will still have a
            // background thread running for reading.
        }

        template <typename T>
        stx::Result<T, std::string> read_next(bool dont_block = true) {
            zmqpp::message msg;
            std::string data{};
            if (!socket.receive(msg, dont_block)) {
                return stx::Err(std::move(data));
            }
            msg >> data;
            T object;
            auto succ = object.ParseFromString(data);
            if (succ) {
                return stx::Ok(std::move(object));
            }
            return stx::Err(std::move(data));
        }

        template <typename T>
        void write(T const& s, const bool dont_block = false) {
            std::string out;
            s.SerializeToString(&out);
            socket.send(out,dont_block);
        }

        bool is_ok() {
            return static_cast<bool>(socket);
        }
    };
}  // namespace rtt::networking

#endif
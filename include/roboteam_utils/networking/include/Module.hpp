#ifndef __MODULE_HPP__
#define __MODULE_HPP__

#define ZMQ_BUILD_DRAFT_API 1
#include <roboteam_proto/Handshake.pb.h>
#include <roboteam_proto/State.pb.h>
#include <stx/result.h>

#include <zmqpp/zmqpp.hpp>

namespace rtt::networking {
    struct Module {
        zmqpp::context context;
        zmqpp::socket socket;

        Module(proto::Handshake handshake);

        stx::Result<proto::ModuleState, std::string> read_next(bool dont_block = true);

        bool is_ok() const noexcept;
    };
}  // namespace rtt::networking

#endif
#include "networking/Module.hpp"

namespace rtt::networking {
Module::Module(proto::Handshake handshake) : socket{context, zmqpp::socket_type::client} {
    socket.connect("tcp://127.0.0.1:16969");
    socket.send(handshake.SerializeAsString());
}

/*
stx::Result<proto::ModuleState, std::string> Module::read_next(bool dont_block) {
    zmqpp::message msg;
    std::string data{};
    if (!socket.receive(msg, dont_block)) {
        return stx::Err(std::move(data));
    }
    msg >> data;
    proto::ModuleState object;
    auto succ = object.ParseFromString(data);
    if (succ) {
        return stx::Ok(std::move(object));
    }
    return stx::Err(std::move(data));
}
*/
bool Module::is_ok() const noexcept { return static_cast<bool>(socket); }
}  // namespace rtt::networking
#include <torch/torch.h>
#include <iostream>

namespace rtt::ai::rl {

rl::rl(std::string name) {
    torch::Tensor tensor = torch::rand({2, 3});
    std::cout << tensor << std::endl;
}
}  // namespace rtt::ai::rl
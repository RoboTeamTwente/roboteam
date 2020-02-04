#pragma once

#include "bt/Decorator.h"

namespace bt {

class Inverter : public Decorator {
   public:
    Status update() override;
    std::string node_name() override { return "Inverter"; };
};

}  // namespace bt

#include "roboteam_utils/normalise.h"
#include "roboteam_utils/constants.h"

#include <string>


namespace roboteam_utils {

    using namespace roboteam_msgs;

    World normalize_world(World world) {
        bool should_normalize = false;
        std::string our_side;

        get_PARAM_NORMALISE_FIELD(should_normalize);
        get_PARAM_OUR_SIDE(our_side);


        if (!(should_normalize && our_side == "right")) {
            // No need to normalize.
            return world;
        }

        World norm_world(world);

        // Rotate the ball.
        norm_world.ball.pos.x *= -1;
        norm_world.ball.pos.y *= -1;

        norm_world.ball.vel.x *= -1;
        norm_world.ball.vel.y *= -1;


        return norm_world;
    }
}

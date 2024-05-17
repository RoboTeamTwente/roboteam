#ifndef RTT_WORLD_HPP
#define RTT_WORLD_HPP
#include <proto/RobotFeedback.pb.h>
#include <roboteam_utils/Print.h>

#include <vector>

#include "WorldData.hpp"
#include "control/positionControl/PositionControl.h"
#include "world/views/WorldDataView.hpp"

namespace rtt::world {

/**
 * @brief Class that represents the world and history of the world.
 *
 * This Class operates under the assumption of an immutable state
 * Adjusting any value inside this world after construction may
 * result in undefined behavior. Any sort of mutation of already constructed
 * WorldData will break the entire system and introduce data-races due to
 * a shared immutable state.
 *
 * WorldData is a snapshot of the current world as seen by the camera
 * Feedback is already applied to the robots when they are constructed
 * As soon as a new world is constructed, which should only be done
 * through updateWorld, the current world is pushed to history, given that
 * this is a Some variant. Using any other unconventional way to push
 * a world to history will result in undefined behavior.
 *
 * The entire system with World relies on immutability,
 * this is something that Rust and haskell enforce and is seen as
 * one of the only ways to keep a project datarace-free
 */
class World {
    /**
     * @brief Structure that provides structured binding support for the world.
     * @param T Type of data to lock (a pointer to!)
     *
     * AcquireInfo<int> get_int() {
     *     static std::mutex mtx;
     *     static int value;
     *     return AcquireInfo{ mtx, &value };
     * }
     *
     * auto const& [_, t_obj] = something::instance();
     * // _ is a lock guard
     * // t_obj is a pointer to T
     */
    template <typename T>
    struct AcquireInfo {
        std::lock_guard<std::mutex> mtx; /**< Lock guard for the info */
        T *data;                         /**< Pointer to the type of data to lock */
    };

   public:
    /**
     * @brief Global singleton for World, scott-meyers style
     * @param[in] resetWorld Boolean that marks whether to reset the world before returning
     * if set to true, world.clear() is called
     * if set to false, worldInstance is simply reset
     * @return A pointer to a static World
     */
    inline static AcquireInfo<World> instance() {
        static World worldInstance{};
        return {std::lock_guard(worldInstance.updateMutex), &worldInstance};
    }

    /**
     * @brief Delete copy constructor for the World class
     */
    World(World const &) = delete;

    /**
     * @brief Delete copy assignment
     */
    World &operator=(World &) = delete;

    /**
     * @brief Delete move constructor for the World class
     */
    World(World &&) = delete;

    /**
     * @brief Delete move assignment operator
     */
    World &operator=(World &&) = delete;

    constexpr static size_t HISTORY_SIZE = 200; /**< Amount of ticks to store in history */

    /**
     * @brief Constructs a World from settings
     * @param settings Settings to use to construct the world, will be stored
     * Undefined behavior if the pointer to settings outlives the lifetime of the settings instance
     * Usage of settings before construction of the STPManager will result in undefined
     * behavior due to uninitialized memory
     */
    explicit World();

    /**
     * @brief Updates the currentWorld
     * @param world World to construct currentWorld from
     */
    void updateWorld(proto::World &protoWorld);

    /**
     * @brief Updates the currentField
     * @param field Field to construct currentField from
     */
    void updateField(proto::SSL_GeometryFieldSize &protoField);

    /**
     * @brief Updates the current field
     * @param protoField Field to update the current field with
     */
    void updateField(Field &protoField);

    /**
     * @brief Gets the current world
     * @return std::nullopt if there is no currentWorld, otherwise Some with the value
     */
    [[nodiscard]] std::optional<view::WorldDataView> getWorld() const noexcept;

    /**
     * @brief Gets the current field
     * @return std::nullopt if there is no currentField, otherwise Some with the value
     */
    [[nodiscard]] std::optional<Field> getField() const noexcept;

    /**
     * @brief Gets a certain world from history
     * @param ticksAgo Amount of ticks ago to fetch world from.
     *     0 gives the current world
     *     1 gives the freshest world in history
     *     HISTORY_SIZE gives the oldest world in history
     * @return Returns the world at index currentIndex - ticksAgo
     */
    [[nodiscard]] std::optional<view::WorldDataView> getHistoryWorld(size_t ticksAgo) const noexcept;

    /**
     * @brief Gets the history size
     * @return size_t The amount of elements in the history
     */
    [[nodiscard]] size_t getHistorySize() const noexcept;

    /**
     * @brief Get a pointer to the general position control, which can be used by all robots.
     * If the object does not exist, it is created
     * @return position control object. See its documentation for more info
     */
    [[nodiscard]] ai::control::PositionControl *getRobotPositionController() noexcept;

#ifdef RUNNING_TEST
    /**
     * Resets the world
     * Resets every member, sets tick duration to 0, etc etc...
     * Everything EXCEPT positioncontrol and settings is reset.
     */
    void reset() noexcept {
        history.clear();
        currentIndex = 0;
        currentWorld.reset();
        currentField.reset();
        lastTick = 0;
        tickDuration = 0;
    }
#endif

   private:
    /**
     * @brief Updates the tickCount, sets lastTick to now(), sets duration to oldNow - now();
     */
    void updateTickTime() noexcept;

    /**
     * @brief Sets the current world, also pushes the old currentWorld into history if this is Some
     * Usage of currentWorld after call will result in UB
     * @param currentWorld New currentWorld
     * @return Returns a reference to the new currentWorld
     */
    WorldData const &setWorld(WorldData &currentWorld) noexcept;

    /**
     * @brief Pushes a world to history, takes ownership of world
     * @param world WorldData to be taken ownership of
     */
    void toHistory(WorldData &world) noexcept;

    std::mutex updateMutex;                               /**< Mutex used when constructing robots to prevent updating of updateMap without wanting it */
    std::vector<rtt::world::WorldData> history;           /**< History of the world, this is where old world data is pushed to */
    size_t currentIndex{0};                               /**< Current index into the circular buffer that is the world history */
    std::optional<WorldData> currentWorld = std::nullopt; /**< Current world, None if no world has been constructed yet, Some if a world is valid */
    std::optional<Field> currentField;                    /**< The current field data */
    uint64_t lastTick;                                    /**< Timestamp of the last tick */
    uint64_t tickDuration{};                              /**< Duration between ticks */
    ai::control::PositionControl positionControl;         /**< The position controller, initially null */
};
}  // namespace rtt::world

#endif  // RTT_WORLD_HPP
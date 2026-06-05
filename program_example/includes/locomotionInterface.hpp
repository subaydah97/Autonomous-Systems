/// @brief Used to control motors.
class LocomotionInterface
{
private:
public:
    /// @brief Tells the motor to move at a constant rate.
    /// @param speed
    /// @return Error codes
    virtual int spin_continously(float speed) = 0;
    /*Error codes:
    0   : No errors
    1   :
    */

    /// @brief Tells the robot to move forward at a constant speed, without ever stopping.
    /// @param speed M/s,
    /// @param angle Radians clockwise, unit circle
    /// @return Error codes
    virtual int spin_toPosition(float speed, float angle) = 0;
    /*Error codes:
    0   : No errors
    1   :
    */

    LocomotionInterface(/* args */);
    ~LocomotionInterface();
};

LocomotionInterface::LocomotionInterface(/* args */)
{
}

LocomotionInterface::~LocomotionInterface()
{
}

/// @brief Used to control motors.
class motorInterface
{
private:
public:
    //Radians always refer to the radians at the outmost edge of the wheel.
    float radiansPerFullRotation;
    /// @brief A float that should range between -1 and 1. Values outside of this range cannot be achieved by this motor.
    float motorInput;

    /// @brief Tells the motor to move at a constant rate.
    /// @param speed
    /// @return Error codes
    virtual int spinContinously(float speed) = 0;
    /*Error codes:
    0   : No errors
    1   :
    */

    /// @brief Tells the robot to move, at a given speed, to a given position.
    /// @param speed M/s,
    /// @param angle Radians clockwise, unit circle
    /// @return Error codes
    virtual int spinToPosition(float speed, float position) = 0;
    /*Error codes:
    0   : No errors
    1   :
    */

    /// @brief Returns the motor input required to achieve the desired speed.
    /// @param desiredSpeed Radians/s
    /// @return A float that should range between -1 and 1. Values outside of this range cannot be achieved by this motor.
    virtual float speedEquation(float desiredSpeed) = 0;

    /// @brief Gets the current possition of the Motor.
    /// @return The Motor position in Radians. Depending on the implementation, this value may not be able to natively track full rotations.
    virtual float getMotorPosition() = 0;

    /// @brief Gets the current velocity of the Motor.
    /// @return The velocity in Radians per second.
    virtual float getMotorVelocity() = 0;

    motorInterface(/* args */);
    ~motorInterface();
};

motorInterface::motorInterface(/* args */)
{
}

motorInterface::~motorInterface()
{
}

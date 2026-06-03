class LocomotionInterface
{
private:
public:
    /// @brief Tells the robot to move forward at a constant speed, without ever stopping.
    /// @param speed M/s,  
    /// @param angle Radians clockwise, unit circle
    /// @return Error codes
    virtual int move(float speed, float angle) = 0;
    /*Error codes:
    0   :
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

class Helicopter
{
  public:
    Helicopter();
    void calculate(const sensor_msgs::Joy::ConstPtr joy, jaws_msgs::Thrusters::Ptr thrusters);
  private:
    float max_angle;
    float max_power;
};

Helicopter::Helicopter()
{
}

void Helicopter::calculate(const sensor_msgs::Joy::ConstPtr joy, jaws_msgs::Thrusters::Ptr thrusters)
{
}

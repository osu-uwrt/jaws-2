class Airplane
{
  public:
    Airplane(float m_a, float m_p);
    void calculate(const sensor_msgs::Joy::ConstPtr joy, jaws_msgs::Thrusters::Ptr thrusters);
  private:
    float max_angle;
    float max_power;
};

Airplane::Airplane(float m_a, float m_p)
{
  max_angle = m_a;
  max_power = m_p;
}

void Airplane::calculate(const sensor_msgs::Joy::ConstPtr joy, jaws_msgs::Thrusters::Ptr thrusters)
{
  thrusters->port_angle = joy->axes[2] * max_angle;
  thrusters->stbd_angle = joy->axes[2] * max_angle;

  thrusters->port_power = joy->axes[1] * max_power;
  thrusters->stbd_power = joy->axes[1] * max_power;
  thrusters->port_power *= 1 + (joy->axes[12] - joy->axes[13]) / 2;
  thrusters->stbd_power *= 1 + (joy->axes[13] - joy->axes[12]) / 2;

  thrusters->aft_power = joy->axes[3] * max_power;
}

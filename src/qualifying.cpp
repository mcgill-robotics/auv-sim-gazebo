#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <string>

int main(int argc, char** argv)
{
  ignition::transport::Node node;
  std::string topic = "/model/clarke/joint/thruster0_joint/cmd_pos";
  iauto pub = node.Advertise<ignition::msgs::StringMsg>(topic);

  if (!pub) {
    std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
    return -1;
  }

  ignition::msgs::StringMsg msg;
  msg.set_data("ignition.msgs.Double -p 'data: 15'");

  pub.Publish(msg);

  return 0;
}

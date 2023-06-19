#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>

namespace gazebo
{
  class RepublishPlugin : public WorldPlugin
  {
    private:
      physics::WorldPtr world;
      event::ConnectionPtr updateConnection;
      std::shared_ptr<rclcpp::Node> node;
      std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> landPadPublishers;
      std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> chargingStationPublishers;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      transport::NodePtr gznode;
      transport::SubscriberPtr sub;
      int pad = 0;
      int spot = 0;
      double updateRate;  // Update rate in seconds
      double elapsedTime; // Elapsed time since the last update

    public:
      void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
      {

        InitializeGazeboNode();
        world = _world;

        node = std::make_shared<rclcpp::Node>("republish_plugin_node");

        double desiredUpdateRate = 0.5; // Adjust this value as needed
        updateRate = 1.0 / desiredUpdateRate;
        elapsedTime = 0.0;

         // Create publishers for land pads
        for (int station = 1; station < 5; ++station)
        {
          for (int pad = 1; pad < 5; ++pad)
          {
            std::string topic = "/landpad_station_" + std::to_string(station) + "/land_pad_" + std::to_string(pad) + "/contacts";
            this->publisher_ = node->create_publisher<std_msgs::msg::String>(topic, 10);
            this->landPadPublishers.push_back(this->publisher_);
          }
        }

        // Create publishers for charging stations
        for (int station = 1; station < 5; ++station)
        {
          for (int spot = 1; spot < 5; ++spot)
          {
            std::string topic = "/charging_station_" + std::to_string(station) + "/charging_spot_" + std::to_string(spot) + "/contacts";
            this->publisher_ = node->create_publisher<std_msgs::msg::String>(topic, 10);
            this->chargingStationPublishers.push_back(this->publisher_);
          }
        }

        std::cout << "size of land pads: " << this->landPadPublishers.size() << '\n';
        std::cout << "size of charging stations: " << this->chargingStationPublishers.size() << '\n';

        // Subscribe to Gazebo world update event
        updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RepublishPlugin::OnUpdate, this, _1));
      }

      void InitializeGazeboNode()
      {
        std::cout << "Initializing Gazebo..." << std::endl;
        this->gznode = transport::NodePtr(new transport::Node());
        this->gznode->Init();
      }

      void OnUpdate(const common::UpdateInfo & /*_info*/)
      {

        elapsedTime += world->Physics()->GetMaxStepSize();

        if (elapsedTime >= updateRate)
        {
          // Reset the elapsed time
          elapsedTime = 0.0;

          // Republish charging station topics
          for (int station = 1; station < 5; ++station)
          {
            for (int spot = 1; spot < 5; ++spot)
            {
              std::string topic = "/gazebo/default/charging_station_" + std::to_string(station) + "/charging_spot_" + std::to_string(spot) + "/link/my_contact/contacts";
              std::cout << topic << std::endl;
              this->sub = this->gznode->Subscribe(topic, &RepublishPlugin::ChargingStationContactCallback, this);
            }
            for (int pad = 1; pad < 5; ++pad)
            {
              std::string topic = "/gazebo/default/landpad_station_" + std::to_string(station) + "/land_pad_" + std::to_string(pad) + "/link/my_contact/contacts";
              std::cout << topic << std::endl;
              this->sub = this->gznode->Subscribe(topic, &RepublishPlugin::LandPadContactCallback, this);
            }
          }
        }
      }

      void LandPadContactCallback(const boost::shared_ptr<const gazebo::msgs::Contacts>& msg)
      {
        std_msgs::msg::String contactString;

        if (this->pad > 15)
        {
          this->pad = 0;
        }


        std::cout << "size of msg: " << msg->contact_size() << '\n';

        // Iterate through each contact in the message
        for (int i = 0; i < msg->contact_size(); ++i)
        {
          const auto &contact = msg->contact(i);

          // Concatenate collision1 and collision2 with a separator (e.g., comma)
          std::string contactInfo = contact.collision1() + "," + contact.collision2();

          std::cout << "Concact Info: " << contactInfo << '\n';

          // Append the contact info to the contact string
          contactString.data += contactInfo + "\n";
        }
        // Publish the contact string to the corresponding rclcpp topic for land pads
        this->landPadPublishers[this->pad]->publish(contactString);
        this->pad++;

      }

      void ChargingStationContactCallback(const boost::shared_ptr<const gazebo::msgs::Contacts>& msg)
      {
        std_msgs::msg::String contactString;
        if (this->spot > 15)
        {
          this->spot = 0;
        }
        // Iterate through each contact in the message
        for (int i = 0; i < msg->contact_size(); ++i)
        {
          const auto &contact = msg->contact(i);

          // Concatenate collision1 and collision2 with a separator (e.g., comma)
          std::string contactInfo = contact.collision1() + "," + contact.collision2();

          // Append the contact info to the contact string
          contactString.data += contactInfo + "\n";
        }

        // Publish the contact string to the corresponding rclcpp topic for charging stations
        this->chargingStationPublishers[this->spot]->publish(contactString);
        this->spot++;
      }
  };
  GZ_REGISTER_WORLD_PLUGIN(RepublishPlugin)
}

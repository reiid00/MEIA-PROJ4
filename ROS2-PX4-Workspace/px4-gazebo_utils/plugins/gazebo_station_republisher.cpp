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
      std::vector<transport::SubscriberPtr> subs;
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
            auto publisher = node->create_publisher<std_msgs::msg::String>(topic, 10);
            this->landPadPublishers.push_back(publisher);
          }
        }

        // Create publishers for charging stations
        for (int station = 1; station < 5; ++station)
        {
          for (int spot = 1; spot < 5; ++spot)
          {
            std::string topic = "/charging_station_" + std::to_string(station) + "/charging_spot_" + std::to_string(spot) + "/contacts";
            auto publisher = node->create_publisher<std_msgs::msg::String>(topic, 10);
            this->chargingStationPublishers.push_back(publisher);
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
              subs.push_back(this->gznode->Subscribe(topic, &RepublishPlugin::ChargingStationContactCallback, this));
            }

            for (int pad = 1; pad < 5; ++pad)
            {
              std::string topic = "/gazebo/default/landpad_station_" + std::to_string(station) + "/land_pad_" + std::to_string(pad) + "/link/my_contact/contacts";
              subs.push_back(this->gznode->Subscribe(topic, &RepublishPlugin::LandPadContactCallback, this));
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

        int m = 0;


        // Iterate through each contact in the message
        for (int i = 0; i < msg->contact_size(); ++i)
        {
          const auto &contact = msg->contact(i);

          // Concatenate collision1 and collision2 with a separator (e.g., comma)
          std::string contactInfo = contact.collision1() + "," + contact.collision2();

          m = calculateValue(contact.collision1(), contact.collision2(), false);

          // Append the contact info to the contact string
          contactString.data += contactInfo + "\n";
        }
        // Publish the contact string to the corresponding rclcpp topic for land pads
        this->landPadPublishers[m]->publish(contactString);
        this->pad++;

      }

      void ChargingStationContactCallback(const boost::shared_ptr<const gazebo::msgs::Contacts>& msg)
      {
        std_msgs::msg::String contactString;
        if (this->spot > 15)
        {
          this->spot = 0;
        }

        int m = 0;

        // Iterate through each contact in the message
        for (int i = 0; i < msg->contact_size(); ++i)
        {
          const auto &contact = msg->contact(i);

          // Concatenate collision1 and collision2 with a separator (e.g., comma)
          std::string contactInfo = contact.collision1() + "," + contact.collision2();

          m = calculateValue(contact.collision1(), contact.collision2(), true);

          // Append the contact info to the contact string
          contactString.data += contactInfo + "\n";
        }

        // Publish the contact string to the corresponding rclcpp topic for charging stations
        this->chargingStationPublishers[m]->publish(contactString);
        this->spot++;
      }

      int calculateValue(const std::string& contact1, const std::string& contact2, bool isChargingType = false)
      {
          std::string prefix = (isChargingType) ? "charging_station" : "landpad_station";

          std::string contactToCheck;
          if (contact1.find(prefix) == 0)
              contactToCheck = contact1;
          else if (contact2.find(prefix) == 0)
              contactToCheck = contact2;
          else
              return 0; // No contact with the desired prefix was found

        // Find the positions of the first and second colons
          size_t firstColonPos = contactToCheck.find(':');
          if (firstColonPos == std::string::npos)
              return 0; // Invalid contact format

          size_t secondColonPos = contactToCheck.find(':', firstColonPos + 2);
          if (secondColonPos == std::string::npos)
              return 0; // Invalid contact format

          // Extract the parent and child indexes
          std::string parentIndexStr = contactToCheck.substr(prefix.length() + 1, firstColonPos - prefix.length() - 1);
          std::string childIndexStr = contactToCheck.substr(secondColonPos - 1);

          try {
              int parentIndex = std::stoi(parentIndexStr);
              int childIndex = std::stoi(childIndexStr);

              return ((parentIndex - 1) * 4) + childIndex -1;
          }
          catch (std::exception& e) {
              return 0; // Invalid index format
          }
      }
  };
  GZ_REGISTER_WORLD_PLUGIN(RepublishPlugin)
}

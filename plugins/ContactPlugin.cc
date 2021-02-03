#include "ContactPlugin.hh"
#include <string>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i) {

    std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

    

    if ((contacts.contact(i).collision1().find("pushcart")!=std::string::npos || 
        contacts.contact(i).collision2().find("pushcart")!=std::string::npos) && 
        ((contacts.contact(i).collision1().find("reemc")!=std::string::npos || 
        contacts.contact(i).collision2().find("reemc")!=std::string::npos) ) ) {
        
        std::cout << "########################## collision between cart and any robot link \n";
        math::Vector3 actualForce;
        
        for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j) {
            std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
            
            actualForce.x =
                contacts.contact(i).wrench(j).body_1_wrench().force().x();
            actualForce.y =
                contacts.contact(i).wrench(j).body_1_wrench().force().y();
            actualForce.z =
                contacts.contact(i).wrench(j).body_1_wrench().force().z();
                
            std::cout << j << "\n  FORCE:" << actualForce << std::endl;
            std::cout << j << "\n  FORCE magnitude:" << actualForce.GetLength() << std::endl;                        
        }
    }
    
    /*for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j) {
      std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    }*/
    
    
  }
}

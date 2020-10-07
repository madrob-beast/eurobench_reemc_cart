#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <stdlib.h>

namespace gazebo
{
  class SimpleDoorConfig : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model, link and joint

      this->model = _parent;
      this->joint=  this->model->GetJoint("door_simple::joint_frame_door");

      if (const char* direction = std::getenv("GAZEBO_DOOR_MODEL_DIRECTION")){
        
        if(std::strcmp(direction,"push") ==0){
          joint->SetUpperLimit(0,1.57);
          joint->SetLowerLimit(0,-0.03);
        }
        else if(std::strcmp(direction,"pull") ==0){
          joint->SetUpperLimit(0,0.03);
          joint->SetLowerLimit(0,-1.57);
        }
        else if(std::strcmp(direction,"pushpull") ==0){
          joint->SetUpperLimit(0,1.57);
          joint->SetLowerLimit(0,-1.57); 
        }
        else {
          std::cerr << "Bad door direction: "<< direction << " , [push][pull][pushpull]" << std::endl;
        }

      }

      if (const char* selfClose = std::getenv("GAZEBO_DOOR_MODEL_SELFCLOSE")){
        
        if(std::strcmp(selfClose,"y") ==0){
          joint->SetStiffnessDamping(0,1.5,0.1,0.0);  //index,spring_stiffnes,damping,spring_zero_load_position
       
        }
        else if(std::strcmp(selfClose,"n") ==0){

        }

        else {
          std::cerr << "Bad door self_closing argument: "<< selfClose << " , [n][y]" << std::endl;
        }

      }
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SimpleDoorConfig::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: physics::JointPtr joint;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SimpleDoorConfig);
}


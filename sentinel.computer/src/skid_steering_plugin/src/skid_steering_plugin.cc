#include "skid_steering/skid_steering_plugin.hh"

 #include <gz/msgs/odometry.pb.h>
 
 #include <limits>
 #include <mutex>
 #include <set>
 #include <string>
 #include <vector>
 
 #include <gz/common/Profiler.hh>
 #include <gz/math/Quaternion.hh>
 #include <gz/math/DiffDriveOdometry.hh>

 #include <gz/math/SpeedLimiter.hh>
 #include <gz/plugin/Register.hh>
 #include <gz/transport/Node.hh>
 
 #include "gz/sim/components/CanonicalLink.hh"
 #include "gz/sim/components/JointPosition.hh"
 #include "gz/sim/components/JointVelocityCmd.hh"
 #include "gz/sim/Link.hh"
 #include "gz/sim/Model.hh"
 #include "gz/sim/Util.hh"
 
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/twist.pb.h>

 using namespace gz;
 using namespace sim;
 using namespace systems;
 
 /// \brief Velocity command.
 struct Commands
 {
   /// \brief Linear velocity.
   double lin;
 
   /// \brief Angular velocity.
   double ang;
 
   Commands() : lin(0.0), ang(0.0) {}
 };
 
 class gz::sim::systems::GazeboRosSkidSteerDrivePrivate
 {
   /// \brief Callback for velocity subscription
   /// \param[in] _msg Velocity message
   public: void OnCmdVel(const msgs::Twist &_msg);
 
   /// \brief Callback for enable/disable subscription
   /// \param[in] _msg Boolean message
   public: void OnEnable(const msgs::Boolean &_msg);
 
   /// \brief Update odometry and publish an odometry message.
   /// \param[in] _info System update information.
   /// \param[in] _ecm The EntityComponentManager of the given simulation
   /// instance.
   public: void UpdateOdometry(const UpdateInfo &_info,
     const EntityComponentManager &_ecm);
 
   /// \brief Update the linear and angular velocities.
   /// \param[in] _info System update information.
   /// \param[in] _ecm The EntityComponentManager of the given simulation
   /// instance.
   public: void UpdateVelocity(const UpdateInfo &_info,
     const EntityComponentManager &_ecm);
 
   /// \brief Gazebo communication node.
   public: transport::Node node;
 
   /// \brief Entity of the left joint
   public: std::vector<Entity> leftJoints;
 
   /// \brief Entity of the right joint
   public: std::vector<Entity> rightJoints;
 
   /// \brief Name of left joint
   public: std::vector<std::string> leftJointNames;
 
   /// \brief Name of right joint
   public: std::vector<std::string> rightJointNames;
 
   /// \brief Calculated speed of left joint
   public: double leftFrontJointSpeed{0};
 
   /// \brief Calculated speed of right joint
   public: double rightFrontJointSpeed{0};
 
   public: double leftRearJointSpeed{0};
 
   /// \brief Calculated speed of right joint
   public: double rightRearJointSpeed{0};

   /// \brief Distance between wheels
   public: double wheelSeparation{1.0};
 
   public: double wheelBase{1.0};

   /// \brief Wheel radius
   public: double wheelRadius{0.2};
 
   /// \brief Model interface
   public: Model model{kNullEntity};
 
   /// \brief The model's canonical link.
   public: Link canonicalLink{kNullEntity};
 
   /// \brief Update period calculated from <odom__publish_frequency>.
   public: std::chrono::steady_clock::duration odomPubPeriod{0};
 
   /// \brief Last sim time odom was published.
   public: std::chrono::steady_clock::duration lastOdomPubTime{0};
 
   /// \brief Diff drive odometry.
  /// \brief Odometry X value
  public: double odomX{0.0};

  /// \brief Odometry Y value
  public: double odomY{0.0};

  /// \brief Odometry yaw value
  public: double odomYaw{0.0};

  /// \brief Odometry old left value
  public: double odomOldLeft{0.0};

  /// \brief Odometry old right value
  public: double odomOldRight{0.0};
 
  public: std::chrono::steady_clock::duration lastOdomTime{0};
   /// \brief Diff drive odometry message publisher.
   public: transport::Node::Publisher odomPub;
 
   /// \brief Diff drive tf message publisher.
   public: transport::Node::Publisher tfPub;
 
   /// \brief Linear velocity limiter.
   public: std::unique_ptr<math::SpeedLimiter> limiterLin;
 
   /// \brief Angular velocity limiter.
   public: std::unique_ptr<math::SpeedLimiter> limiterAng;
 
   /// \brief Previous control command.
   public: Commands last0Cmd;
 
   /// \brief Previous control command to last0Cmd.
   public: Commands last1Cmd;
 
   /// \brief Last target velocity requested.
   public: msgs::Twist targetVel;
 
   /// \brief Enable/disable state of the controller.
   public: bool enabled;
 
   /// \brief A mutex to protect the target velocity command.
   public: std::mutex mutex;
 
   /// \brief frame_id from sdf.
   public: std::string sdfFrameId;
 
   /// \brief child_frame_id from sdf.
   public: std::string sdfChildFrameId;
 };
 
 //////////////////////////////////////////////////
 GazeboRosSkidSteerDrive::GazeboRosSkidSteerDrive()
   : dataPtr(std::make_unique<GazeboRosSkidSteerDrivePrivate>())
 {
 }
 
 //////////////////////////////////////////////////
 void GazeboRosSkidSteerDrive::Configure(const Entity &_entity,
     const std::shared_ptr<const sdf::Element> &_sdf,
     EntityComponentManager &_ecm,
     EventManager &/*_eventMgr*/)
 {
   this->dataPtr->model = Model(_entity);
 
   // Get the canonical link
   std::vector<Entity> links = _ecm.ChildrenByComponents(
       this->dataPtr->model.Entity(), components::CanonicalLink());
   if (!links.empty())
     this->dataPtr->canonicalLink = Link(links[0]);
 
   if (!this->dataPtr->model.Valid(_ecm))
   {
     gzerr << "GazeboRosSkidSteerDrive plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
     return;
   }
 
   // Get params from SDF
   auto sdfElem = _sdf->FindElement("left_front_joint");
   while (sdfElem)
   {
     this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
     sdfElem = sdfElem->GetNextElement("left_front_joint");
   }
   sdfElem = _sdf->FindElement("right_front_joint");
   while (sdfElem)
   {
     this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
     sdfElem = sdfElem->GetNextElement("right_front_joint");
   }
   sdfElem = _sdf->FindElement("left_rear_joint");
   while (sdfElem)
   {
     this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
     sdfElem = sdfElem->GetNextElement("left_rear_joint");
   }
   sdfElem = _sdf->FindElement("right_rear_joint");
   while (sdfElem)
   {
     this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
     sdfElem = sdfElem->GetNextElement("right_real_joint");
   }
 
   this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
       this->dataPtr->wheelSeparation).first;
   this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
       this->dataPtr->wheelRadius).first;
    this->dataPtr->wheelBase = _sdf->Get<double>("wheel_base",
        this->dataPtr->wheelRadius).first;
   // Instantiate the speed limiters.
   this->dataPtr->limiterLin = std::make_unique<math::SpeedLimiter>();
   this->dataPtr->limiterAng = std::make_unique<math::SpeedLimiter>();
 
   // Parse speed limiter parameters.
 
   // Min Velocity
   if (_sdf->HasElement("min_velocity"))
   {
     const double minVel = _sdf->Get<double>("min_velocity");
     this->dataPtr->limiterLin->SetMinVelocity(minVel);
     this->dataPtr->limiterAng->SetMinVelocity(minVel);
   }
   if (_sdf->HasElement("min_linear_velocity"))
   {
     const double minLinVel = _sdf->Get<double>("min_linear_velocity");
     this->dataPtr->limiterLin->SetMinVelocity(minLinVel);
   }
   if (_sdf->HasElement("min_angular_velocity"))
   {
     const double minAngVel = _sdf->Get<double>("min_angular_velocity");
     this->dataPtr->limiterAng->SetMinVelocity(minAngVel);
   }
 
   // Max Velocity
   if (_sdf->HasElement("max_velocity"))
   {
     const double maxVel = _sdf->Get<double>("max_velocity");
     this->dataPtr->limiterLin->SetMaxVelocity(maxVel);
     this->dataPtr->limiterAng->SetMaxVelocity(maxVel);
   }
   if (_sdf->HasElement("max_linear_velocity"))
   {
     const double maxLinVel = _sdf->Get<double>("max_linear_velocity");
     this->dataPtr->limiterLin->SetMaxVelocity(maxLinVel);
   }
   if (_sdf->HasElement("max_angular_velocity"))
   {
     const double maxAngVel = _sdf->Get<double>("max_angular_velocity");
     this->dataPtr->limiterAng->SetMaxVelocity(maxAngVel);
   }
 
   // Min Acceleration
   if (_sdf->HasElement("min_acceleration"))
   {
     const double minAccel = _sdf->Get<double>("min_acceleration");
     this->dataPtr->limiterLin->SetMinAcceleration(minAccel);
     this->dataPtr->limiterAng->SetMinAcceleration(minAccel);
   }
   if (_sdf->HasElement("min_linear_acceleration"))
   {
     const double minLinAccel = _sdf->Get<double>("min_linear_acceleration");
     this->dataPtr->limiterLin->SetMinAcceleration(minLinAccel);
   }
   if (_sdf->HasElement("min_angular_acceleration"))
   {
     const double minAngAccel = _sdf->Get<double>("min_angular_acceleration");
     this->dataPtr->limiterAng->SetMinAcceleration(minAngAccel);
   }
 
   // Max Acceleration
   if (_sdf->HasElement("max_acceleration"))
   {
     const double maxAccel = _sdf->Get<double>("max_acceleration");
     this->dataPtr->limiterLin->SetMaxAcceleration(maxAccel);
     this->dataPtr->limiterAng->SetMaxAcceleration(maxAccel);
   }
   if (_sdf->HasElement("max_linear_acceleration"))
   {
     const double maxLinAccel = _sdf->Get<double>("max_linear_acceleration");
     this->dataPtr->limiterLin->SetMaxAcceleration(maxLinAccel);
   }
   if (_sdf->HasElement("max_angular_acceleration"))
   {
     const double maxAngAccel = _sdf->Get<double>("max_angular_acceleration");
     this->dataPtr->limiterAng->SetMaxAcceleration(maxAngAccel);
   }
 
   // Min Jerk
   if (_sdf->HasElement("min_jerk"))
   {
     const double minJerk = _sdf->Get<double>("min_jerk");
     this->dataPtr->limiterLin->SetMinJerk(minJerk);
     this->dataPtr->limiterAng->SetMinJerk(minJerk);
   }
   if (_sdf->HasElement("min_linear_jerk"))
   {
     const double minLinJerk = _sdf->Get<double>("min_linear_jerk");
     this->dataPtr->limiterLin->SetMinJerk(minLinJerk);
   }
   if (_sdf->HasElement("min_angular_jerk"))
   {
     const double minAngJerk = _sdf->Get<double>("min_angular_jerk");
     this->dataPtr->limiterAng->SetMinJerk(minAngJerk);
   }
 
   // Max Jerk
   if (_sdf->HasElement("max_jerk"))
   {
     const double maxJerk = _sdf->Get<double>("max_jerk");
     this->dataPtr->limiterLin->SetMaxJerk(maxJerk);
     this->dataPtr->limiterAng->SetMaxJerk(maxJerk);
   }
   if (_sdf->HasElement("max_linear_jerk"))
   {
     const double maxLinJerk = _sdf->Get<double>("max_linear_jerk");
     this->dataPtr->limiterLin->SetMaxJerk(maxLinJerk);
   }
   if (_sdf->HasElement("max_angular_jerk"))
   {
     const double maxAngJerk = _sdf->Get<double>("max_angular_jerk");
     this->dataPtr->limiterAng->SetMaxJerk(maxAngJerk);
   }
 
   double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
   if (odomFreq > 0)
   {
     std::chrono::duration<double> odomPer{1 / odomFreq};
     this->dataPtr->odomPubPeriod =
       std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
   }
 

 
   // Subscribe to commands
   std::vector<std::string> topics;
   if (_sdf->HasElement("topic"))
   {
     topics.push_back(_sdf->Get<std::string>("topic"));
   }
   topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
   auto topic = validTopic(topics);
 
   this->dataPtr->node.Subscribe(topic, &GazeboRosSkidSteerDrivePrivate::OnCmdVel,
       this->dataPtr.get());
 
   // Subscribe to enable/disable
   std::vector<std::string> enableTopics;
   enableTopics.push_back(
     "/model/" + this->dataPtr->model.Name(_ecm) + "/enable");
   auto enableTopic = validTopic(enableTopics);
 
   if (!enableTopic.empty())
   {
     this->dataPtr->node.Subscribe(enableTopic, &GazeboRosSkidSteerDrivePrivate::OnEnable,
         this->dataPtr.get());
   }
   this->dataPtr->enabled = true;
 
   std::vector<std::string> odomTopics;
   if (_sdf->HasElement("odom_topic"))
   {
     odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
   }
   odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
       "/odometry");
   auto odomTopic = validTopic(odomTopics);
 
   this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
       odomTopic);
 
   std::string tfTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
     "/tf"};
   if (_sdf->HasElement("tf_topic"))
     tfTopic = _sdf->Get<std::string>("tf_topic");
   this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
       tfTopic);
 
   if (_sdf->HasElement("frame_id"))
     this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");
 
   if (_sdf->HasElement("child_frame_id"))
     this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");
 
   gzmsg << "GazeboRosSkidSteerDrive subscribing to twist messages on [" << topic << "]"
          << std::endl;
 }
 
 //////////////////////////////////////////////////
 void GazeboRosSkidSteerDrive::PreUpdate(const UpdateInfo &_info,
     EntityComponentManager &_ecm)
 {
   GZ_PROFILE("GazeboRosSkidSteerDrive::PreUpdate");
 
   // \TODO(anyone) Support rewind
   if (_info.dt < std::chrono::steady_clock::duration::zero())
   {
     gzwarn << "Detected jump back in time ["
         << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
         << "s]. System may not work properly." << std::endl;
   }
 
   // If the joints haven't been identified yet, look for them
   static std::set<std::string> warnedModels;
   auto modelName = this->dataPtr->model.Name(_ecm);
   if (this->dataPtr->leftJoints.empty() ||
       this->dataPtr->rightJoints.empty())
   {
     bool warned{false};
     for (const std::string &name : this->dataPtr->leftJointNames)
     {
       Entity joint = this->dataPtr->model.JointByName(_ecm, name);
       if (joint != kNullEntity)
         this->dataPtr->leftJoints.push_back(joint);
       else if (warnedModels.find(modelName) == warnedModels.end())
       {
         gzwarn << "Failed to find left joint [" << name << "] for model ["
                 << modelName << "]" << std::endl;
         warned = true;
       }
     }
 
     for (const std::string &name : this->dataPtr->rightJointNames)
     {
       Entity joint = this->dataPtr->model.JointByName(_ecm, name);
       if (joint != kNullEntity)
         this->dataPtr->rightJoints.push_back(joint);
       else if (warnedModels.find(modelName) == warnedModels.end())
       {
         gzwarn << "Failed to find right joint [" << name << "] for model ["
                 << modelName << "]" << std::endl;
         warned = true;
       }
     }
     if (warned)
     {
       warnedModels.insert(modelName);
     }
   }
 
   if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty())
     return;
 
   if (warnedModels.find(modelName) != warnedModels.end())
   {
     gzmsg << "Found joints for model [" << modelName
            << "], plugin will start working." << std::endl;
     warnedModels.erase(modelName);
   }
 
   // Nothing left to do if paused.
   if (_info.paused)
     return;
 
   for (Entity joint : this->dataPtr->leftJoints)
   {
     // skip this entity if it has been removed
     if (!_ecm.HasEntity(joint))
       continue;
 
     // Update wheel velocity
     auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
 
     if (vel == nullptr)
     {
       _ecm.CreateComponent(
           joint, components::JointVelocityCmd({this->dataPtr->leftFrontJointSpeed}));
       _ecm.CreateComponent(
            joint, components::JointVelocityCmd({this->dataPtr->leftRearJointSpeed}));

     }
     else
     {
       *vel = components::JointVelocityCmd({this->dataPtr->leftFrontJointSpeed});
       *vel = components::JointVelocityCmd({this->dataPtr->leftRearJointSpeed});
     }
   }
 
   for (Entity joint : this->dataPtr->rightJoints)
   {
     // skip this entity if it has been removed
     if (!_ecm.HasEntity(joint))
       continue;
 
     // Update wheel velocity
     auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
 
     if (vel == nullptr)
     {
       _ecm.CreateComponent(joint,
           components::JointVelocityCmd({this->dataPtr->rightFrontJointSpeed}));
       _ecm.CreateComponent(joint,
            components::JointVelocityCmd({this->dataPtr->rightRearJointSpeed}));
     }
     else
     {
       *vel = components::JointVelocityCmd({this->dataPtr->rightFrontJointSpeed});
       *vel = components::JointVelocityCmd({this->dataPtr->rightRearJointSpeed});

     }
   }
 
   // Create the left and right side joint position components if they
   // don't exist.
   auto leftPos = _ecm.Component<components::JointPosition>(
       this->dataPtr->leftJoints[0]);
   if (!leftPos && _ecm.HasEntity(this->dataPtr->leftJoints[0]))
   {
     _ecm.CreateComponent(this->dataPtr->leftJoints[0],
         components::JointPosition());
   }
 
   auto rightPos = _ecm.Component<components::JointPosition>(
       this->dataPtr->rightJoints[0]);
   if (!rightPos && _ecm.HasEntity(this->dataPtr->rightJoints[0]))
   {
     _ecm.CreateComponent(this->dataPtr->rightJoints[0],
         components::JointPosition());
   }
 }
 
 //////////////////////////////////////////////////
 void GazeboRosSkidSteerDrive::PostUpdate(const UpdateInfo &_info,
     const EntityComponentManager &_ecm)
 {
   GZ_PROFILE("GazeboRosSkidSteerDrive::PostUpdate");
   // Nothing left to do if paused.
   if (_info.paused)
     return;
 
   this->dataPtr->UpdateVelocity(_info, _ecm);
   this->dataPtr->UpdateOdometry(_info, _ecm);
 }
 
 //////////////////////////////////////////////////
 void GazeboRosSkidSteerDrivePrivate::UpdateOdometry(const UpdateInfo &_info,
     const EntityComponentManager &_ecm)
 {
   GZ_PROFILE("GazeboRosSkidSteerDrive::UpdateOdometry");
   // Initialize, if not already initialized.
 
   if (this->leftJoints.empty() || this->rightJoints.empty())
     return;
 
   
   // Get the first joint positions for the left and right side.
   auto leftPosFront = _ecm.Component<components::JointPosition>(this->leftJoints[0]);
   auto rightPosFront = _ecm.Component<components::JointPosition>(this->rightJoints[0]);
   auto leftPosRear = _ecm.Component<components::JointPosition>(this->leftJoints[1]);
   auto rightPosRear = _ecm.Component<components::JointPosition>(this->rightJoints[1]);
    // std::cout << leftPos << " " << rightPos << "   ";
   // Abort if the joints were not found or just created.
   if (!leftPosFront || !rightPosFront || leftPosFront->Data().empty() ||
       rightPosFront->Data().empty())
   {
     return;
   }

   auto leftAverage = (leftPosFront->Data()[0] + leftPosRear->Data()[0]) / 2.0;
   auto rightAverage = (rightPosFront->Data()[0] + rightPosRear->Data()[0]) / 2.0;

   double leftDist = (leftAverage - this->odomOldLeft) * this->wheelRadius;
   double rightDist = (rightAverage - this->odomOldRight) * this->wheelRadius;
   
   // Compute the linear displacement of the robot
   double dist = (leftDist + rightDist) * 0.5;
   
   // Compute angular displacement (yaw change)
   double deltaAngle = (rightDist - leftDist) / this->wheelSeparation;
   
   // Update the robot's yaw orientation
   this->odomYaw += deltaAngle;
   this->odomYaw = math::Angle(this->odomYaw).Normalized().Radian();
   
   // Update the robot's X and Y position
   this->odomX += dist * cos(this->odomYaw);
   this->odomY += dist * sin(this->odomYaw);
   
   // Compute time difference
   auto odomTimeDiff = _info.simTime - this->lastOdomTime;
   double tdiff = std::chrono::duration<double>(odomTimeDiff).count();
   
   // Compute linear and angular velocity
   double odomLinearVelocity = dist / tdiff;
   double odomAngularVelocity = deltaAngle / tdiff;
   
   // Store current wheel positions for the next update
   this->lastOdomTime = _info.simTime;
   this->odomOldLeft = leftAverage;
   this->odomOldRight = rightAverage;

   // Throttle odometry publishing
   auto diff = _info.simTime - this->lastOdomPubTime;
   if (diff > std::chrono::steady_clock::duration::zero() &&
       diff < this->odomPubPeriod)
   {
     return;
   }
   this->lastOdomPubTime = _info.simTime;
 
 
   // Construct the odometry message and publish it.
   msgs::Odometry msg;
   msg.mutable_pose()->mutable_position()->set_x(this->odomX);
   msg.mutable_pose()->mutable_position()->set_y(this->odomY);
 
   math::Quaterniond orientation(0, 0, this->odomYaw);
   msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);
 
   msg.mutable_twist()->mutable_linear()->set_x(odomLinearVelocity);
   msg.mutable_twist()->mutable_angular()->set_z(odomAngularVelocity);
 
   // Set the time stamp in the header
   msg.mutable_header()->mutable_stamp()->CopyFrom(
       convert<msgs::Time>(_info.simTime));
 
   // Set the frame id.
   auto frame = msg.mutable_header()->add_data();
   frame->set_key("frame_id");
   if (this->sdfFrameId.empty())
   {
     frame->add_value(this->model.Name(_ecm) + "/odom");
   }
   else
   {
     frame->add_value(this->sdfFrameId);
   }
 
   std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
   if (this->sdfChildFrameId.empty())
   {
     if (linkName)
     {
       auto childFrame = msg.mutable_header()->add_data();
       childFrame->set_key("child_frame_id");
       childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
     }
   }
   else
   {
     auto childFrame = msg.mutable_header()->add_data();
     childFrame->set_key("child_frame_id");
     childFrame->add_value(this->sdfChildFrameId);
   }
 
   // Construct the Pose_V/tf message and publish it.
   msgs::Pose_V tfMsg;
   msgs::Pose *tfMsgPose = tfMsg.add_pose();
   tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
   tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
   tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());
 
   // Publish the messages
   this->odomPub.Publish(msg);
   this->tfPub.Publish(tfMsg);
 }
 
 //////////////////////////////////////////////////
 void GazeboRosSkidSteerDrivePrivate::UpdateVelocity(const UpdateInfo &_info,
     const EntityComponentManager &/*_ecm*/)
 {
   GZ_PROFILE("GazeboRosSkidSteerDrive::UpdateVelocity");
 
   double linVel;
   double angVel;
   {
     std::lock_guard<std::mutex> lock(this->mutex);
     linVel = this->targetVel.linear().x();
     angVel = this->targetVel.angular().z();
   }
 
  //  // Limit the target velocity if needed.
  //  this->limiterLin->Limit(
  //      linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
  //  this->limiterAng->Limit(
  //      angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);
 
   // Update history of commands.
   this->last1Cmd = last0Cmd;
   this->last0Cmd.lin = linVel;
   this->last0Cmd.ang = angVel;
   double leftSpeed = (this->last0Cmd.lin - 
    this->last0Cmd.ang * this->wheelSeparation / 2.0) / this->wheelRadius;
   double rightSpeed = (this->last0Cmd.lin + 
     this->last0Cmd.ang * this->wheelSeparation / 2.0) / this->wheelRadius;

    this->leftFrontJointSpeed = leftSpeed;
    this->leftRearJointSpeed = leftSpeed;
    this->rightFrontJointSpeed = rightSpeed;
    this->rightRearJointSpeed = rightSpeed;
 }
 
 //////////////////////////////////////////////////
 void GazeboRosSkidSteerDrivePrivate::OnCmdVel(const msgs::Twist &_msg)
 {
   std::lock_guard<std::mutex> lock(this->mutex);
   if (this->enabled)
   {
     this->targetVel = _msg;
   }
 }
 
 //////////////////////////////////////////////////
 void GazeboRosSkidSteerDrivePrivate::OnEnable(const msgs::Boolean &_msg)
 {
   std::lock_guard<std::mutex> lock(this->mutex);
   this->enabled = _msg.data();
   if (!this->enabled)
   {
     math::Vector3d zeroVector{0, 0, 0};
     msgs::Set(this->targetVel.mutable_linear(), zeroVector);
     msgs::Set(this->targetVel.mutable_angular(), zeroVector);
   }
 }
 
 GZ_ADD_PLUGIN(GazeboRosSkidSteerDrive,
                     System,
                     GazeboRosSkidSteerDrive::ISystemConfigure,
                     GazeboRosSkidSteerDrive::ISystemPreUpdate,
                     GazeboRosSkidSteerDrive::ISystemPostUpdate)
 
 GZ_ADD_PLUGIN_ALIAS(GazeboRosSkidSteerDrive, "gz::sim::systems::GazeboRosSkidSteerDrive")
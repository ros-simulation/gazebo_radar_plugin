/*
 * Copyright 2016-2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo_radar_plugin/ModeCRadarPlugin.hh"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include "gazebo/sensors/Noise.hh"
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <algorithm>
#include <sstream>
#include <string>

using namespace gazebo;

static const uint16_t DEFAULT_TRANSPONDER_CODE = 1200;

GZ_REGISTER_MODEL_PLUGIN(ModeCRadarPlugin);


std::string TrimNamespace(const std::string &modelName)
{
  // Trim namespaces
  size_t index = modelName.find_last_of('|');
  return modelName.substr(index + 1);
}

/// \brief Determine the type of a gazebo model from its name
std::string DetermineModelType(const std::string &modelName)
{
  std::string modelType(TrimNamespace(modelName));

  // Trim trailing underscore and number caused by inserting multiple of the same model
  size_t index = modelType.find_last_not_of("0123456789");
  if (modelType[index] == '_' && index > 1)
  {
    modelType = modelType.substr(0, index);
  }

  // Trim "_clone" suffix if exists
  index = modelType.rfind("_clone");
  if (index != std::string::npos)
  {
    modelType.erase(index);
  }

  return modelType;
}

uint16_t GetTransponderCode(const std::string & modelName)
{
  std::string reduced_str;
  for ( int i = 0 ; i < modelName.length(); i++)
  {
    if (isdigit(modelName[i]))
    {
      reduced_str.push_back(modelName[i]);
    }
  }
  if (reduced_str.length() > 0)
  {
    return stoi(reduced_str);
  }
  else
  {
    return DEFAULT_TRANSPONDER_CODE;
  }
}

/////////////////////////////////////////////////
ModeCRadarPlugin::ModeCRadarPlugin()
{
}

/////////////////////////////////////////////////
ModeCRadarPlugin::~ModeCRadarPlugin()
{
  this->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ModeCRadarPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robotNamespace = "logical_camera";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement(
        "robotNamespace")->Get<std::string>() + "/";
  }

  this->world = _parent->GetWorld();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->onlyPublishKnownModels = false;
  if (_sdf->HasElement("known_model_types"))
  {
    ROS_DEBUG("Only publishing known model types");
    this->onlyPublishKnownModels = true;
    this->knownModelTypes.clear();
    sdf::ElementPtr knownModelTypesElem = _sdf->GetElement("known_model_types");
    if (!knownModelTypesElem->HasElement("type"))
    {
      gzerr << "Unable to find <type> elements in the <known_model_types> section\n";
      return;
    }
    sdf::ElementPtr knownModelTypeElem = knownModelTypesElem->GetElement("type");
    while (knownModelTypeElem)
    {
      // Parse the model type, which is encoded in model names.
      std::string type = knownModelTypeElem->Get<std::string>();

      ROS_DEBUG_STREAM("New known model type: " << type);
      this->knownModelTypes.push_back(type);
      knownModelTypeElem = knownModelTypeElem->GetNextElement("type");
    }
  }
  else
  {
    ROS_DEBUG("Publishing all model types");
  }

  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  this->FindLogicalCamera();
  if (!this->sensor)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }

  // Handle noise model settings.
  if (_sdf->HasElement("position_noise"))
  {
    this->noiseModels["POSITION_NOISE"] =
      sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("position_noise")->GetElement("noise"),
      "logical_camera");
  }

  this->name = _parent->GetName();
  std::string radarTopic_ros = this->name + "_radar";
  if (_sdf->HasElement("radar_topic_ros")) {
    radarTopic_ros = _sdf->Get<std::string>("radar_topic_ros");
  }

  this->radar_sensor_frameid = this->name + "_radar";
  if (_sdf->HasElement("radar_sensor_frameid"))
  {
    _sdf->GetElement("radar_sensor_frameid")->GetValue()->Get(this->radar_sensor_frameid);
  }

  this->max_range = 10000;
  if (_sdf->HasElement("max_range"))
  {
    _sdf->GetElement("max_range")->GetValue()->Get(this->max_range);
  }
  
  this->hfov = 2 * M_PI;
  if (_sdf->HasElement("hfov"))
  {
    _sdf->GetElement("hfov")->GetValue()->Get(this->hfov);
  }

  this->vfov = M_PI;
  if (_sdf->HasElement("vfov"))
  {
    _sdf->GetElement("vfov")->GetValue()->Get(this->vfov);
  }

  this->imageSub = this->node->Subscribe(this->sensor->Topic(),
          &ModeCRadarPlugin::OnImage, this);
  gzdbg << "Subscribing to gazebo topic: " << this->sensor->Topic() << "\n";

  this->radarPub = this->rosnode->advertise<gazebo_radar_plugin::ModeCRadarSummary>(radarTopic_ros, 1, true);
  gzdbg << "Publishing to ROS topic: " << radarPub.getTopic() << "\n";
}

void ModeCRadarPlugin::FindLogicalCamera()
{
  sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();

  // Go through each link's sensors until a logical camera is found
  for (physics::LinkPtr link : this->model->GetLinks())
  {
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      sensors::SensorPtr sensor = sensorManager->GetSensor(link->GetSensorName(i));
      if (sensor->Type() == "logical_camera")
      {
        this->sensor = sensor;
        break;
      }
    }
    if (this->sensor)
    {
      this->cameraLink = link;
      break;
    }
  }
}

/////////////////////////////////////////////////
void ModeCRadarPlugin::OnImage(ConstLogicalCameraImagePtr &_msg)
{
  gazebo_radar_plugin::ModeCRadarSummary radar_msg;

  //TODO use gazebo time directly instead of ROS time?
  radar_msg.header.stamp = ros::Time::now();
  radar_msg.header.frame_id = this->radar_sensor_frameid;

  math::Vector3 cameraPosition = math::Vector3(msgs::ConvertIgn(_msg->pose().position()));
  math::Quaternion cameraOrientation = math::Quaternion(
    msgs::ConvertIgn(_msg->pose().orientation()));
  math::Pose cameraPose = math::Pose(cameraPosition, cameraOrientation);

  std::ostringstream logStream;
  math::Pose modelPose;
  for (int i = 0; i < _msg->model_size(); ++i)
  {
    std::string modelName = _msg->model(i).name();
    std::string modelType = DetermineModelType(modelName);
    auto modelPtr = this->world->GetModel(modelName);

    if (!this->ModelTypeToPublish(modelType))
    {
      logStream << "Not publishing model: " << modelName << " of type: " << modelType << std::endl;
    }
    else
    {
      logStream << "Publishing model: " << modelName << " of type: " << modelType << std::endl;
      math::Vector3 modelPosition = math::Vector3(
        msgs::ConvertIgn(_msg->model(i).pose().position()));
      math::Quaternion modelOrientation = math::Quaternion(
        msgs::ConvertIgn(_msg->model(i).pose().orientation()));
      modelPose = math::Pose(modelPosition, modelOrientation);
      uint16_t t_code = GetTransponderCode(modelName);
      this->AddNoise(modelPose);
      this->AppendRadarContact(radar_msg, cameraPose, modelPose, t_code, radar_msg.header);
    }

    // Check any children models
    auto nestedModels = modelPtr->NestedModels();
    for (auto nestedModel : nestedModels)
    {
      modelName = nestedModel->GetName();
      modelType = DetermineModelType(modelName);
      if (!this->ModelTypeToPublish(modelType))
      {
        logStream << "Not publishing model: " << modelName << " of type: " << modelType << std::endl;
        continue;
      }
      logStream << "Publishing model: " << modelName << " of type: " << modelType  << std::endl;
      // Convert the world pose of the model into the camera frame
      modelPose = (nestedModel->GetWorldPose()) - cameraPose;
      uint16_t t_code = GetTransponderCode(modelName);
      this->AddNoise(modelPose);
      this->AppendRadarContact(radar_msg, cameraPose, modelPose, t_code, radar_msg.header);
    }
  }

  if (!logStream.str().empty())
  {
    ROS_DEBUG_THROTTLE(1, "%s", logStream.str().c_str());
  }
  this->radarPub.publish(radar_msg);
}

bool ModeCRadarPlugin::ModelTypeToPublish(const std::string & modelType)
{
  bool publishModelType = true;

  // Check if there are restrictions on which models to publish
  if (this->onlyPublishKnownModels)
  {
    // Only publish the model if its type is one of the known types
    auto it = std::find(this->knownModelTypes.begin(), this->knownModelTypes.end(), modelType);
    bool knownModel = it != this->knownModelTypes.end();
    publishModelType = knownModel;
  }
  return publishModelType;
}

void ModeCRadarPlugin::AddNoise(math::Pose & pose)
{
  if (this->noiseModels.find("POSITION_NOISE") != this->noiseModels.end())
  {
    // Apply additive noise to the model position
    pose.pos.x =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.pos.x);
    pose.pos.y =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.pos.y);
    pose.pos.z =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.pos.z);
  }
}

void ModeCRadarPlugin::AppendRadarContact(
  gazebo_radar_plugin::ModeCRadarSummary & radar_msg,
  const math::Pose & cameraPose, const math::Pose & modelPose,
  const uint16_t code, const std_msgs::Header & header)
{
  gazebo_radar_plugin::ModeCRadar contact;
  contact.header = header;

  math::Pose temp = modelPose; //GetYaw isn't const??
  contact.range = temp.pos.GetLength();
  contact.bearing = atan2(-1 * modelPose.pos.y, modelPose.pos.x);
  double azimouth = asin(modelPose.pos.z / contact.range);
  // ModeC Radar reports altitude at destination, not relative
  contact.altitude = cameraPose.pos.z + modelPose.pos.z;
  contact.code = code;
  contact.ident = false;
  
  if (fabs(contact.bearing) * 2 < this->hfov &&
      fabs(azimouth) * 2 < this->vfov &&
      contact.range < this->max_range)
  {
    radar_msg.contacts.push_back(contact);
  }
}

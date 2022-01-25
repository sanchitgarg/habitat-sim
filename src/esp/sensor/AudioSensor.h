// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_AUDIOSENSOR_H_
#define ESP_SENSOR_AUDIOSENSOR_H_

#include "esp/core/esp.h"
#include "esp/sensor/Sensor.h"
#include "esp/assets/MeshData.h"
#include "HabitatAcoustics.h"

namespace esp {
namespace sensor {

struct AudioSensorSpec : public SensorSpec {
  // audio file, geometry etc
  AudioSensorSpec();
  void sanityCheck() const override;
  bool isVisualSensorSpec() const override { return false; }
  ESP_SMART_POINTERS(AudioSensorSpec)
};

class AudioSensor : public Sensor {
 public:
  // Constructor
  explicit AudioSensor(scene::SceneNode& node, AudioSensorSpec::ptr spec);

  /**
   * @brief Return that this is not a visual sensor
   */
  bool isVisualSensor() const override { return false; }

  void setAudioSourceTransform(vec3f sourcePos, vec4f sourceRotQuat);
  void setAgentTransform(vec3f agentPos, vec4f agentRotQuat);
  void setAudioSimulationConfigs(const HabitatAcoustics::Configuration& config);
  void setChannelLayout(const HabitatAcoustics::ChannelLayout& channelLayout);
  void setOutputFolder(const char* folderPath);

  // todo sangarg : implement this
  bool getObservation(sim::Simulator& sim, Observation& obs) override;
  bool getObservationSpace(ObservationSpace& space) override;
  bool drawObservation(sim::Simulator& sim);
  bool readObservation(Observation& obs);
  bool displayObservation(sim::Simulator& sim) override { return false; };

  std::size_t getChannelCount();
  std::size_t getSampleCount();
  float getImpulseResponse(const std::size_t channelIndex, const std::size_t sampleIndex);

  void runSimulation(sim::Simulator& sim);
  void reset();

protected:
  AudioSensorSpec::ptr audioSensorSpec_ =
      std::dynamic_pointer_cast<AudioSensorSpec>(spec_);
  std::unique_ptr<HabitatAcoustics::Simulator> audioSimulator;
  esp::assets::MeshData::ptr sceneMesh;
  std::vector<std::uint16_t> objectIds;

private:
  void createAudioSimulator();
  void loadSemanticMesh(sim::Simulator& sim);
  void loadMesh(sim::Simulator& sim);
  std::string getSimulationFolder();

  int currentSimCount = -1;
  vec3f lastSourcePos;
  vec4f lastSourceRot;
  vec3f lastAgentPos;
  vec4f lastAgentRot;
  HabitatAcoustics::Configuration acousticsConfig;
  HabitatAcoustics::ChannelLayout channelLayout;
  bool configsSet = false;
  bool newInitialization = false;
  bool newSource = false;
  std::string outputFolderPath;

public:
  ESP_SMART_POINTERS(AudioSensor)
};

} // namespace sensor
} // namespace esp

#endif // ESP_SENSOR_AUDIOSENSOR_H_

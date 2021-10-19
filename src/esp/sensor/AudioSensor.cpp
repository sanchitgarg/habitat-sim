// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <fstream>

#include "AudioSensor.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace sensor {

AudioSensorSpec::AudioSensorSpec() : SensorSpec() {
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ AudioSensorSpec constructor";
  uuid = "audio";
  sensorType = SensorType::Audio;
  sensorSubType = SensorSubType::ImpulseResponse;
}

void AudioSensorSpec::sanityCheck() const {
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ sanityCheck";
  SensorSpec::sanityCheck();
  CORRADE_ASSERT(
    sensorType == SensorType::Audio,
    "AudioSensorSpec::sanityCheck(): sensorType must be Audio", );
}

AudioSensor::AudioSensor(scene::SceneNode& node, AudioSensorSpec::ptr spec)
    : Sensor{node, std::move(spec)} {
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ AudioSensor constructor";
  audioSensorSpec_->sanityCheck();
}

void AudioSensor::createAudioSimulator() {

  ESP_DEBUG() << "LOOOOOGGGGG ------------------ Create audio simulator";

  if (!configsSet)
  {
    ESP_DEBUG() << "ERROR LOOOOOGGGGG ------------------ setConfig not called, will use default configs";
    return;
  }

  audioSimulator = std::make_unique<HabitatAcoustics::Simulator>();

  // Configure audio simulator
  ESP_DEBUG() << acousticsConfig;

  audioSimulator->Configure(acousticsConfig);
  ++currentSimCount;
}

void AudioSensor::setAudioSourceLocation(vec3f sourcePos) {
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ setAudioSourceLocation : " << sourcePos;
  lastSourcePos = sourcePos;
}

void AudioSensor::setAgentLocation(vec3f agentPos) {
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ setAgentLocation : " << agentPos;
  createAudioSimulator();
  HabitatAcoustics::ChannelLayout channelLayout;
  channelLayout.channelCount = 2;
  channelLayout.channelType = 3;

  audioSimulator->AddListener(HabitatAcoustics::Vector3f{agentPos(0), agentPos(1), agentPos(2)}, channelLayout);
}

void AudioSensor::setAudioSimulationConfigs(const HabitatAcoustics::Configuration& config) {
  acousticsConfig = config;
  configsSet = true;
}

bool AudioSensor::getObservationSpace(ObservationSpace& space) {
  if (!audioSimulator) {
    ESP_ERROR() << "getObservationSpace : AUDIO SIMULATOR NOT CREATED ";
    return false;
  }

  ESP_DEBUG() << "LOOOOOGGGGG ------------------ getObservationSpace";
  space.spaceType = ObservationSpaceType::None; // TODO sangarg : check what this is
  space.shape = {audioSimulator->GetChannelCount(),
                 audioSimulator->GetSampleCount()};
  space.dataType = core::DataType::DT_FLOAT;

  ESP_DEBUG() << "LOOOOOGGGGG ------------------ getObservationSpace -> " << space.shape[0] << " " << space.shape[1];
  return true;
}

bool AudioSensor::drawObservation(sim::Simulator& sim) {
  if (!audioSimulator) {
    ESP_ERROR() << "drawObservation : AUDIO SIMULATOR NOT CREATED ";
    return false;
  }

  ESP_DEBUG() << "LOOOOOGGGGG ------------------ drawObservation";

  sceneMesh = sim.getJoinedMesh(false);

  HabitatAcoustics::VertexData vertices;

  vertices.vertices = sceneMesh->vbo.data();
  vertices.byteOffset = 0;
  vertices.vertexCount = sceneMesh->vbo.size();
  vertices.vertexStride = 0;

  HabitatAcoustics::IndexData indices;

  indices.indices = sceneMesh->ibo.data();
  indices.byteOffset = 0; // todo sangarg : Most likely this is correct since it is a vector<uint32_t>
  indices.indexCount = sceneMesh->ibo.size();
  ESP_DEBUG() << "Vertex count : " << vertices.vertexCount << ", Index count : " << indices.indexCount;
  audioSimulator->LoadMeshData(vertices, indices);

  ESP_DEBUG() << "Adding source at position : " << lastSourcePos;
  audioSimulator->AddSource(HabitatAcoustics::Vector3f{lastSourcePos(0), lastSourcePos(1), lastSourcePos(2)});

  audioSimulator->RunSimulation(getSimulationFolder());
  return true;
}

std::string AudioSensor::getSimulationFolder() {
  if (outputFolderPath.empty())
    return std::string("/home/sangarg/AudioSimulation" + std::to_string(currentSimCount));

  return outputFolderPath + std::to_string(currentSimCount);
}

void AudioSensor::setOutputFolder(const char* folderPath) {
  outputFolderPath = std::string(folderPath);
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ setOutputFolder: " << outputFolderPath;
}

bool AudioSensor::readObservation(Observation& obs) {
  if (!audioSimulator) {
    ESP_ERROR() << "drawObservation : AUDIO SIMULATOR NOT CREATED ";
    return false;
  }

ESP_DEBUG() << "LOOOOOGGGGG ------------------ readObservation";
  // TODO figure out when this buffer needs to be cleaned up
  // This would depend on if the audio source/listener is moving or if we
  // want a static source
  ObservationSpace space;
  getObservationSpace(space);
  //if (buffer_ == nullptr) {
    buffer_ = core::Buffer::create(space.shape, space.dataType);
  //}
  obs.buffer = buffer_;
  std::size_t offset = 0;

  // TODO sangarg
  // for (std::size_t channelIndex = 0; channelIndex < space.shape[0]; ++channelIndex) {
  //   for (std::size_t sampleIndex = 0; sampleIndex < space.shape[1]; ++sampleIndex) {
  //     // todo sangarg : Dumping out the output to console for now
  //     std::memcpy(obs.buffer + offset, )
  //     ESP_DEBUG() << sampleIndex << "\t" << audioSimulator->GetImpulseResponse(channelIndex, sampleIndex);

  //     // Copy data into buffer_.data and keep jumping by sizeof(float)
  //   }
  // }

  std::size_t channelCount = audioSimulator->GetChannelCount();
  std::size_t sampleCount = audioSimulator->GetSampleCount();
  ESP_DEBUG() << "ChannelCount : " <<  channelCount;
  ESP_DEBUG() << "SampleCount : " << sampleCount;

  for (std::size_t channelIndex = 0; channelIndex < channelCount; ++channelIndex)
  {
    std::ofstream file;
    std::string fileName = getSimulationFolder() + "/ir" + std::to_string(channelIndex) + ".txt";

    file.open(fileName);

    for (std::size_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
      file << sampleIndex << "\t" << audioSimulator->GetImpulseResponse(channelIndex, sampleIndex) << std::endl;
    }

    ESP_DEBUG() << "File written : " << fileName;
    file.close();
  }

  audioSimulator = nullptr;
  return true;
}

void AudioSensor::runSimulation(sim::Simulator& sim)
{
  if (!audioSimulator) {
    ESP_ERROR() << "runSimulation : AUDIO SIMULATOR NOT CREATED ";
    return;
  }

  ESP_DEBUG() << "LOOOOOGGGGG ------------------ runSimulation";

  drawObservation(sim);
}

bool AudioSensor::getObservation(sim::Simulator& sim, Observation& obs) {
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ getObservation";
  drawObservation(sim);
  readObservation(obs);
  return true;
}

std::size_t AudioSensor::getChannelCount()
{
  if (!audioSimulator) {
    ESP_ERROR() << "getChannelCount : AUDIO SIMULATOR NOT CREATED ";
    return false;
  }

  ESP_DEBUG() << "LOOOOOGGGGG ------------------ getChannelCount" << audioSimulator->GetChannelCount();
  return audioSimulator->GetChannelCount();
}

std::size_t AudioSensor::getSampleCount()
{
  if (!audioSimulator) {
    ESP_ERROR() << "getSampleCount : AUDIO SIMULATOR NOT CREATED ";
    return false;
  }

  ESP_DEBUG() << "LOOOOOGGGGG ------------------ getSampleCount" << audioSimulator->GetSampleCount();
  return audioSimulator->GetSampleCount();
}

float AudioSensor::getImpulseResponse(const std::size_t channelIndex, const std::size_t sampleIndex)
{
  return audioSimulator->GetImpulseResponse(channelIndex, sampleIndex);
}

}  // namespace sensor
}  // namespace esp

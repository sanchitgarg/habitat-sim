// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <fstream>
#include <unordered_set>
#include <unordered_map>
#include <sstream>

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

  ++currentSimCount;
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ Create audio simulator";

  if (!configsSet)
  {
    ESP_DEBUG() << "ERROR LOOOOOGGGGG ------------------ setConfig not called, will use default configs";
    return;
  }

  // If the audio simulator already exists, do not create a new one
  if (audioSimulator)
    return;

  newInitialization = true;
  audioSimulator = std::make_unique<HabitatAcoustics::Simulator>();
  lastAgentPos = {__FLT_MIN__, __FLT_MIN__, __FLT_MIN__};

  // Configure audio simulator
  ESP_DEBUG() << acousticsConfig;

  audioSimulator->Configure(acousticsConfig);
}

void AudioSensor::setAudioSourceTransform(vec3f sourcePos, vec4f sourceRotQuat) {
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ setAudioSourceTransform : pos[" << sourcePos << "]";//, rotQuat[" << sourceRotQuat << "]";
  lastSourcePos = sourcePos;
  lastSourceRot = sourceRotQuat;
  newSource = true;
}

void AudioSensor::setAgentTransform(vec3f agentPos, vec4f agentRotQuat) {
  ESP_DEBUG() << "LOOOOOGGGGG ------------------ setAgentTransform : pos[" << agentPos << "], rotQuat[" << agentRotQuat << "]";

  // Create the audio simulator object if it does not already exist
  createAudioSimulator();

  // If the agent position has changed, add a listener
  if (newInitialization || (lastAgentPos != agentPos) || !(lastAgentRot.isApprox(agentRotQuat)))
  {
    audioSimulator->AddListener(
      HabitatAcoustics::Vector3f{agentPos(0), agentPos(1), agentPos(2)},
      HabitatAcoustics::Quaternion{agentRotQuat(0), agentRotQuat(1), agentRotQuat(2), agentRotQuat(3)},
      channelLayout);
  }
}

void AudioSensor::setAudioSimulationConfigs(const HabitatAcoustics::Configuration& config) {
  acousticsConfig = config;
  configsSet = true;
}

void AudioSensor::setChannelLayout(const HabitatAcoustics::ChannelLayout& channelLayout)
{
  this->channelLayout = channelLayout;
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

  if (newInitialization)
  {
    newInitialization = false;
    ESP_DEBUG() << "New initialization, will upload geometry and add the source at position : " << lastSourcePos;

    if (acousticsConfig.enableMaterials && sim.semanticSceneExists())
    {
      ESP_DEBUG() << "LOOOOGGGGGGGG ----------------- Loading semantic scene";
      loadSemanticMesh(sim);
    }
    else
    {
      // load the normal render mesh without any material info
      ESP_DEBUG() << "LOOOOGGGGGGGG ----------------- Semantic scene does not exist or materials are disabled, will use default material";
      loadMesh(sim);
    }
  }

  if (newSource)
  {
    // If its a new source, add it.
    // A new initialization should always come with a new source
    // Mark that the source as added
    newSource = false;
    ESP_DEBUG() << "Adding source at position : " << lastSourcePos;
    audioSimulator->AddSource(
      HabitatAcoustics::Vector3f{lastSourcePos(0), lastSourcePos(1), lastSourcePos(2)},
      HabitatAcoustics::Quaternion{lastSourceRot(0), lastSourceRot(1), lastSourceRot(2), lastSourceRot(3)});
  }

  const std::string simFolder = getSimulationFolder();
  ESP_DEBUG() << "Running simulation, folder : " << simFolder;
  audioSimulator->RunSimulation(simFolder);
  return true;
}

void AudioSensor::loadSemanticMesh(sim::Simulator& sim) {

  objectIds.clear();

  sceneMesh = sim.getJoinedSemanticMesh(objectIds);

  std::shared_ptr<scene::SemanticScene> semanticScene = sim.getSemanticScene();

  const std::vector<std::shared_ptr<scene::SemanticCategory>>& categories = semanticScene->categories();
  const std::vector<std::shared_ptr<scene::SemanticObject>>& objects = semanticScene->objects();

  // Debug related stuff
  {
    ESP_DEBUG() << "LOG --------------- Category size : " << categories.size();
    ESP_DEBUG() << "LOG --------------- Objects size : " << objects.size();
    ESP_DEBUG() << "LOG --------------- objectIds size : " << objectIds.size();
    ESP_DEBUG() << "LOG --------------- sceneMesh vbo size : " << sceneMesh->vbo.size();
    ESP_DEBUG() << "LOG --------------- sceneMesh ibo size : " << sceneMesh->ibo.size();
    ESP_DEBUG() << "LOG --------------- sceneMesh cbo size : " << sceneMesh->cbo.size();

    std::unordered_map<int, std::string> uniqCategories;
    std::unordered_map<std::string, int> catToCatId;

    // Get uniq categories for debugging
    for (const auto category: categories) {
      auto itr = uniqCategories.find(category->index());
      if (itr != uniqCategories.end())
      {
        if (itr->second != category->name())
          ESP_DEBUG() << "ERRROOOORRRR --------------- Incorrect category mapping, incoming mapping <"
            << category->index() << ", " << category->name() << ">"
            << "; existing <" << itr->first << ", " << itr->second << ">";
      }
      else {
        uniqCategories.insert({category->index(), category->name()});
        catToCatId.insert({ category->name(), category->index()});
        ESP_DEBUG() << "LOG --------------- Category names : " <<  category->index() << ", " << category->name();
      }
    }

    std::unordered_set<uint16_t> uniqObjIds;
    std::unordered_map<std::string, std::unordered_set<uint16_t>> categoryNameToObjecIds;
    for (auto objIds : objectIds)
    {
      if (uniqObjIds.insert(objIds).second)
      {
        categoryNameToObjecIds[objects[objIds]->category()->name()].insert(objIds);
      }
    }
    ESP_DEBUG() << "LOG --------------- Uniq ObjectId size : " << uniqObjIds.size();

    for (auto catToObjId: categoryNameToObjecIds)
    {
      std::stringstream ss;
      ss << "LOG -------------------- categoryName to ids : " << catToObjId.first << " -> ";
      for (auto i : catToObjId.second)
      {
        ss << i << ", ";
      }
      ESP_DEBUG() << ss.str();

      if (catToCatId.find(catToObjId.first) == catToCatId.end())
      {
        ESP_DEBUG() << "ERRRROOOORRRRRR ------------------- found cat name that was not seen earlier : " << catToObjId.first;
      }
    }
  } // Debug related stuff

  HabitatAcoustics::VertexData vertices;

  vertices.vertices = sceneMesh->vbo.data();
  vertices.byteOffset = 0;
  vertices.vertexCount = sceneMesh->vbo.size();
  vertices.vertexStride = 0;

  audioSimulator->LoadMeshVertices(vertices);

  std::unordered_map<std::string, std::vector<uint32_t>> categoryNameToIndices;

  auto& ibo = sceneMesh->ibo;
  for (std::size_t iboIdx = 0; iboIdx < ibo.size(); iboIdx += 3)
  {
    // For each index in the ibo
    //  get the object id
    //    get the object using the id
    //      get the category name from the object
    std::string cat1 = objects[objectIds[ibo[iboIdx]]]->category()->name();
    std::string cat2 = objects[objectIds[ibo[iboIdx + 1]]]->category()->name();
    std::string cat3 = objects[objectIds[ibo[iboIdx + 2]]]->category()->name();
    std::string catToUse;

    if (cat1 == cat2 && cat1 == cat3)
    {
      // If all 3 categories are the same, save the indices to cat1 in the categoryNameToIndices map
      catToUse = cat1;
    }
    else if (cat1 != cat2 && cat1 != cat3)
    {
      // If cat1 != 2 and 3
      // then either all 3 are different, or cat1 is different while 2==3
      // Either case, use 1
      // reason : if all 3 are different, we cant determine which one is correct
      // if this is the odd one out, then the triangle is actually of this category
      catToUse = cat1;
    }
    else if (cat1 == cat2)
    {
      // if we reach here, cat 1 == 2 but != 3
      // use 3
      catToUse = cat3;
    }
    else
    {
      // else 1 == 3 but != 2
      // use 2
      catToUse = cat2;
    }

    categoryNameToIndices[catToUse].push_back(ibo[iboIdx]);
    categoryNameToIndices[catToUse].push_back(ibo[iboIdx + 1]);
    categoryNameToIndices[catToUse].push_back(ibo[iboIdx + 2]);
  }

  std::size_t indicesLoaded = 0;
  std::size_t totalIndicesLoaded = 0;

  for (auto catToIndices : categoryNameToIndices)
  {
    HabitatAcoustics::IndexData indices;

    indices.indices = catToIndices.second.data();
    indices.byteOffset = 0;
    indices.indexCount = catToIndices.second.size();

    ++indicesLoaded;
    const bool lastUpdate = (indicesLoaded == categoryNameToIndices.size());

    ESP_DEBUG()
      << "Vertex count : " << vertices.vertexCount
      << ", Index count : " << indices.indexCount
      << ", Material : " << catToIndices.first
      << ", LastUpdate : " << lastUpdate;

    audioSimulator->LoadMeshIndices(
      indices,
      catToIndices.first,
      lastUpdate);

    totalIndicesLoaded += indices.indexCount;
  }

  if (totalIndicesLoaded != sceneMesh->ibo.size())
  {
    ESP_DEBUG() << "EERRRRRRROOOORRRRRR --------- totalIndicesLoaded != sceneMesh->ibo.size() : (" << totalIndicesLoaded << " != " << sceneMesh->ibo.size() << ")";
  }
}

void AudioSensor::loadMesh(sim::Simulator& sim) {
  sceneMesh = sim.getJoinedMesh(true);

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
}

std::string AudioSensor::getSimulationFolder() {
  if (outputFolderPath.empty())
    return std::string("/home/AudioSimulation" + std::to_string(currentSimCount));

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

void AudioSensor::reset()
{
  audioSimulator = nullptr;
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
  if (nullptr == audioSimulator)
  {
    ESP_ERROR() << "getImpulseResponse : AUDIO SIMULATOR NOT CREATED ";
    return 0.0f;
  }

  return audioSimulator->GetImpulseResponse(channelIndex, sampleIndex);
}

}  // namespace sensor
}  // namespace esp

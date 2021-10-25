# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim
import habitat_sim.sim
import habitat_sim.sensor
import habitat_sim._ext.habitat_sim_bindings as hsim_bindings

import numpy as np
from numpy import ndarray

# def make_default_settings():
#     settings = {
#         "width": 1280,  # Spatial resolution of the observations
#         "height": 720,
#         "scene": "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb",  # Scene path
#         "render_asset":
#         "default_agent": 0,
#         "sensor_height": 1.5,  # Height of sensors in meters
#         "sensor_pitch": -math.pi / 8.0,  # sensor pitch (x rotation in rads)
#         "color_sensor_1st_person": True,  # RGB sensor
#         "color_sensor_3rd_person": False,  # RGB sensor 3rd person
#         "depth_sensor_1st_person": False,  # Depth sensor
#         "semantic_sensor_1st_person": False,  # Semantic sensor
#         "seed": 1,
#         "enable_physics": True,  # enable dynamics simulation
#     }
#     return settings

def main():
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = (
        #"data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
        "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb"
        #"data/test_assets/scenes/simple_room.glb"
    )
    backend_cfg.scene_dataset_config_file = ("data/scene_datasets/mp3d_example/17DRP5sb8fy/scene_dataset_config.json")
    backend_cfg.enable_physics = True

    agent_config = habitat_sim.AgentConfiguration()

    cfg = habitat_sim.Configuration(backend_cfg, [agent_config])

    sim = habitat_sim.Simulator(cfg)

    acoustics_config = hsim_bindings.HabitatAcousticsConfiguration()
    acoustics_config.dumpWaveFiles = True
    acoustics_config.enableMaterials = True
    # acoustics_config.threadCount = 2

    audio_sensor_spec = habitat_sim.AudioSensorSpec()
    audio_sensor_spec.uuid = "audio_sensor"

    # add the audio sensor
    sim.add_sensor(audio_sensor_spec)

    # Get the audio sensor object
    audio_sensor = sim.get_agent(0)._sensors["audio_sensor"]

    # set habitat acoustic configs
    audio_sensor.setAudioSimulationConfigs(acoustics_config)

    # optionally set the output folder path
    outputFolderPath = "/home/sangarg/AudioSimulation"
    audio_sensor.setOutputFolder(outputFolderPath)

    # set audio source location, no need to set the agent location, will be set implicitly
    audio_sensor.setAudioSourceLocation(np.array([3.1035, 1.57245, -4.15972]))

    # run the simulation
    for i in range (3):
        print(i)
        p = outputFolderPath + str(i) + "/ir";
        obs = sim.get_sensor_observations()

        # get the simulation results
        channelCount = audio_sensor.getChannelCount()
        sampleCount = audio_sensor.getSampleCount()

        for channelIndex in range (0, channelCount):
            filePath = p + str(channelIndex) + ".txt"
            f = open(filePath, "w")
            print("Writing file : ", filePath)
            for sampleIndex in range (0, sampleCount):
                f.write(str(sampleIndex) + "\t" + str(audio_sensor.getImpulseResponse(channelIndex, sampleIndex)) + "\n")
            f.close()

    sim.close()

if __name__ == "__main__":
    main()

#[Sensor] AudioSensor.cpp(59)::setAudioSourceLocation : LOOOOOGGGGG ------------------ setAudioSourceLocation :  Vector(3.1035, 1.57245, -4.15972)
#[Default] viewer.cpp(359)::showAgentStateMsg : Agent position [2.55447,0.072447,-1.27108] Agent orientation         0 -0.104515         0  0.994523
#[Sensor] AudioSensor.cpp(64)::setAgentLocation : LOOOOOGGGGG ------------------ setAgentLocation :  Vector(2.55447, 0.072447, -1.27108)

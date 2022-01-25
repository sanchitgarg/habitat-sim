# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim
import habitat_sim.sim
import habitat_sim.sensor
import habitat_sim._ext.habitat_sim_bindings as hsim_bindings

import numpy as np
from numpy import ndarray

# test semantic scene load

def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene_id = settings["scene"]
    sim_cfg.enable_physics = settings["enable_physics"]
    sim_cfg.load_semantic_mesh = True
    #settings["load_semantic_mesh"]
    agent_config = habitat_sim.AgentConfiguration()

    return habitat_sim.Configuration(sim_cfg, [agent_config])

def make_default_settings():
    settings = {
        "width": 1280,  # Spatial resolution of the observations
        "height": 720,
        "scene": "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb",  # Scene path
        "render_asset": "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb",
        "house_filename": "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.house",
        "semantic_asset": "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy_semantic.ply",
        "default_agent": 0,
        #"sensor_height": 1.5,  # Height of sensors in meters
        #"sensor_pitch": -math.pi / 8.0,  # sensor pitch (x rotation in rads)
        #"color_sensor_1st_person": True,  # RGB sensor
        #"color_sensor_3rd_person": False,  # RGB sensor 3rd person
        #"depth_sensor_1st_person": False,  # Depth sensor
        #"semantic_sensor_1st_person": False,  # Semantic sensor
        #"seed": 1,
        "enable_physics": True,  # enable dynamics simulation
        "load_semantic_mesh" : True,
    }
    return settings

def testSemanticScene():
    sim_settings = make_default_settings()

    cfg = make_cfg(sim_settings)
    sim = habitat_sim.Simulator(cfg)

    audio_sensor_spec = habitat_sim.AudioSensorSpec()
    audio_sensor_spec.uuid = "audio_sensor"

    # add the audio sensor
    sim.add_sensor(audio_sensor_spec)

    # Get the audio sensor object
    audio_sensor = sim.get_agent(0)._sensors["audio_sensor"]

    # set habitat acoustic configs
    acoustics_config = hsim_bindings.HabitatAcousticsConfiguration()
    acoustics_config.dumpWaveFiles = True
    acoustics_config.enableMaterials = False
    audio_sensor.setAudioSimulationConfigs(acoustics_config)

    # # optionally set the output folder path
    outputFolderPath = "/home/sangarg/AudioSimulation"
    audio_sensor.setOutputFolder(outputFolderPath)

    # # set audio source location, no need to set the agent location, will be set implicitly
    # audio_sensor.setAudioSourceLocation(np.array([-0.628058, 1.68568, 14.3147]))
    audio_sensor.setAudioSourceTransform(np.array([3.1035, 1.57245, -4.15972]), np.array([1, 0, 0, 0]))

    # run the simulation
    obs = sim.get_sensor_observations()
    sim.close()

def main():
    testSemanticScene()

if __name__ == "__main__":
    main()

#r] AudioSensor.cpp(59)::setAudioSourceLocation : LOOOOOGGGGG ------------------ setAudioSourceLocation :  Vector(3.27603, 1.57245, -4.88318)
#[Sensor] AudioSensor.cpp(217)::setOutputFolder : LOOOOOGGGGG ------------------ setOutputFolder:  /home/sangarg/AudioSimulation
#[Sensor] AudioSensor.cpp(64)::setAgentLocation : LOOOOOGGGGG ------------------ setAgentLocation :  Vector(2.30119, 0.072447, -0.698174)

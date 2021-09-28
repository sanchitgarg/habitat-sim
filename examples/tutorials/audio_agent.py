# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim
import habitat_sim.sim
import habitat_sim.sensor
import habitat_sim._ext.habitat_sim_bindings as hsim_bindings

import numpy as np
from numpy import ndarray

def main():
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = (
        "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    )
    backend_cfg.enable_physics = True

    agent_config = habitat_sim.AgentConfiguration()

    cfg = habitat_sim.Configuration(backend_cfg, [agent_config])

    sim = habitat_sim.Simulator(cfg)

    acoustics_config = hsim_bindings.HabitatAcousticsConfiguration()
    acoustics_config.dumpWaveFiles = True

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
    audio_sensor.setAudioSourceLocation(np.array([-0.628058, 1.68568, 14.3147]))

    # run the simulation
    obs = sim.get_sensor_observations()

    # get the simulation results
    channelCount = audio_sensor.getChannelCount()
    sampleCount = audio_sensor.getSampleCount()

    for channelIndex in range (0, channelCount):
        filePath = outputFolderPath + "0/ir" + str(channelIndex) + ".txt"
        f = open(filePath, "w")
        print("Writing file : ", filePath)
        for sampleIndex in range (0, sampleCount):
            f.write(str(sampleIndex) + "\t" + str(audio_sensor.getImpulseResponse(channelIndex, sampleIndex)) + "\n")
        f.close()

    sim.close()

if __name__ == "__main__":
    main()

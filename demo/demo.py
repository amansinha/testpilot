# -*- coding: utf-8 -*-
'''
Copyright (c) 2020, Trustworthy AI, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1.  Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2.  Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3.  Neither the name of the copyright holder(s) nor the names of any contributors
may be used to endorse or promote products derived from this software without
specific prior written permission. No license is granted to the trademarks of
the copyright holders even if such marks are included in this software.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''
###############################################################################
# demo.py
# Author: Aman Sinha
# Purpose: example of running TestPilot agents
# Notes:
#   to make sure my editor saves in utf-8 here is a nice character: é
###############################################################################

import argparse
import glob
import numpy as np
import os
import sys
import time

import carla
sys.path.append('../')
import testpilot
import testpilot.utils.Utils as Utils
import testpilot.utils.CarlaUtils as CarlaUtils


TOWN = 'Town04'
STARTPOSE_TOWN04 = carla.Transform(carla.Location(x=-6.221527, y=-168.1685, z=1.200000),
                                   carla.Rotation(pitch=0.000000, yaw=89.775124, roll=0.000000))
WEATHER_PRESETS = [carla.WeatherParameters.ClearNoon,
                   carla.WeatherParameters.CloudyNoon,
                   carla.WeatherParameters.WetNoon,
                   carla.WeatherParameters.WetCloudyNoon,
                   carla.WeatherParameters.SoftRainNoon,
                   carla.WeatherParameters.MidRainyNoon,
                   carla.WeatherParameters.HardRainNoon,
                   carla.WeatherParameters.ClearSunset,
                   carla.WeatherParameters.CloudySunset,
                   carla.WeatherParameters.WetSunset,
                   carla.WeatherParameters.WetCloudySunset,
                   carla.WeatherParameters.SoftRainSunset,
                   carla.WeatherParameters.MidRainSunset,
                   carla.WeatherParameters.HardRainSunset]
RUNNING_STEPS = 200

def initialize_environment(carla_ip, carla_port, num_testpilots, show_ui):
    testpilots = [testpilot.TestPilot(i, show_ui) for i in range(num_testpilots)]
    done = False
    while not done:
        try:
            client = CarlaUtils.carla_connect(carla_ip, carla_port, TOWN)
            waypoints = CarlaUtils.get_waypoints(client, STARTPOSE_TOWN04)
            done = True
        except RuntimeError:
            time.sleep(1)
    return client, testpilots, waypoints


def simulate(client, testpilots, waypoints, seed, draw=False):
    def cleanup():
        for sensor in sensor_list:
            sensor.destroy()
        for actor in actor_list:
            actor.destroy()

    np.random.seed(seed)
    world = client.get_world()

    #set weather
    world.set_weather(WEATHER_PRESETS[np.random.randint(len(WEATHER_PRESETS))])

    # start testpilots, initialize their poses, and add their sensors
    for tp in testpilots:
        tp.start()
    actor_list = testpilot.set_poses(testpilots, waypoints, client, world)
    sensor_list = CarlaUtils.set_sensors(actor_list, draw, client, world)

    # run the simulation
    with CarlaUtils.CarlaSyncMode(world, *sensor_list, fps=20) as sync_mode:
        for cnt in range(RUNNING_STEPS):
            if draw:
                snapshot, *images, image_hover = sync_mode.tick(timeout=60.0)
            else:
                snapshot, *images = sync_mode.tick(timeout=60.0)

            if cnt == 0:
                CarlaUtils.initialize_velocities_batch(actor_list, [tp.v0 for tp in testpilots], client)
            control_actions = [tp.step(cnt, image) for tp, image in zip(testpilots, images)]
            CarlaUtils.set_controls(actor_list, control_actions, client)
            if draw:
                Utils.draw_image(DISPLAY, image_hover)
                pygame.display.flip()

        # stop testpilots
        for tp in testpilots:
            tp.stop()
        # miscellaneous other cleanup for stopping the simulation
        cleanup()
    if any((sensor.is_alive for sensor in sensor_list)) or any((actor.is_alive for actor in actor_list)):
        client.reload_world()
    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--draw', type=int, default=1, help='draw images to screen')
    parser.add_argument('--num_testpilots', type=int, default=1, help='number of agents')
    parser.add_argument('--testpilot_ui', type=int, default=1, help='show testpilot UI')
    parser.add_argument('--carla_ip', default='localhost', help='IP address for CARLA')
    parser.add_argument('--carla_port', type=int, default=2010, help='Port for CARLA')
    parser.add_argument('--runs', type=int, default=2, help='Number of simulations to run')
    args = parser.parse_args()
    args.draw = bool(args.draw)
    args.testpilot_ui = bool(args.testpilot_ui)
    if args.draw:
        global DISPLAY
        import pygame
        pygame.init()
        DISPLAY = pygame.display.set_mode((1200, 1200), pygame.HWSURFACE | pygame.DOUBLEBUF)
    client, testpilots, waypoints = initialize_environment(carla_ip=args.carla_ip,
                                                           carla_port=args.carla_port,
                                                           num_testpilots=args.num_testpilots,
                                                           show_ui=args.testpilot_ui)
    for i in range(args.runs):
        print('iteration', i)
        simulate(client=client, testpilots=testpilots, waypoints=waypoints, seed=i, draw=args.draw)
    for tp in testpilots:
        tp.kill()

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
# testpilot.py
# Author: Aman Sinha
# Purpose: contains encapsulating class and helper methods for running
#   TestPilot agents.
# Notes:
#   to make sure my editor saves in utf-8 here is a nice character: é
###############################################################################
import numpy as np
import os

from .utils import Utils, CarlaUtils

PATH = os.path.abspath(__file__)
DIR_PATH = os.path.dirname(PATH)


class TestPilot(object):
    DOCKER_IP_PREFIX = '172.18.0.'
    CRUISE_SPEED_MS = 20.0
    CRUISE_SPEED_KPH = CRUISE_SPEED_MS*3.6

    def __init__(self, pilot_id, show_ui):
        self.id = pilot_id
        self.docker_ip = TestPilot.DOCKER_IP_PREFIX + str(self.id+2)
        self.show_ui = show_ui
        if self.show_ui:
            os.system(DIR_PATH+'/testpilot0.5/docker_ui.sh ' + self.docker_ip + ' ' + str(self.id))
        else:
            os.system(DIR_PATH+'/testpilot0.5/docker.sh ' + self.docker_ip + ' ' + str(self.id))
        self.vehicle_type = 'vehicle.toyota.prius'
        self.height = 0.7686808109283447
        self.socket, self.controls_socket, self.time_socket = Utils.open_sockets(self.docker_ip)
        self.v0 = 0.
        self.speed = 0.
        self.steering_angle = 0.

    def start(self):
        if self.show_ui:
            os.system('docker exec -d -w /tmp/openpilot/selfdrive testpilot_'+str(self.id)+' ./script_ui.sh')
        else:
            os.system('docker exec -d -w /tmp/openpilot/selfdrive testpilot_'+str(self.id)+' ./script.sh')
        self.socket.recv()
        self.v0 = np.random.uniform(TestPilot.CRUISE_SPEED_MS/2., TestPilot.CRUISE_SPEED_MS)

    def stop(self):
        os.system('docker exec -d testpilot_'+str(self.id)+' pkill -9 python')

    def step(self, cnt, image_rgb):
        if cnt == 0:
            for _ in range(15):
                Utils.send_image(self.socket, image_rgb)
            self.speed = self.v0
            self.steering_angle = 0.
            return None
        self.time_socket.send_string(str(cnt-1))
        state_array = np.ascontiguousarray([self.speed, self.steering_angle, TestPilot.CRUISE_SPEED_KPH])
        self.time_socket.recv()
        Utils.send_array(self.controls_socket, state_array)
        Utils.send_image(self.socket, image_rgb)
        inputs = Utils.recv_array(self.controls_socket)
        self.speed = inputs[0]
        self.steering_angle = inputs[1]
        return inputs

    def kill(self):
        self.socket.close()
        self.controls_socket.close()
        self.time_socket.close()
        os.system('docker kill testpilot_'+str(self.id))


def set_poses(testpilots, waypoints, client, world):
    idx = np.random.choice(3*len(testpilots), size=len(testpilots), replace=False) + np.random.randint(len(waypoints)-3*len(testpilots))
    poses = [waypoints[i].transform for i in idx]
    vehicle_types = [tp.vehicle_type for tp in testpilots]
    heights = [tp.height for tp in testpilots]
    actor_list = CarlaUtils.poses_batch(poses, vehicle_types, heights, client, world)
    return actor_list

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
# assumes the main runner file has already added carla to system path
import carla
import queue

CARLA_VERSION = [int(x) for x in carla.__file__.split('-')[-4].split('.')]
assert(CARLA_VERSION[0]==0 and CARLA_VERSION[1]==9), 'TestPilot works with Carla 0.9.x'
apply_velocity = carla.command.ApplyVelocity if CARLA_VERSION[2]<10 else carla.command.ApplyTargetVelocity
apply_angular_velocity = carla.command.ApplyAngularVelocity if CARLA_VERSION[2]<10 else carla.command.ApplyTargetAngularVelocity


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """
    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.map = self.world.get_map()
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def set_sensors(vehicles, draw, client, world):
    actor_list = []
    blueprint_library = world.get_blueprint_library()
    cam = blueprint_library.find('sensor.camera.rgb')
    cam.set_attribute('image_size_x', '1164')
    cam.set_attribute('image_size_y', '874')
    batch = [carla.command.SpawnActor(cam,
                                      carla.Transform(carla.Location(x=0.7, z=1.22)),
                                      actor.id) for actor in vehicles]
    if draw:
        cam = blueprint_library.find('sensor.camera.rgb')
        cam.set_attribute('image_size_x', '1200')
        cam.set_attribute('image_size_y', '1200')
        batch.append(carla.command.SpawnActor(
                        cam,
                        carla.Transform(carla.Location(x=-30.0, z=20.8), carla.Rotation(pitch=-15)),
                        vehicles[0].id))
    for response in client.apply_batch_sync(batch):
        if response.has_error():
            assert(1 == 0), response.error
        else:
            actor_list.append(response.actor_id)
    return world.get_actors(actor_list)


def poses_batch(poses, vehicle_types, heights, client, world):
    actor_list = []
    batch = []
    bp = world.get_blueprint_library()
    for pose, vehicle_type, height in zip(poses, vehicle_types, heights):
        transform = carla.Transform(
            carla.Location(x=pose.location.x, y=pose.location.y, z=pose.location.z+height+0.001),
            pose.rotation)
        vt = bp.find(vehicle_type)
        batch.append(carla.command.SpawnActor(vt, transform).then(carla.command.SetSimulatePhysics(carla.command.FutureActor, True)))
    for response in client.apply_batch_sync(batch):
        if response.has_error():
            assert(1 == 0), response.error
        else:
            actor_list.append(response.actor_id)
    return world.get_actors(actor_list)


def initialize_velocities_batch(vehicles, vels, client):
    batch = []
    for vehicle, vel in zip(vehicles, vels):
        rot = vehicle.get_transform().rotation
        fv = rot.get_forward_vector()
        batch.append(apply_velocity(vehicle, vel*fv))
    for response in client.apply_batch_sync(batch):
        if response.has_error():
            assert(1 == 0), response.error


def carla_connect(host, port, town):
    client = carla.Client(host, port)
    client.set_timeout(20.0)
    client.load_world(town)
    return client


def set_controls(actor_list, inputs_list, client):
    batch = []
    for vehicle, inputs in zip(actor_list, inputs_list):
        if inputs is None:
            continue
        batch.extend(_set_ego_controls_batch(vehicle, inputs))
    for response in client.apply_batch_sync(batch):
        if response.has_error():
            assert(1 == 0), response.error


def _set_ego_controls_batch(vehicle, inputs):
    rot = vehicle.get_transform().rotation
    rot.pitch += 90
    vv = rot.get_forward_vector()
    rot.pitch -= 90
    rot.yaw += inputs[3]
    fv = rot.get_forward_vector()
    return [apply_velocity(vehicle, inputs[0]*fv),
            apply_angular_velocity(vehicle, inputs[2]*vv)]


def get_waypoints(client, startpose, waypoint_separation=20., max_distance=1050):
    m = client.get_world().get_map()
    current_w = m.get_waypoint(startpose.location)
    waypoints = [current_w]
    for _ in range(int(max_distance/waypoint_separation)):
        waypoints.append(waypoints[-1].next(waypoint_separation)[0])
    all_waypoints = []
    for w in waypoints:
        all_waypoints.append(w)
        temp_w = w
        for _ in range(3):
            if temp_w.lane_change & carla.LaneChange.Right:
                right_w = temp_w.get_right_lane()
            if right_w and right_w.lane_type == carla.LaneType.Driving:
                all_waypoints.append(right_w)
            temp_w = right_w
    return all_waypoints

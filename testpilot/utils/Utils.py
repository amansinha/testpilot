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
import cv2
import numba
import numpy as np
import pygame
import zmq


def open_sockets(docker_ip, context=None):
    context = zmq.Context.instance()
    socket = context.socket(zmq.PAIR)
    socket.connect("tcp://" + docker_ip + ":5559")
    controls_socket = context.socket(zmq.PAIR)
    controls_socket.connect("tcp://" + docker_ip + ":5560")
    time_socket = context.socket(zmq.PAIR)
    time_socket.connect("tcp://" + docker_ip + ":5561")
    return socket, controls_socket, time_socket


def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


@numba.njit
def _process_image(buf, h, w):
    array = np.frombuffer(buf, dtype=np.dtype("uint8"))
    array = np.reshape(array, (h, w, 4))
    array = array[:, :, :3]
    return array


def send_image(socket, image, blend=False):
    """send image from carla to testpilot"""
    buf = image.raw_data
    h = image.height
    w = image.width
    array = _process_image(buf, h, w)
    socket.send(bytearray(cv2.imencode('.bmp', array)[1].tostring()))
    return np.copy(array)


def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    buf = memoryview(msg)
    A = np.frombuffer(buf, dtype=md['dtype'])
    return A.reshape(md['shape'])


def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(dtype=str(A.dtype), shape=A.shape)
    socket.send_json(md, flags | zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)

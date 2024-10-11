#!/usr/bin/python3
# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# If you compiled Gazebo from source you should modify your
# `PYTHONPATH`:
#
# export PYTHONPATH=$PYTHONPATH:<path to ws>/install/lib/python
#
# Now you can run the example:
#
# python3 examples/scripts/python_api/testFixture.py

import os

from gz.common6 import set_verbosity
from gz.sim9 import TestFixture, World, world_entity, Model, Joint
from gz.math8 import Vector3d
from gz.transport14 import Node
from gz.msgs11.world_control_pb2 import WorldControl
from gz.msgs11.world_reset_pb2 import WorldReset
from gz.msgs11.boolean_pb2 import Boolean
set_verbosity(4)

file_path = os.path.dirname(os.path.realpath(__file__))

fixture = TestFixture(os.path.join(file_path, 'world.sdf'))

post_iterations = 0
iterations = 0
pre_iterations = 0
first_iteration = True
joint_dict = {}

def on_pre_udpate_cb(_info, _ecm):
    global pre_iterations
    global first_iteration
    global joint_dict
    print("PReupdate running")
    pre_iterations += 1
    #if first_iteration:
    first_iteration = False
    world_e = world_entity(_ecm)
    print('World entity is ', world_e)
    w = World(world_e)
    v = w.gravity(_ecm)
    print('Gravity ', v)
    modelEntity = w.model_by_name(_ecm, 'cerberus_anymal_b_sensor_config_1')

    print('Entity for falling model is: ', modelEntity)
    joints = Model(modelEntity).joints(_ecm)
    for joint_entity in joints:
        j = Joint(joint_entity)
        #print(j.name(_ecm))
        j.enable_position_check(_ecm)
        joint_dict[j.name(_ecm)] = {"entity": joint_entity}

def on_post_udpate_cb(_info, _ecm):
    global joint_dict
    poses = {}
    
    print(_info.paused)
    if _info.paused:
        return
    
    for joint_name in joint_dict:
        entity = joint_dict[joint_name]["entity"]
        joint = Joint(entity)
        if not joint.valid(_ecm):
            print("invalid joint")
            continue
        pose = joint.position(_ecm)
        if pose is not None and len(pose) > 0:
            poses[joint_name] = pose[0]
    print(poses)


fixture.on_post_update(on_post_udpate_cb)

fixture.on_pre_update(on_pre_udpate_cb)
fixture.finalize()

server = fixture.server()
server.run(True, 1000, False)
reset = WorldReset(all=True, time_only=False, model_only=False)
node = Node()

node.request("/world/empty/control", WorldControl(
    reset = reset), WorldControl, Boolean, 300)
first_iteration = True
print("Resetting")
server.run(True, 1000, False)
print('iterations ', iterations)
print('post_iterations ', post_iterations)
print('pre_iterations ', pre_iterations)

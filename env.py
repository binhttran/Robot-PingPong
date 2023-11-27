import numpy as np
import robosuite as suite
from robosuite.models.robots import Panda
from robosuite.models import MujocoWorldBase
from robosuite.models.grippers import gripper_factory

world = MujocoWorldBase()

mujoco_robot = Panda()

gripper = gripper_factory('PandaGripper')
mujoco_robot.add_gripper(gripper)

mujoco_robot.set_base_xpos([0, 0, 0])
world.merge(mujoco_robot)

from robosuite.models.arenas import TableArena

mujoco_arena = TableArena()
mujoco_arena.set_origin([0.8, 0, 0])
world.merge(mujoco_arena)

from robosuite.models.objects import BallObject
from robosuite.utils.mjcf_utils import new_joint

sphere = BallObject(
    name="sphere",
    size=[0.04],
    rgba=[0, 0.5, 0.5, 1]).get_obj()
sphere.set('pos', '1.0 0 1.0')
world.worldbody.append(sphere)

model = world.get_model(mode="mujoco")

import mujoco

data = mujoco.MjData(model)
while data.time < 1:
    mujoco.mj_step(model, data)
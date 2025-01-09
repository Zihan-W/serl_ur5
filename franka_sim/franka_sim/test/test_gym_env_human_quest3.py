import time
import gymnasium as gym
from ur_env.utils.socket_server import SocketServer

import mujoco
import mujoco.viewer
import numpy as np

from franka_sim import envs


class Quest3Wrapper(gym.ActionWrapper):
    def __init__(self, env):
        super().__init__(env)

        self.socket_server = SocketServer(ip_port=('169.254.91.200', 34567))
    
    def action(self, action) -> np.ndarray:
        quest3_action, button_x, button_a = self.socket_server.get_action()
        print(f"quest3_action: {quest3_action}")

        if button_a == 1:
            return np.zeros(4)
        
        return np.array([-quest3_action[3]*0.1, -quest3_action[4]*0.1, quest3_action[5]*0.1, 1-quest3_action[6]])

    def step(self, action):
        new_action = self.action(action)
        print(f"new action: {new_action}")
        obs, rew, done, truncated, info = self.env.step(new_action)
        # info["intervene_action"] = new_action
        # info["left"] = self.left.any()
        # info["right"] = self.right.any()
        return obs, rew, done, truncated, info


env = envs.PandaPickCubeGymEnv(action_scale=(0.1, 1))
env = Quest3Wrapper(env)
action_spec = env.action_space


def sample():
    a = np.random.uniform(action_spec.low, action_spec.high, action_spec.shape)
    return a.astype(action_spec.dtype)


m = env.model
d = env.data

reset = False
KEY_SPACE = 32


def key_callback(keycode):
    if keycode == KEY_SPACE:
        global reset
        reset = True


env.reset()
with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
    start = time.time()
    while viewer.is_running():
        if reset:
            env.reset()
            reset = False
        else:
            step_start = time.time()
            env.step(sample())
            viewer.sync()
            time_until_next_step = env.control_dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

from stable_baselines3.common.vec_env import SubprocVecEnv


NUM_ROBOTS = 8


def make_env(rank):
    def _init():
        env = MyGymEnv()
        env.seed(100 + rank)
        return env
    return _init


env = SubprocVecEnv([make_env(i) for i in range(NUM_ROBOTS)])
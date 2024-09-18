import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)


register(
    id='slider_env-v0',
    entry_point='rl_slider.envs:SliderEnv',
)
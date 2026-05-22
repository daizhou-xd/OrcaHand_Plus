import gymnasium as gym
import pytest

from orca_sim import register_envs

# Instantiating all available environments regardless of versioning
pytestmark = pytest.mark.filterwarnings(
    "ignore:.*environment .* is out of date.*:DeprecationWarning"
)


def test_register_envs_is_idempotent_and_envs_can_be_made() -> None:
    register_envs()
    register_envs()

    for env_id, obs_shape, action_shape, expect_empty_info in [
        ("OrcaHandLeft-v1", (34,), (17,), True),
        ("OrcaHandLeft-v2", (34,), (17,), True),
        ("OrcaHandRight-v1", (34,), (17,), True),
        ("OrcaHandRight-v2", (34,), (17,), True),
        ("OrcaHandRightCubeOrientation-v1", (51,), (17,), False),
        ("OrcaHandRightCubeOrientation-v2", (51,), (17,), False),
        ("OrcaHandCombined-v1", (68,), (34,), True),
        ("OrcaHandCombined-v2", (68,), (34,), True),
    ]:
        assert env_id in gym.registry

        env = gym.make(env_id)
        try:
            obs, info = env.reset()

            assert obs.shape == obs_shape
            assert env.action_space.shape == action_shape
            if expect_empty_info:
                assert info == {}
            else:
                assert "red_face_up_alignment" in info
        finally:
            env.close()

import numpy as np
import pytest
from physics_sim.servos import PROFILES, ServoBank


def test_commands_are_delayed_and_acceleration_limited():
    bank = ServoBank(PROFILES['7.4v'], [0.])
    for _ in range(2):
        assert bank.update([1.], [0.], [0.])[0] == 0
    previous_velocity = 0.
    for _ in range(100):
        bank.update([1.], [0.], [0.])
        assert abs(bank.velocity[0]) <= bank.spec.command_speed + 1e-10
        # Exclude the final position clamp, which may remove commanded velocity.
        if bank.target[0] < 1.:
            assert abs(bank.velocity[0] - previous_velocity) <= .050001
        previous_velocity = bank.velocity[0]


@pytest.mark.parametrize('profile', list(PROFILES))
def test_stall_and_speed_limits_cannot_be_overridden_by_large_error(profile):
    bank = ServoBank(PROFILES[profile], [2., -2.])
    torque = bank.update([2., -2.], [0., 0.], [0., 0.])
    assert np.all(np.abs(torque) <= bank.spec.stall_torque)
    torque = bank.update([2., -2.], [0., 0.], [bank.spec.no_load_speed, -bank.spec.no_load_speed])
    assert np.allclose(torque, 0)
    assert np.all(bank.saturated)


def test_disable_removes_torque_and_invalid_feedback_is_rejected():
    bank = ServoBank(PROFILES['12v'], [1.])
    assert bank.update([1.], [0.], [0.])[0] > 0
    assert bank.update([1.], [0.], [0.], enabled=False)[0] == 0
    with pytest.raises(ValueError):
        bank.update([1.], [float('nan')], [0.])


def test_sustained_stall_derates_toward_continuous_rating():
    bank = ServoBank(PROFILES['7.4v'], [2.])
    for _ in range(10000):
        bank.update([2.], [0.], [0.])
    assert bank.spec.continuous_torque <= bank.torque[0] < bank.spec.continuous_torque * 1.01

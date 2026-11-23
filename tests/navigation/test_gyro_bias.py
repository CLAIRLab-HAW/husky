"""gyro_bias's core: it learns the gyro's bias only while the wheels stand, and takes it off every rate."""

from __future__ import annotations

import pytest

from husky_navigation import gyro_bias as gb

BIAS = (5.7e-3, -1.5e-3, 6e-4)  # the a200-0553's UM7 at rest, x / y / z (2026-11-23)


def _stand(state: gb.GyroBias, seconds: float, rate=BIAS, t0: float = 0.0) -> float:
    """Wheels at 10 Hz reporting standstill, the gyro at 20 Hz reporting ``rate``; returns the end time."""
    t = t0
    while t < t0 + seconds:
        if round(t * 20) % 2 == 0:
            state.wheels(t, 0.0, 0.0)
        state.correct(t, rate)
        t += 0.05
    return t


def test_at_rest_the_bias_is_learned_and_taken_off():
    state = gb.GyroBias()
    end = _stand(state, 5 * gb.TAU_S)
    # an exponential mean after ~5 time constants: within 1 % of the bias
    assert state.bias == pytest.approx(BIAS, rel=0.01)
    assert state.correct(end, BIAS) == pytest.approx((0.0, 0.0, 0.0), abs=1e-4)


def test_nothing_is_learned_in_the_first_second_after_the_wheels_stop():
    state = gb.GyroBias()
    _stand(state, gb.STILL_S * 0.9)
    assert state.bias == [0.0, 0.0, 0.0]


def test_a_turn_is_not_learned_and_the_learned_bias_comes_off_it():
    state = gb.GyroBias()
    t = _stand(state, 5 * gb.TAU_S)
    learned = list(state.bias)
    turn = (0.0, -0.4, 0.0)  # 0.4 rad/s about the base's up axis, the UM7's -y
    for _ in range(40):
        state.wheels(t, 0.0, 0.4)
        out = state.correct(t, turn)
        t += 0.05
    assert state.bias == learned
    assert out[1] == pytest.approx(-0.4 - learned[1])


def test_stale_wheel_odometry_says_nothing_about_standstill():
    state = gb.GyroBias()
    state.wheels(0.0, 0.0, 0.0)
    for k in range(1, 200):
        state.correct(k * 0.05, BIAS)  # no wheel report after t=0
    assert state.bias == [0.0, 0.0, 0.0]


def test_the_learned_bias_is_capped():
    state = gb.GyroBias()
    _stand(state, 10 * gb.TAU_S, rate=(0.3, 0.0, 0.0))  # a base turned by hand while its wheels stand
    assert state.bias[0] == pytest.approx(gb.MAX_BIAS_RAD_S)

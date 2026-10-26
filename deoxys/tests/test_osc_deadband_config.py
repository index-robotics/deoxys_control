"""A deadband the server silently ignores, or one that swallows the stiffness, leaves the arm off target."""

import math

import pytest
from easydict import EasyDict

from deoxys.utils.config_utils import verify_controller_config


def _osc_cfg(deadband) -> EasyDict:
    return EasyDict(
        {
            "controller_type": "OSC_POSE",
            "Kp": {"translation": 150.0, "rotation": 250.0},
            "deadband": deadband,
        }
    )


@pytest.mark.parametrize(
    "deadband",
    [
        5e-4,
        {"translation": -1e-4},
        {"rotation": math.nan},
        {"translation": 0.05},
        {"rotation": 0.5},
        {"rotation": [5e-4, 5e-4]},
    ],
)
def test_invalid_deadband_rejected(deadband):
    with pytest.raises(ValueError):
        verify_controller_config(_osc_cfg(deadband))

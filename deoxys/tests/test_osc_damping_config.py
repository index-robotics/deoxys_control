"""Negative/NaN Kd injects energy along dq: the lunge-to-power_limit_violation failure."""

import math

import pytest
from easydict import EasyDict

from deoxys.utils.config_utils import verify_controller_config


def _osc_cfg(kd) -> EasyDict:
    cfg = {"controller_type": "OSC_POSE", "Kp": {"translation": 150.0, "rotation": 250.0}}
    if kd is not None:
        cfg["Kd"] = kd
    return EasyDict(cfg)


@pytest.mark.parametrize(
    "kd", [{"rotation": -46.9}, {"rotation": math.nan}, {"translation": [1.0, 2.0]}]
)
def test_invalid_kd_rejected(kd):
    with pytest.raises(ValueError):
        verify_controller_config(_osc_cfg(kd))

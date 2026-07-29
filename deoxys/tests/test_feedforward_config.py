"""Back-compat tests for the feedforward default-fill in verify_controller_config.

The safety-critical guarantee: a JOINT_IMPEDANCE config written before the
feedforward feature (no ``feedforward_cfg`` block) must keep passing
``verify_controller_config(use_default=False)``. The default-fill is deliberately
silent (it does NOT set ``field_missing``), so these tests lock that it neither
raises nor leaves ``feedforward_cfg`` absent -- the invariant most likely to
regress if someone later "tidies up" the fill logic.
"""

import pytest
from easydict import EasyDict

from deoxys.utils.config_utils import verify_controller_config


def _pre_feature_joint_cfg() -> EasyDict:
    """A JOINT_IMPEDANCE config as written before feedforward existed."""
    return EasyDict(
        {
            "controller_type": "JOINT_IMPEDANCE",
            "traj_interpolator_cfg": {
                "traj_interpolator_type": "LINEAR_JOINT_POSITION",
                "time_fraction": 0.3,
            },
            "joint_kp": [160.0, 160.0, 160.0, 160.0, 85.0, 165.0, 55.0],
            "joint_kd": [25.0, 25.0, 25.0, 25.0, 18.0, 26.0, 15.0],
        }
    )


def test_pre_feature_config_passes_without_defaults():
    """No feedforward_cfg + use_default=False must not raise, and must fill it."""
    cfg = _pre_feature_joint_cfg()
    assert "feedforward_cfg" not in cfg
    verify_controller_config(cfg, use_default=False)
    # Filled with the baseline (disabled) block, so control() can read it.
    assert cfg.feedforward_cfg.enable is False
    assert cfg.feedforward_cfg.vel_scale == 1.0
    assert cfg.feedforward_cfg.acc_scale == 1.0
    assert cfg.feedforward_cfg.use_coriolis is False


def test_partial_feedforward_block_gets_subkey_defaults():
    """A hand-written partial block (only `enable`) gets the remaining defaults."""
    cfg = _pre_feature_joint_cfg()
    cfg["feedforward_cfg"] = EasyDict({"enable": True})
    verify_controller_config(cfg, use_default=False)
    assert cfg.feedforward_cfg.enable is True  # operator's value preserved
    assert cfg.feedforward_cfg.vel_scale == 1.0  # sub-keys defaulted
    assert cfg.feedforward_cfg.acc_scale == 1.0
    assert cfg.feedforward_cfg.use_coriolis is False


def test_missing_required_field_still_raises_without_defaults():
    """The silent FF fill must not mask a genuinely missing required field."""
    cfg = EasyDict(
        {
            "controller_type": "JOINT_IMPEDANCE",
            "traj_interpolator_cfg": {
                "traj_interpolator_type": "LINEAR_JOINT_POSITION",
                "time_fraction": 0.3,
            },
            # joint_kp deliberately absent -> field_missing -> must raise.
            "joint_kd": [25.0, 25.0, 25.0, 25.0, 18.0, 26.0, 15.0],
        }
    )
    with pytest.raises(ValueError):
        verify_controller_config(cfg, use_default=False)

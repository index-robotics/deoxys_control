"""Byte-compatibility tests for the OSC_POSE velocity feedforward.

The feedforward config is an additive proto3 field (``feedforward = 6``) on
``FrankaOSCPoseControllerMessage``. The safety-critical guarantee is that the
*disabled* path is byte-for-byte identical to a client without this feature, so
``ff_enable=false`` reverts the wire format exactly and an old client talking to
a new binary (or vice versa) degrades cleanly to the baseline law.
"""

import deoxys.proto.franka_interface.franka_controller_pb2 as p

# Serialization of a baseline OSC_POSE message (feedforward untouched), recorded
# with the pre-feedforward stubs (main). Because the only new field is the
# (unset) message field 6, this blob is exactly what a pre-feedforward client
# emits for the same goal/stiffness/config.
GOLDEN_HEX = (
    "0a3611000000000000e03f199a9999999999b9bf21333333333333d33f29cdcccccccccc0840"
    "319a9999999999a93f399a9999999999c9bf12180000000000207c400000000000207c400000"
    "000000207c401a180000000000406f400000000000406f400000000000406f402a3a0a380000"
    "0000000000000000000000000000000000000000000000000000000000009a9999999999b93f"
    "000000000000e03f000000000000e03f"
)


def _baseline_msg() -> p.FrankaOSCPoseControllerMessage:
    msg = p.FrankaOSCPoseControllerMessage()
    msg.goal.is_delta = False
    msg.goal.x, msg.goal.y, msg.goal.z = 0.5, -0.1, 0.3
    msg.goal.ax, msg.goal.ay, msg.goal.az = 3.1, 0.05, -0.2
    msg.translational_stiffness[:] = [450.0] * 3
    msg.rotational_stiffness[:] = [250.0] * 3
    msg.config.residual_mass_vec[:] = [0.0, 0.0, 0.0, 0.0, 0.1, 0.5, 0.5]
    return msg


def test_disabled_feedforward_is_byte_identical():
    """A message that never touches ``feedforward`` matches the pre-feature blob."""
    msg = _baseline_msg()
    # The field must be entirely absent from the wire, not merely default.
    assert "feedforward" not in {f.name for f, _ in msg.ListFields()}
    assert msg.SerializeToString().hex() == GOLDEN_HEX


def test_default_feedforward_submessage_adds_no_bytes():
    """Reading (not setting) the submessage must not materialize it on the wire."""
    msg = _baseline_msg()
    assert msg.feedforward.ff_enable is False
    assert "feedforward" not in {f.name for f, _ in msg.ListFields()}
    assert msg.SerializeToString().hex() == GOLDEN_HEX


def test_enabled_feedforward_roundtrips():
    """The enabled path carries the flag, scales and 6-vectors through a round trip."""
    msg = _baseline_msg()
    msg.feedforward.ff_enable = True
    msg.feedforward.ff_vel_scale = 1.0
    msg.feedforward.ff_acc_scale = 0.5
    msg.feedforward.v_d[:] = [0.1, -0.2, 0.3, 0.4, -0.5, 0.6]
    msg.feedforward.a_d[:] = [0.2] * 6

    assert "feedforward" in {f.name for f, _ in msg.ListFields()}

    parsed = p.FrankaOSCPoseControllerMessage()
    parsed.ParseFromString(msg.SerializeToString())
    assert parsed.feedforward.ff_enable is True
    assert parsed.feedforward.ff_vel_scale == 1.0
    assert parsed.feedforward.ff_acc_scale == 0.5
    assert list(parsed.feedforward.v_d) == [0.1, -0.2, 0.3, 0.4, -0.5, 0.6]
    assert list(parsed.feedforward.a_d) == [0.2] * 6
    # Baseline goal/gains are preserved alongside the feedforward block.
    assert list(parsed.translational_stiffness) == [450.0] * 3

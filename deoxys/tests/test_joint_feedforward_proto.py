"""Byte-compatibility tests for the JOINT_IMPEDANCE computed-torque feedforward.

The feedforward config is an additive proto3 field (``feedforward = 4``) on
``FrankaJointImpedanceControllerMessage``. The safety-critical guarantee is that
the *disabled* path is byte-for-byte identical to a client without this feature,
so ``ff_enable=false`` reverts the wire format exactly and an old client talking
to a new binary (or vice versa) degrades cleanly to the baseline law.
"""

import deoxys.proto.franka_interface.franka_controller_pb2 as p

# Serialization of a baseline JOINT_IMPEDANCE message (feedforward untouched)
# with the sim-validated "medium" gains, recorded with these stubs. Because the
# only new field is the (unset) message field 4, this blob is exactly what a
# pre-feedforward client emits for the same goal/kp/kd.
GOLDEN_HEX = (
    "0a3f119a9999999999b93f199a9999999999c9bf21333333333333d33f29000000000000f8bf"
    "31000000000000e03f39333333333333f33f41666666666666e6bf12380000000000006440"
    "00000000000064400000000000006440000000000000644000000000004055400000000000"
    "a064400000000000804b401a380000000000003940000000000000394000000000000039400"
    "00000000000394000000000000032400000000000003a400000000000002e40"
)

_GOAL_Q = [0.1, -0.2, 0.3, -1.5, 0.5, 1.2, -0.7]
_KP = [160.0, 160.0, 160.0, 160.0, 85.0, 165.0, 55.0]
_KD = [25.0, 25.0, 25.0, 25.0, 18.0, 26.0, 15.0]


def _baseline_msg() -> p.FrankaJointImpedanceControllerMessage:
    msg = p.FrankaJointImpedanceControllerMessage()
    msg.goal.is_delta = False
    for i, q in enumerate(_GOAL_Q, start=1):
        setattr(msg.goal, f"q{i}", q)
    msg.kp[:] = _KP
    msg.kd[:] = _KD
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
    # Touching an accessor without assignment leaves proto3 messages absent.
    assert msg.feedforward.ff_enable is False
    assert "feedforward" not in {f.name for f, _ in msg.ListFields()}
    assert msg.SerializeToString().hex() == GOLDEN_HEX


def test_enabled_feedforward_roundtrips():
    """The enabled path carries the flags and 7-vectors through a round trip."""
    msg = _baseline_msg()
    msg.feedforward.ff_enable = True
    msg.feedforward.ff_vel_scale = 1.0
    msg.feedforward.ff_acc_scale = 1.0
    msg.feedforward.use_coriolis = False
    msg.feedforward.dq_d[:] = [0.1] * 7
    msg.feedforward.ddq_d[:] = [0.2] * 7

    assert "feedforward" in {f.name for f, _ in msg.ListFields()}

    parsed = p.FrankaJointImpedanceControllerMessage()
    parsed.ParseFromString(msg.SerializeToString())
    assert parsed.feedforward.ff_enable is True
    assert parsed.feedforward.ff_vel_scale == 1.0
    assert list(parsed.feedforward.dq_d) == [0.1] * 7
    assert list(parsed.feedforward.ddq_d) == [0.2] * 7
    # Baseline goal/gains are preserved alongside the feedforward block.
    assert list(parsed.kp) == _KP

"""OSC_POSE velocity feedforward: the disabled path must match a pre-feature client."""

import deoxys.proto.franka_interface.franka_controller_pb2 as p

# A baseline OSC_POSE message serialized with the pre-feedforward proto (main).
GOLDEN_HEX = (
    "0a3611000000000000e03f199a9999999999b9bf21333333333333d33f29cdcccccccccc0840"
    "319a9999999999a93f399a9999999999c9bf12180000000000207c400000000000207c400000"
    "000000207c401a180000000000406f400000000000406f400000000000406f402a3a0a380000"
    "0000000000000000000000000000000000000000000000000000000000009a9999999999b93f"
    "000000000000e03f000000000000e03f"
)


def test_disabled_feedforward_is_byte_identical():
    msg = p.FrankaOSCPoseControllerMessage()
    msg.goal.is_delta = False
    msg.goal.x, msg.goal.y, msg.goal.z = 0.5, -0.1, 0.3
    msg.goal.ax, msg.goal.ay, msg.goal.az = 3.1, 0.05, -0.2
    msg.translational_stiffness[:] = [450.0] * 3
    msg.rotational_stiffness[:] = [250.0] * 3
    msg.config.residual_mass_vec[:] = [0.0, 0.0, 0.0, 0.0, 0.1, 0.5, 0.5]
    assert msg.feedforward.ff_enable is False  # reading must not materialize it
    assert msg.SerializeToString().hex() == GOLDEN_HEX


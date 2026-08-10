import struct
import unittest

import xt_proto



class EncoderCalibrationProtocolTest(unittest.TestCase):
    def test_pack_encoder_mapping_uses_alignment_current(self):
        frame = xt_proto.pack_cal_enc(
            current_a=1.25, omega_elec_rad_s=40.0, seq=11
        )
        fields = struct.unpack("<BBBBBBBii", frame)
        self.assertEqual(fields[:4], (0x58, 1, 1, 11))
        self.assertEqual(fields[4:7], (7, 1, 0))
        self.assertEqual(fields[7:], (1250, 40000))

class CalibrationProtocolTest(unittest.TestCase):
    def test_pack_l_ident_uses_voltage_and_per_axis_trials(self):
        frame = xt_proto.pack_cal_l(step_voltage_v=2.75, n_trials=7, seq=19)
        self.assertEqual(len(frame), 15)
        magic, version, msg_type, seq, cmd, kind, reserved, voltage_mv, trials = (
            struct.unpack("<BBBBBBBii", frame)
        )
        self.assertEqual((magic, version, msg_type, seq), (0x58, 1, 1, 19))
        self.assertEqual((cmd, kind, reserved), (7, 5, 0))
        self.assertEqual(voltage_mv, 2750)
        self.assertEqual(trials, 7)

    def test_decode_l_ident_reports_ld_and_lq(self):
        frame = struct.pack(
            "<BBBBBBHiibBH",
            xt_proto.MAGIC,
            xt_proto.VERSION,
            xt_proto.TYPE_CAL,
            23,
            xt_proto.CAL_SUB_INDUCTANCE,
            4,
            1000,
            340_000,
            410_000,
            0,
            1,
            0x800A,
        )
        result = xt_proto.parse_frame(frame)
        self.assertIsInstance(result, xt_proto.CalTelem)
        self.assertAlmostEqual(result.offset_rad, 340e-6)
        self.assertAlmostEqual(result.residual_rad, 410e-6)
        self.assertEqual(result.sample_count, 10)
        self.assertTrue(result.persisted)

    def test_decode_encoder_mapping_keeps_electrical_residual_units(self):
        frame = struct.pack(
            "<BBBBBBHiibBH",
            xt_proto.MAGIC,
            xt_proto.VERSION,
            xt_proto.TYPE_CAL,
            7,
            xt_proto.CAL_SUB_ENC_PHASE,
            4,
            1000,
            125,
            42,
            -1,
            1,
            240,
        )
        result = xt_proto.parse_frame(frame)
        self.assertAlmostEqual(result.offset_rad, 0.125)
        self.assertAlmostEqual(result.residual_rad, 0.042)
        self.assertEqual(result.sign, -1)
        self.assertEqual(result.sample_count, 240)

class ServoCommandProtocolTest(unittest.TestCase):
    def test_pack_servo_uses_velocity_and_d_axis_current(self):
        frame = xt_proto.pack_servo(12.345, -0.678, seq=7)
        self.assertEqual(
            struct.unpack("<BBBBBii", frame),
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 7,
             xt_proto.CMD_SERVO, 12_345, -678),
        )


class ControlReplyProtocolTest(unittest.TestCase):
    def test_decode_extended_speed_and_voltage_observability(self):
        frame = struct.pack(
            xt_proto._CTRL_FMT,
            xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CTRL_REPLY, 9,
            xt_proto.CMD_SERVO, 0, 0, 4, 1, 1, 0,
            100, 200, 0, 3000, 700, 188_800, 1000, 12_000,
            48_000, 1234, 500, 200_000, 2_643_200, 6_250,
        )
        result = xt_proto.parse_frame(frame)
        self.assertIsInstance(result, xt_proto.CtrlReply)
        self.assertAlmostEqual(result.omega_mech_rad_s, 188.8)
        self.assertAlmostEqual(result.omega_cmd_rad_s, 200.0)
        self.assertAlmostEqual(result.omega_elec_rad_s, 2643.2)
        self.assertAlmostEqual(result.voltage_headroom_v, 6.25)

    def test_decode_legacy_control_reply(self):
        frame = struct.pack(
            xt_proto._CTRL_BASE_FMT,
            xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CTRL_REPLY, 9,
            xt_proto.CMD_QUERY, 0, 0, 0, 1, 1, 0,
            0, 0, 0, 0, 0, 10_000, 0, 0,
            48_000, 0, 0, 0,
        )
        result = xt_proto.parse_frame(frame)
        self.assertIsInstance(result, xt_proto.CtrlReply)
        self.assertAlmostEqual(result.omega_mech_rad_s, 10.0)
        self.assertEqual(result.omega_elec_rad_s, 0.0)
        self.assertEqual(result.voltage_headroom_v, 0.0)



class SnapProtocolTest(unittest.TestCase):
    def test_parse_seven_channel_snapdata(self):
        # Firmware 0.4.13: 7 channels (Id,Iq,I1,I2,I3,θmech,θelec) x 4 samples,
        # 56 B payload + 8 B header = 64 B FD frame.
        n = 4
        ch = xt_proto.SNAP_CHANNELS
        self.assertEqual(ch, 7)
        raw = list(range(1, 1 + n * ch))  # nonzero int16 samples
        frame = struct.pack(
            "<BBBBHBB",
            xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_SNAP_DATA, 3,
            0, n, 0,
        ) + struct.pack("<" + "h" * (n * ch), *raw)
        self.assertEqual(len(frame), 64)
        result = xt_proto.parse_frame(frame)
        self.assertIsInstance(result, xt_proto.SnapData)
        self.assertEqual(result.n, 4)
        self.assertEqual(len(result.samples_mA), n * ch)
        self.assertEqual(result.samples_mA, list(raw))

    def test_snap_channel_keys_match_firmware_order(self):
        self.assertEqual(xt_proto.SNAP_CHANNEL_KEYS, [
            "id_a", "iq_a", "i1_a", "i2_a", "i3_a",
            "theta_mech_rad", "theta_elec_rad",
        ])


if __name__ == "__main__":
    unittest.main()

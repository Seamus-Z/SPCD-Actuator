import math
import struct
import unittest

import xt_proto
from gui.web_server import CanBridge



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
    _FMT = "<BBBBBBBHiiiiiiiiiHHHH"

    def test_pack_servo_uses_velocity_and_d_axis_current(self):
        frame = xt_proto.pack_servo(12.345, -0.678, seq=7)
        self.assertEqual(
            struct.unpack(self._FMT, frame),
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 7,
             xt_proto.CMD_SERVO, xt_proto.SERVO_CTRL_POSITION, 0, 0,
             xt_proto.POSITION_NAN_MRAD, 12_345, -678, 0,
             xt_proto.POSITION_NAN_MRAD, 0, 0,
             xt_proto.POSITION_NAN_MRAD, xt_proto.POSITION_NAN_MRAD,
             1000, 1000, 1000, 0),
        )

    def test_pack_servo_position_mode(self):
        frame = xt_proto.pack_servo(
            1.5, 0.25, seq=3, position_rad=0.5, stop_position_rad=1.0,
            max_torque_nm=0.4, feedforward_nm=0.01,
            velocity_limit_rad_s=30.0, accel_limit_rad_s2=80.0,
            kp_scale=1.5, kd_scale=0.5, ilimit_scale=0.0,
        )
        self.assertEqual(
            struct.unpack(self._FMT, frame),
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 3,
             xt_proto.CMD_SERVO, xt_proto.SERVO_CTRL_POSITION, 0, 0,
             500, 1500, 250, 0,
             1000, 400, 10,
             30000, 80000,
             1500, 500, 0, 0),
        )

    def test_pack_current_mode(self):
        frame = xt_proto.pack_current(0.5, -1.25, seq=9)
        self.assertEqual(
            struct.unpack(self._FMT, frame),
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 9,
             xt_proto.CMD_SERVO, xt_proto.SERVO_CTRL_CURRENT, 0, 0,
             0, 0, 500, -1250,
             xt_proto.POSITION_NAN_MRAD, 0, 0,
             xt_proto.POSITION_NAN_MRAD, xt_proto.POSITION_NAN_MRAD,
             1000, 1000, 1000, 0),
        )

    def test_pack_mit_mode(self):
        frame = xt_proto.pack_mit(
            position_rad=12.5,
            velocity_rad_s=-1.25,
            kp=3.5,
            kd=0.2,
            feedforward_nm=0.05,
            max_torque_nm=0.8,
            seq=4,
        )
        self.assertEqual(
            struct.unpack(self._FMT, frame),
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 4,
             xt_proto.CMD_SERVO, xt_proto.SERVO_CTRL_MIT, 0, 0,
             12500, -1250, 3500, 200,
             xt_proto.POSITION_NAN_MRAD, 800, 50,
             xt_proto.POSITION_NAN_MRAD, xt_proto.POSITION_NAN_MRAD,
             1000, 1000, 1000, 0),
        )


class GuiControlStreamTest(unittest.TestCase):
    _FMT = "<BBBBBBBHiiiiiiiiiHHHH"

    def test_servo_stream_packs_a_servo_command(self):
        bridge = object.__new__(CanBridge)
        frame = bridge._pack_stream_frame(
            "servo", {"omega_mech": 12.345, "id": -0.678}, seq=7
        )
        self.assertEqual(
            struct.unpack(self._FMT, frame),
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 7,
             xt_proto.CMD_SERVO, xt_proto.SERVO_CTRL_POSITION, 0, 0,
             xt_proto.POSITION_NAN_MRAD, 12_345, -678, 0,
             xt_proto.POSITION_NAN_MRAD, 0, 0,
             xt_proto.POSITION_NAN_MRAD, xt_proto.POSITION_NAN_MRAD,
             1000, 1000, 1000, 0),
        )

    def test_current_stream_packs_a_current_command(self):
        bridge = object.__new__(CanBridge)
        frame = bridge._pack_stream_frame(
            "current", {"id": 0.5, "iq": -1.25}, seq=8
        )
        self.assertEqual(
            struct.unpack(self._FMT, frame),
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 8,
             xt_proto.CMD_SERVO, xt_proto.SERVO_CTRL_CURRENT, 0, 0,
             0, 0, 500, -1250,
             xt_proto.POSITION_NAN_MRAD, 0, 0,
             xt_proto.POSITION_NAN_MRAD, xt_proto.POSITION_NAN_MRAD,
             1000, 1000, 1000, 0),
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



class ConfProtocolTest(unittest.TestCase):
    def test_pack_conf_get_layout(self):
        frame = xt_proto.pack_conf_get(xt_proto.CONF_GROUP_MOTOR, seq=3)
        self.assertEqual(len(frame), 9)
        self.assertEqual(
            struct.unpack("<BBBBBBBH", frame),
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 3,
             xt_proto.CMD_CONF, xt_proto.CONF_OP_GET,
             xt_proto.CONF_GROUP_MOTOR, 0),
        )

    def test_pack_conf_set_appends_payload(self):
        payload = xt_proto.pack_foc_conf(200.0, 1.0, 1.0, 1.0, 10000.0)
        frame = xt_proto.pack_conf_set(
            xt_proto.CONF_GROUP_FOC, payload, seq=5
        )
        self.assertEqual(len(frame), 9 + 24)
        hdr = struct.unpack_from("<BBBBBBBH", frame, 0)
        self.assertEqual(
            hdr,
            (xt_proto.MAGIC, xt_proto.VERSION, xt_proto.TYPE_CMD, 5,
             xt_proto.CMD_CONF, xt_proto.CONF_OP_SET,
             xt_proto.CONF_GROUP_FOC, 0),
        )
        self.assertEqual(frame[9:], payload)

    def test_pack_conf_save_load_defaults(self):
        save = xt_proto.pack_conf_save(seq=1)
        load = xt_proto.pack_conf_load(xt_proto.CONF_GROUP_SERVO, seq=2)
        defaults = xt_proto.pack_conf_defaults(seq=3)
        self.assertEqual(
            struct.unpack("<BBBBBBBH", save)[4:],
            (xt_proto.CMD_CONF, xt_proto.CONF_OP_SAVE,
             xt_proto.CONF_GROUP_ALL, 0),
        )
        self.assertEqual(
            struct.unpack("<BBBBBBBH", load)[4:],
            (xt_proto.CMD_CONF, xt_proto.CONF_OP_LOAD,
             xt_proto.CONF_GROUP_SERVO, 0),
        )
        self.assertEqual(
            struct.unpack("<BBBBBBBH", defaults)[4:],
            (xt_proto.CMD_CONF, xt_proto.CONF_OP_DEFAULTS,
             xt_proto.CONF_GROUP_ALL, 0),
        )

    def test_motor_conf_roundtrip(self):
        raw = xt_proto.pack_motor_conf(
            pole_pairs=14.0,
            resistance_ohm=0.65,
            inductance_H=340e-6,
            bemf_Vpeak_per_krpm=11.5,
            max_phase_current_A=4.9,
            fw_speed_rad_s=200.0,
            bus_V=48.0,
        )
        self.assertEqual(len(raw), 32)
        decoded = xt_proto.unpack_motor_conf(raw)
        self.assertEqual(decoded["pole_pairs"], 14.0)
        self.assertAlmostEqual(decoded["resistance_ohm"], 0.65)
        self.assertAlmostEqual(decoded["inductance_H"], 340e-6)
        self.assertEqual(decoded["bus_V"], 48.0)

    def test_foc_conf_roundtrip(self):
        raw = xt_proto.pack_foc_conf(180.0, 0.9, 0.8, 0.7, 5000.0)
        self.assertEqual(len(raw), 24)
        decoded = xt_proto.unpack_foc_conf(raw)
        self.assertEqual(decoded["bandwidth_hz"], 180.0)
        self.assertEqual(decoded["max_current_desired_rate_A_s"], 5000.0)

    def test_servo_conf_roundtrip_nan_accel_and_sign(self):
        raw = xt_proto.pack_servo_conf(
            kp=4.0,
            ki=0.0,
            kd=0.05,
            ilimit=0.0,
            max_iq_A=3.0,
            velocity_threshold=0.5,
            max_position_slip_rad=3.141592653589793,
            max_velocity_error_rad_s=0.0,
            default_velocity_limit_rad_s=200.0,
            default_accel_limit_rad_s2=None,  # unlimited => NaN
            sign_f=-1.0,
        )
        self.assertEqual(len(raw), 48)
        decoded = xt_proto.unpack_servo_conf(raw)
        self.assertEqual(decoded["kp"], 4.0)
        self.assertEqual(decoded["sign_f"], -1.0)
        self.assertTrue(math.isnan(decoded["default_accel_limit_rad_s2"]))

        raw2 = xt_proto.pack_servo_conf(
            kp=1.0, ki=0.0, kd=0.0, ilimit=0.0, max_iq_A=1.0,
            velocity_threshold=0.0, max_position_slip_rad=0.0,
            max_velocity_error_rad_s=0.0,
            default_velocity_limit_rad_s=10.0,
            default_accel_limit_rad_s2=50.0,
            sign_f=1.0,
        )
        decoded2 = xt_proto.unpack_servo_conf(raw2)
        self.assertEqual(decoded2["default_accel_limit_rad_s2"], 50.0)
        self.assertEqual(decoded2["sign_f"], 1.0)

    def test_encoder_conf_roundtrip(self):
        raw = xt_proto.pack_encoder_conf(400.0, 0.15, 160.0)
        self.assertEqual(len(raw), 16)
        decoded = xt_proto.unpack_encoder_conf(raw)
        self.assertEqual(decoded["pll_filter_hz"], 400.0)
        self.assertAlmostEqual(decoded["spike_error_rad"], 0.15)
        self.assertEqual(decoded["filter_us"], 160.0)

    def test_parse_conf_reply(self):
        payload = xt_proto.pack_motor_conf(
            14.0, 0.65, 340e-6, 11.5, 4.9, 200.0, 48.0
        )
        frame = struct.pack(
            "<BBBBBBH",
            xt_proto.MAGIC,
            xt_proto.VERSION,
            xt_proto.TYPE_CONF,
            11,
            xt_proto.CONF_OP_GET,
            xt_proto.CONF_GROUP_MOTOR,
            xt_proto.CONF_FLAG_FLASH_VALID,
        ) + payload
        result = xt_proto.parse_frame(frame)
        self.assertIsInstance(result, xt_proto.ConfReply)
        self.assertEqual(result.seq, 11)
        self.assertEqual(result.op, xt_proto.CONF_OP_GET)
        self.assertEqual(result.group, xt_proto.CONF_GROUP_MOTOR)
        self.assertTrue(result.flash_valid)
        self.assertEqual(result.payload, payload)
        decoded = xt_proto.unpack_motor_conf(result.payload)
        self.assertEqual(decoded["pole_pairs"], 14.0)

    def test_conf_set_roundtrip_through_reply(self):
        payload = xt_proto.pack_encoder_conf(350.0, 0.2, 120.0)
        req = xt_proto.pack_conf_set(
            xt_proto.CONF_GROUP_ENCODER, payload, seq=9
        )
        # Device echo-style TYPE_CONF reply with same payload.
        reply = (
            xt_proto.pack_header(xt_proto.TYPE_CONF, 9)
            + struct.pack(
                "<BBH",
                xt_proto.CONF_OP_SET,
                xt_proto.CONF_GROUP_ENCODER,
                0,
            )
            + payload
        )
        parsed = xt_proto.parse_frame(reply)
        self.assertEqual(parsed.payload, req[9:])
        self.assertEqual(
            xt_proto.unpack_encoder_conf(parsed.payload)["pll_filter_hz"],
            350.0,
        )




    def test_cal_status_conf_roundtrip(self):
        payload = xt_proto.pack_cal_status_conf(
            flags=(xt_proto.CAL_FLAG_ENCODER | xt_proto.CAL_FLAG_RESISTANCE),
            resistance_ohm=0.65,
            inductance_d_H=None,
            inductance_q_H=None,
            bemf_v_per_hz=0.01,
        )
        self.assertEqual(len(payload), 20)
        decoded = xt_proto.unpack_cal_status_conf(payload)
        self.assertTrue(decoded["encoder"])
        self.assertTrue(decoded["resistance"])
        self.assertFalse(decoded["inductance"])
        self.assertAlmostEqual(decoded["R"], 0.65, places=5)
        self.assertIsNone(decoded["Ld"])
        self.assertAlmostEqual(decoded["Ke"], 0.01, places=5)
        gui = xt_proto.unpack_conf_fields("cal", payload)
        self.assertTrue(gui["encoder"])


if __name__ == "__main__":
    unittest.main()

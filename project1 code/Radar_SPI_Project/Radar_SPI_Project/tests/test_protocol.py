import math
import struct
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from radar_spi_protocol import crc8, encode_command, decode_telemetry, validate
from uno3_spi_radar_gui import parse_object, parse_status


class ProtocolTests(unittest.TestCase):
    def test_crc_known_check(self):
        self.assertEqual(crc8(b"123456789"), 0xF4)  # CRC-8/ATM known check

    def test_command(self):
        frame = encode_command(19, 3, 2)
        self.assertEqual(len(frame), 48)
        self.assertEqual(frame[:6], bytes([0xA5, 1, 19, 3, 0, 2]))
        self.assertEqual(frame[6:47], bytes(41))
        validate(frame)
        tampered = bytearray(frame)
        tampered[5] ^= 1
        with self.assertRaises(ValueError): validate(tampered)

    def test_cylindrical_roundtrip(self):
        frame = bytearray(encode_command(20, 2))
        frame[4] = 0x20
        frame[5] = 1
        frame[42:47] = bytes((2, 1, 4, 17, 0))
        struct.pack_into("<BBHHhhBB", frame, 6, 7, 0x03, 300, 9000, 15, -20, 230, 0)
        frame[47] = crc8(frame[:47])
        data = decode_telemetry(frame)
        self.assertTrue(data.cylindrical_wire)
        self.assertEqual(data.objects[0].id, 7)
        self.assertAlmostEqual(data.objects[0].x_mm, 500, delta=0.5)
        self.assertAlmostEqual(data.objects[0].y_mm, 800, delta=0.5)
        self.assertAlmostEqual(data.objects[0].vx_mm_s, 20, delta=0.1)
        self.assertAlmostEqual(data.objects[0].vy_mm_s, 15, delta=0.1)

    def test_cartesian_roundtrip(self):
        frame = bytearray(encode_command(23, 2))
        frame[4] = 7
        frame[5] = 1
        struct.pack_into("<BBhhhhBB", frame, 6, 8, 3, 800, 500, -10, 5, 150, 1)
        frame[47] = crc8(frame[:47])
        data = decode_telemetry(frame)
        self.assertFalse(data.cylindrical_wire)
        o = data.objects[0]
        self.assertEqual((o.x_mm, o.y_mm, o.r_mm, o.theta_deg), (800, 500, 300, 0))
        self.assertEqual((o.vr_mm_s, o.vt_mm_s), (-10, 5))

    def test_gui_serial_parsers(self):
        o = parse_object("OBJECT|2|800|500|300|0|-10|5|-10|5|150|1|7".split("|"))
        self.assertEqual(o["id"], 2)
        self.assertEqual(o["theta"], 0)
        status = "STATUS|200|2|2|1|3000|0|1|488|1|0|2|1|1|12|0|7|0|0|320|30|1|0"
        self.assertEqual(parse_status(status.split("|"))["ack"], 7)


if __name__ == "__main__":
    unittest.main()

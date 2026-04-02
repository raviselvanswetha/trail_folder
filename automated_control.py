import struct
import socket
import time

IP = "192.168.178.132"
PORT = 1030

DST_SYS = 0x04
DST_COMP = 0x01
SRC_SYS = 0x01
SRC_COMP = 0x01
MSG_ID_PTZ_ANGLE = 0x000012

PITCH_MAX_UP = 30.0
PITCH_MAX_DOWN = -90.0
YAW_MAX_RIGHT = 150.0
YAW_MAX_LEFT = -150.0


def crc16_x25(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def clamp_pitch(pitch_deg: float) -> float:
    return max(PITCH_MAX_DOWN, min(PITCH_MAX_UP, pitch_deg))


def clamp_yaw(yaw_deg: float) -> float:
    return max(YAW_MAX_LEFT, min(YAW_MAX_RIGHT, yaw_deg))


def dp50t_angle_payload(pitch_deg: float, yaw_deg: float) -> bytes:
    pitch_deg = clamp_pitch(pitch_deg)
    yaw_deg = clamp_yaw(yaw_deg)

    if pitch_deg == 0:
        pitch_dir = 0x02   # no motion
        pitch_val = 0
    else:
        pitch_dir = 0x01 if pitch_deg < 0 else 0x00   # down/up
        pitch_val = int(round(abs(pitch_deg) * 100))

    if yaw_deg == 0:
        yaw_dir = 0x02     # no motion
        yaw_val = 0
    else:
        yaw_dir = 0x01 if yaw_deg > 0 else 0x00       # right/left
        yaw_val = int(round(abs(yaw_deg) * 100))

    return struct.pack("<BHBHB", pitch_dir, pitch_val, yaw_dir, yaw_val, 0x00)


def build_blst_frame(dst_sys: int, dst_comp: int, msg_id: int, payload: bytes, seq: int) -> bytes:
    body = struct.pack(
        "<BBBBBB",
        len(payload),
        dst_sys,
        dst_comp,
        seq & 0xFF,
        SRC_SYS,
        SRC_COMP,
    )
    body += struct.pack("<I", msg_id)[:3]   # 3-byte LE msg_id
    body += payload
    crc = crc16_x25(body)
    return b"\xFD" + body + struct.pack("<H", crc)


class DP50TController:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_angle(self, pitch_deg: float, yaw_deg: float):
        payload = dp50t_angle_payload(pitch_deg, yaw_deg)
        frame = build_blst_frame(DST_SYS, DST_COMP, MSG_ID_PTZ_ANGLE, payload, self.seq)
        self.sock.sendto(frame, (self.ip, self.port))
        print(f"SEQ={self.seq:03d} pitch={pitch_deg:6.1f} yaw={yaw_deg:6.1f} payload={payload.hex(' ')}")
        self.seq = (self.seq + 1) & 0xFF

    def close(self):
        self.sock.close()


def frange(start: float, stop: float, step: float):
    if step == 0:
        raise ValueError("step cannot be 0")
    x = start
    if step > 0:
        while x <= stop + 1e-9:
            yield round(x, 6)
            x += step
    else:
        while x >= stop - 1e-9:
            yield round(x, 6)
            x += step


def pitch_then_yaw_sequence(cam: DP50TController, step=10.0, delay_s=0.5):
    # 1) pitch +30 -> -90 with yaw fixed at 0
    print("Pitch sweep: +30 -> -90, yaw = 0")
    for pitch in frange(30.0, -90.0, -abs(step)):
        cam.send_angle(pitch, 0.0)
        time.sleep(delay_s)

    # 2) return pitch to neutral (0), yaw still 0
    print("Return pitch to neutral: pitch = 0, yaw = 0")
    cam.send_angle(0.0, 0.0)
    time.sleep(1.0)

    # 3) yaw +150 -> -150 with pitch fixed at 0
    print("Yaw sweep: +150 -> -150, pitch = 0")
    for yaw in frange(150.0, -150.0, -abs(step)):
        cam.send_angle(0.0, yaw)
        time.sleep(delay_s)

    # optional: return yaw to neutral too
    print("Return yaw to neutral: pitch = 0, yaw = 0")
    cam.send_angle(0.0, 0.0)


if __name__ == "__main__":
    cam = DP50TController(IP, PORT)
    try:
        pitch_then_yaw_sequence(cam, step=10.0, delay_s=0.5)
    finally:
        cam.close()
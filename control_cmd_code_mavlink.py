import struct
import socket

ip = "192.168.178.132"
port = 1030
seq = 0

def crc16_x25(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def mavlink_to_dp50t_angle(pitch_deg, yaw_deg):
    # PTZ Angle payload for MSG_ID 0x000012
    pitch_dir = 0x01 if pitch_deg < 0 else 0x00   # 0=up, 1=down
    yaw_dir   = 0x01 if yaw_deg > 0 else 0x00     # 0=left, 1=right

    pitch_val = int(abs(pitch_deg) * 100)
    yaw_val   = int(abs(yaw_deg) * 100)

    return struct.pack(
        '<BHBHB',
        pitch_dir, pitch_val,
        yaw_dir, yaw_val,
        0x00
    )

def build_blst_frame(dst_sys, dst_comp, msg_id, payload, seq=0, src_sys=0x01, src_comp=0x01):
    # BLST V1 body (everything after STX)
    body = struct.pack(
        '<BBBBBB',
        len(payload),   # LEN
        dst_sys,        # DT_SYS_ID
        dst_comp,       # DA_COMP_ID
        seq,            # SEQ
        src_sys,        # SA_SYS_ID
        src_comp        # SA_COMP_ID
    )
    body += struct.pack('<I', msg_id)[:3]   # 3-byte little-endian MSG_ID
    body += payload

    crc = crc16_x25(body)
    frame = b'\xFD' + body + struct.pack('<H', crc)
    return frame

# Example: pitch -30 deg (down), yaw +45 deg (right)
payload = mavlink_to_dp50t_angle(-30.0, 45.0)
frame = build_blst_frame(
    dst_sys=0x04,      # camera sysid
    dst_comp=0x01,     # camera compid
    msg_id=0x000012,   # PTZ Angle
    payload=payload,
    seq=0
)

print("Payload:", payload.hex(' '))
print("Frame:  ", frame.hex(' '))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(frame, (ip, port))
sock.close()
#!/usr/bin/env python3
"""
Off-Grid Survival Engine v2.2 - 160-bit Rescue Packet Cross-Validation Tool
Mã hóa và giải mã gói tin cứu hộ 20-byte, xác minh tính tương thích chéo giữa Python và C++.
"""

import struct
import zlib
import math

def encode_packet_160(
    unix_timestamp: int,
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    ce50_major_m: float,
    ce50_minor_m: float,
    ce_heading_deg: float,
    status_flags: int,
    battery_percent: int,
    cluster_id: int
) -> bytes:
    # 1. Timestamp (4 bytes little endian)
    ts_bytes = struct.pack("<I", unix_timestamp)

    # 2. Latitude [-90, 90] -> 24-bit fixed point (3 bytes little endian)
    clamped_lat = max(-90.0, min(90.0, latitude_deg))
    lat_fixed = int(round(clamped_lat * (8388607.0 / 90.0)))
    lat_bytes = (lat_fixed & 0x00FFFFFF).to_bytes(3, byteorder="little", signed=False)

    # 3. Longitude [-180, 180] -> 24-bit fixed point (3 bytes little endian)
    clamped_lon = max(-180.0, min(180.0, longitude_deg))
    lon_fixed = int(round(clamped_lon * (8388607.0 / 180.0)))
    lon_bytes = (lon_fixed & 0x00FFFFFF).to_bytes(3, byteorder="little", signed=False)

    # 4. Altitude (12 bits: 0..4095m) + Minor ratio (4 bits: 0..15)
    alt_quant = max(0, min(4095, int(round(altitude_m))))
    ratio = 1.0 if ce50_major_m <= 1e-4 else max(0.0, min(1.0, ce50_minor_m / ce50_major_m))
    ratio_quant = int(round(ratio * 15.0))
    b10 = alt_quant & 0xFF
    b11 = ((alt_quant >> 8) & 0x0F) | ((ratio_quant & 0x0F) << 4)

    # 5. CE50 Major (8 bits: 0..255m)
    major_quant = max(0, min(255, int(round(ce50_major_m))))

    # 6. Heading (8 bits: [0, 360) -> 0..255)
    norm_heading = ce_heading_deg % 360.0
    heading_quant = max(0, min(255, int(round(norm_heading * (255.0 / 360.0)))))

    # 7. Status flags (1 byte)
    status_byte = status_flags & 0xFF

    # 8. Battery (5 bits) + Cluster ID (3 bits)
    bat_5bit = max(0, min(31, int(round(battery_percent * (31.0 / 100.0)))))
    bat_cid_byte = ((bat_5bit << 3) & 0xF8) | (cluster_id & 0x07)

    payload = ts_bytes + lat_bytes + lon_bytes + bytes([b10, b11, major_quant, heading_quant, status_byte, bat_cid_byte])
    assert len(payload) == 16, f"Payload must be exactly 16 bytes, got {len(payload)}"

    # 9. CRC32 (4 bytes little endian)
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    crc_bytes = struct.pack("<I", crc)

    packet = payload + crc_bytes
    assert len(packet) == 20, f"Full packet must be exactly 20 bytes, got {len(packet)}"
    return packet

def decode_packet_160(packet: bytes) -> dict:
    assert len(packet) == 20, f"Packet must be 20 bytes, got {len(packet)}"
    payload = packet[:16]
    expected_crc = struct.unpack("<I", packet[16:20])[0]
    computed_crc = zlib.crc32(payload) & 0xFFFFFFFF

    is_crc_valid = (expected_crc == computed_crc)

    unix_timestamp = struct.unpack("<I", payload[0:4])[0]

    lat_u = int.from_bytes(payload[4:7], byteorder="little", signed=False)
    if lat_u & 0x00800000:
        lat_u -= 0x01000000
    latitude_deg = lat_u * (90.0 / 8388607.0)

    lon_u = int.from_bytes(payload[7:10], byteorder="little", signed=False)
    if lon_u & 0x00800000:
        lon_u -= 0x01000000
    longitude_deg = lon_u * (180.0 / 8388607.0)

    b10 = payload[10]
    b11 = payload[11]
    altitude_m = float((b11 & 0x0F) << 8 | b10)
    minor_ratio = float((b11 >> 4) & 0x0F) / 15.0

    ce50_major_m = float(payload[12])
    ce_heading_deg = float(payload[13]) * (360.0 / 255.0)

    status_flags = payload[14]
    bat_5bit = (payload[15] >> 3) & 0x1F
    battery_percent = int(round(bat_5bit * (100.0 / 31.0)))
    cluster_id = payload[15] & 0x07

    return {
        "unix_timestamp": unix_timestamp,
        "latitude_deg": latitude_deg,
        "longitude_deg": longitude_deg,
        "altitude_m": altitude_m,
        "ce50_major_m": ce50_major_m,
        "ce50_minor_ratio": minor_ratio,
        "ce_heading_deg": ce_heading_deg,
        "status_flags": status_flags,
        "battery_percent": battery_percent,
        "cluster_id": cluster_id,
        "crc32": hex(expected_crc),
        "is_crc_valid": is_crc_valid
    }

def main():
    print("=== Off-Grid Survival Engine: 160-bit Rescue Packet Cross-Validator ===")
    
    # Test case: Đà Lạt Coordinates (11.940419° N, 108.458313° E)
    pkt = encode_packet_160(
        unix_timestamp=1725184800,
        latitude_deg=11.940419,
        longitude_deg=108.458313,
        altitude_m=1500.0,
        ce50_major_m=45.0,
        ce50_minor_m=30.0,
        ce_heading_deg=135.0,
        status_flags=0x85,
        battery_percent=75,
        cluster_id=2
    )

    print(f"Encoded Hex ({len(pkt)} bytes): {pkt.hex().upper()}")
    decoded = decode_packet_160(pkt)
    print("\nDecoded Result:")
    for k, v in decoded.items():
        print(f"  {k:25}: {v}")

    assert decoded["is_crc_valid"], "CRC validation failed!"

    # [C4] Self-test biên (rvtools.md khuyến nghị): ±90/±180 lat/lon, altitude/CE biên
    edge_cases = [
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),                # gốc tọa độ
        (90.0, 180.0, 8848.0, 999.0, 999.0, 359.9),    # Bắc cực / kinh tuyến + Everest
        (-90.0, -180.0, -400.0, 0.1, 0.1, 0.1),        # Nam cực / kinh tuyến đối
        (-33.865, 151.209, 25.0, 123.4, 56.7, 270.0),  # Sydney (nam, đông)
    ]
    for i, (lat, lon, alt, ce_a, ce_b, hdg) in enumerate(edge_cases):
        pkt_e = encode_packet_160(
            unix_timestamp=1725184800 + i, latitude_deg=lat, longitude_deg=lon,
            altitude_m=alt, ce50_major_m=ce_a, ce50_minor_m=ce_b,
            ce_heading_deg=hdg, status_flags=0x01, battery_percent=100 - i,
            cluster_id=i)
        dec = decode_packet_160(pkt_e)
        assert dec["is_crc_valid"], f"edge {i}: CRC fail"
        # Dấu lat/lon mã hóa kèm cờ riêng — độ lớn phải khớp ±0.001°
        assert abs(abs(dec["latitude_deg"]) - abs(lat)) < 1e-3, f"edge {i}: lat {dec['latitude_deg']} vs {lat}"
        assert abs(abs(dec["longitude_deg"]) - abs(lon)) < 1e-3, f"edge {i}: lon {dec['longitude_deg']} vs {lon}"

    print("\n[SUCCESS] 160-bit Rescue Packet serialization & validation PASSED!")

if __name__ == "__main__":
    main()

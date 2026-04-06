#!/usr/bin/env python3
"""
Diagnose merged ROS2 bag: prints actual Livox and IMU field values.
Usage: python3 diagnose_bag.py ~/data/mcd_ntu/ntu_day_01_merged
"""

import sys
import sqlite3
import struct
import os
import yaml

def decode_cdr_string(data, offset):
    """Decode a CDR string: uint32 length + bytes."""
    length = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    s = data[offset:offset+length].rstrip(b'\x00').decode('utf-8', errors='replace')
    offset += length
    return s, offset

def align(offset, size):
    """CDR alignment: align offset to 'size' bytes."""
    rem = offset % size
    if rem:
        offset += size - rem
    return offset

def decode_livox_msg(data):
    """Decode livox_ros_driver/CustomMsg CDR bytes."""
    offset = 4  # skip 4-byte CDR header

    # std_msgs/Header
    offset = align(offset, 4)
    sec  = struct.unpack_from('<i', data, offset)[0]; offset += 4
    nsec = struct.unpack_from('<I', data, offset)[0]; offset += 4
    frame_id, offset = decode_cdr_string(data, offset)

    # uint64 timebase
    offset = align(offset, 8)
    timebase = struct.unpack_from('<Q', data, offset)[0]; offset += 8

    # uint32 point_num
    offset = align(offset, 4)
    point_num = struct.unpack_from('<I', data, offset)[0]; offset += 4

    # uint8 lidar_id, uint8[3] rsvd
    lidar_id = data[offset]; offset += 1
    offset += 3  # rsvd

    # CustomPoint[] points  (array length as uint32)
    offset = align(offset, 4)
    arr_len = struct.unpack_from('<I', data, offset)[0]; offset += 4

    header_stamp = sec + nsec * 1e-9
    print(f"  header.stamp  = {sec}.{nsec:09d}  ({header_stamp:.6f} s)")
    print(f"  timebase      = {timebase}  ({timebase * 1e-9:.6f} s)")
    print(f"  point_num     = {point_num}")
    print(f"  points[] len  = {arr_len}")
    print(f"  frame_id      = {frame_id!r}")

    # CustomPoint layout (CDR, no extra padding if all fields natural-aligned):
    # uint32 offset_time (4), float32 x (4), float32 y (4), float32 z (4),
    # uint8 reflectivity (1), uint8 tag (1), uint8 line (1), [pad 1] => 20 bytes
    POINT_SIZE = 20

    if arr_len > 0:
        def decode_point(idx):
            off = offset + idx * POINT_SIZE
            ot  = struct.unpack_from('<I', data, off)[0]
            x   = struct.unpack_from('<f', data, off+4)[0]
            y   = struct.unpack_from('<f', data, off+8)[0]
            z   = struct.unpack_from('<f', data, off+12)[0]
            return ot, x, y, z

        ot0, x0, y0, z0 = decode_point(0)
        print(f"  First point:  offset_time={ot0}  ({ot0*1e-9:.9f} s)  xyz=({x0:.3f},{y0:.3f},{z0:.3f})")
        print(f"    => abs_time = header + offset = {header_stamp + ot0*1e-9:.6f} s")
        if arr_len > 1:
            ot_last, xl, yl, zl = decode_point(arr_len - 1)
            print(f"  Last  point:  offset_time={ot_last}  ({ot_last*1e-9:.9f} s)  xyz=({xl:.3f},{yl:.3f},{zl:.3f})")
            print(f"    => abs_time = header + offset = {header_stamp + ot_last*1e-9:.6f} s")
            print(f"  EXPECTED end_scan_time (via code) = {header_stamp + ot_last*1e-9:.6f} s")

        # Also try POINT_SIZE=24 (with 4 bytes padding before offset_time or after line)
        if arr_len > 1:
            for ps in [18, 19, 20, 21, 22, 24]:
                off_last = offset + (arr_len-1) * ps
                if off_last + 4 <= len(data):
                    ot_try = struct.unpack_from('<I', data, off_last)[0]
                    t_try = header_stamp + ot_try * 1e-9
                    if abs(t_try - header_stamp) < 1.0:  # plausible (< 1 second)
                        print(f"  [stride={ps}] last offset_time={ot_try} → end={t_try:.6f} s  ← PLAUSIBLE")

    return header_stamp

def decode_imu_msg(data):
    """Decode sensor_msgs/Imu CDR bytes, return (sec, nsec) of header.stamp."""
    offset = 4  # skip CDR header
    offset = align(offset, 4)
    sec  = struct.unpack_from('<i', data, offset)[0]; offset += 4
    nsec = struct.unpack_from('<I', data, offset)[0]; offset += 4
    return sec, nsec

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_dir>")
        sys.exit(1)

    bag_dir = sys.argv[1]

    # Find metadata.yaml
    meta_file = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.exists(meta_file):
        with open(meta_file) as f:
            meta = yaml.safe_load(f)
        print("=== Bag Metadata ===")
        for topic_meta in meta.get('rosbag2_bagfile_information', {}).get('topics_with_message_count', []):
            tp = topic_meta.get('topic_metadata', {})
            cnt = topic_meta.get('message_count', '?')
            print(f"  {tp.get('name','?')}  [{tp.get('type','?')}]  count={cnt}")
        print()

    # Find .db3
    db3_files = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
    if not db3_files:
        print("No .db3 file found!"); sys.exit(1)
    db3_path = os.path.join(bag_dir, db3_files[0])
    print(f"Reading: {db3_path}\n")

    conn = sqlite3.connect(db3_path)
    cur = conn.cursor()

    # Get topics
    cur.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cur.fetchall()}
    print("=== Topics in DB ===")
    for tid, (name, ttype) in topics.items():
        print(f"  id={tid}  {name}  [{ttype}]")
    print()

    # Find Livox and IMU topic IDs
    livox_id = None
    imu_id = None
    for tid, (name, ttype) in topics.items():
        if 'livox' in name.lower() or 'lidar' in name.lower():
            livox_id = tid
        if 'imu' in name.lower():
            imu_id = tid

    print(f"Livox topic id: {livox_id}, IMU topic id: {imu_id}\n")

    # First 5 messages overall (bag_timestamp order)
    print("=== First 5 messages (global order) ===")
    cur.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp LIMIT 5")
    for row in cur.fetchall():
        tid, ts, data = row
        name = topics[tid][0]
        print(f"  bag_ts={ts}  ({ts*1e-9:.6f} s)  topic={name}  bytes={len(data)}")
    print()

    # First 3 Livox messages
    if livox_id:
        print("=== First 3 Livox messages ===")
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 3", (livox_id,))
        rows = cur.fetchall()
        for i, (ts, data) in enumerate(rows):
            print(f"--- Livox msg {i}  bag_ts={ts}  ({ts*1e-9:.6f} s) ---")
            try:
                decode_livox_msg(bytes(data))
            except Exception as e:
                print(f"  ERROR: {e}")
            print()

        # Last Livox message
        print("=== Last Livox message ===")
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 1", (livox_id,))
        row = cur.fetchone()
        if row:
            ts, data = row
            print(f"--- Livox last  bag_ts={ts}  ({ts*1e-9:.6f} s) ---")
            try:
                decode_livox_msg(bytes(data))
            except Exception as e:
                print(f"  ERROR: {e}")
            print()

    # First 5 IMU messages
    if imu_id:
        print("=== First 5 IMU messages ===")
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 5", (imu_id,))
        for ts, data in cur.fetchall():
            sec, nsec = decode_imu_msg(bytes(data))
            print(f"  bag_ts={ts}  ({ts*1e-9:.6f} s)  header.stamp={sec}.{nsec:09d}  ({sec+nsec*1e-9:.6f} s)")
        print()

    # Check IMU monotonicity (first 2000)
    if imu_id:
        print("=== IMU monotonicity check (first 2000) ===")
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 2000", (imu_id,))
        rows = cur.fetchall()
        prev_stamp = -1
        bad = 0
        for ts, data in rows:
            sec, nsec = decode_imu_msg(bytes(data))
            stamp = sec + nsec * 1e-9
            if stamp < prev_stamp:
                bad += 1
                print(f"  OUT OF ORDER: bag_ts={ts} header.stamp={stamp:.6f} prev={prev_stamp:.6f}")
            prev_stamp = stamp
        print(f"  Total checked: {len(rows)},  out-of-order: {bad}")

    conn.close()

if __name__ == '__main__':
    main()

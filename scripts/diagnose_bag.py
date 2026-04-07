#!/usr/bin/env python3
"""
Diagnose merged ROS2 bag: prints actual Livox and IMU field values.
Tries both max-4 and max-8 byte CDR alignment for uint64.
Usage: python3 diagnose_bag.py ~/data/mcd_ntu/ntu_day_01_merged
"""

import sys
import sqlite3
import struct
import os
import yaml

def decode_cdr_string(data, offset):
    length = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    s = data[offset:offset+length].rstrip(b'\x00').decode('utf-8', errors='replace')
    offset += length
    return s, offset

def align(offset, size):
    rem = offset % size
    if rem:
        offset += size - rem
    return offset

def decode_livox_msg(data, max_align=8):
    """
    Decode livox_ros_driver/CustomMsg CDR bytes.
    max_align: maximum alignment boundary (4 or 8).
    """
    offset = 4  # skip 4-byte CDR header

    # std_msgs/Header (ROS2: no seq)
    sec  = struct.unpack_from('<i', data, offset)[0]; offset += 4
    nsec = struct.unpack_from('<I', data, offset)[0]; offset += 4
    frame_id, offset = decode_cdr_string(data, offset)

    # uint64 timebase — align to min(8, max_align)
    u64_align = min(8, max_align)
    offset = align(offset, u64_align)
    timebase = struct.unpack_from('<Q', data, offset)[0]; offset += 8

    # uint32 point_num
    offset = align(offset, 4)
    point_num = struct.unpack_from('<I', data, offset)[0]; offset += 4

    # uint8 lidar_id, uint8[3] rsvd
    lidar_id = data[offset]; offset += 1
    offset += 3

    # CustomPoint[] array length
    offset = align(offset, 4)
    arr_len = struct.unpack_from('<I', data, offset)[0]; offset += 4

    header_stamp = sec + nsec * 1e-9
    return {
        'header_stamp': header_stamp,
        'sec': sec, 'nsec': nsec,
        'timebase': timebase,
        'point_num': point_num,
        'arr_len': arr_len,
        'frame_id': frame_id,
        'data_offset': offset,
    }

def decode_points(data, offset, arr_len, stride=20):
    """Decode first and last CustomPoint from CDR bytes."""
    results = {}
    if arr_len == 0:
        return results
    # First point
    off = offset
    if off + stride <= len(data):
        ot = struct.unpack_from('<I', data, off)[0]
        x  = struct.unpack_from('<f', data, off+4)[0]
        y  = struct.unpack_from('<f', data, off+8)[0]
        z  = struct.unpack_from('<f', data, off+12)[0]
        results['first'] = (ot, x, y, z)
    # Last point
    off = offset + (arr_len - 1) * stride
    if off + stride <= len(data):
        ot = struct.unpack_from('<I', data, off)[0]
        x  = struct.unpack_from('<f', data, off+4)[0]
        y  = struct.unpack_from('<f', data, off+8)[0]
        z  = struct.unpack_from('<f', data, off+12)[0]
        results['last'] = (ot, x, y, z)
    return results

def hexdump(data, offset, length=64):
    chunk = data[offset:offset+length]
    hex_str = ' '.join(f'{b:02x}' for b in chunk)
    return hex_str

def decode_imu_msg(data):
    offset = 4
    sec  = struct.unpack_from('<i', data, offset)[0]; offset += 4
    nsec = struct.unpack_from('<I', data, offset)[0]; offset += 4
    return sec, nsec

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_dir>")
        sys.exit(1)

    bag_dir = sys.argv[1]

    meta_file = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.exists(meta_file):
        with open(meta_file) as f:
            meta = yaml.safe_load(f)
        print("=== Bag Metadata ===")
        for tm in meta.get('rosbag2_bagfile_information', {}).get('topics_with_message_count', []):
            tp = tm.get('topic_metadata', {})
            cnt = tm.get('message_count', '?')
            print(f"  {tp.get('name','?')}  [{tp.get('type','?')}]  count={cnt}")
        print()

    db3_files = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
    if not db3_files:
        print("No .db3 file found!"); sys.exit(1)
    db3_path = os.path.join(bag_dir, db3_files[0])
    print(f"Reading: {db3_path}\n")

    conn_db = sqlite3.connect(db3_path)
    cur = conn_db.cursor()

    cur.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cur.fetchall()}

    livox_id = None; imu_id = None
    for tid, (name, ttype) in topics.items():
        if 'livox' in name.lower() or 'lidar' in name.lower(): livox_id = tid
        if name == '/vn100/imu': imu_id = tid

    # ── Livox deep inspection ──────────────────────────────────────────────
    if livox_id:
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1", (livox_id,))
        row = cur.fetchone()
        if row:
            ts, raw = row
            data = bytes(raw)
            print(f"=== Livox message (bag_ts={ts}, {ts*1e-9:.3f} s) ===")
            print(f"  Total bytes: {len(data)}")
            print(f"  CDR header bytes: {hexdump(data, 0, 4)}")
            print()

            # Try both alignment strategies
            for max_align in [4, 8]:
                try:
                    r = decode_livox_msg(data, max_align=max_align)
                    pts = decode_points(data, r['data_offset'], r['arr_len'])
                    print(f"  [max_align={max_align}]")
                    print(f"    header.stamp = {r['sec']}.{r['nsec']:09d}  ({r['header_stamp']:.6f} s)")
                    print(f"    timebase     = {r['timebase']}  ({r['timebase']*1e-9:.3f} s)")
                    print(f"    point_num    = {r['point_num']}")
                    print(f"    arr_len      = {r['arr_len']}")
                    print(f"    frame_id     = {r['frame_id']!r}")
                    if 'first' in pts:
                        ot, x, y, z = pts['first']
                        print(f"    first point: offset_time={ot} ns ({ot*1e-9:.6f}s)  xyz=({x:.2f},{y:.2f},{z:.2f})")
                        print(f"      => abs_time = {r['header_stamp'] + ot*1e-9:.6f} s")
                    if 'last' in pts:
                        ot, x, y, z = pts['last']
                        print(f"    last  point: offset_time={ot} ns ({ot*1e-9:.6f}s)  xyz=({x:.2f},{y:.2f},{z:.2f})")
                        print(f"      => abs_time = {r['header_stamp'] + ot*1e-9:.6f} s  ← expected end_stamp")
                    print()
                except Exception as e:
                    print(f"  [max_align={max_align}] ERROR: {e}\n")

            # Raw hex around the key area (after frame_id at offset ~28)
            print(f"  Raw hex [offset 20..80]:")
            print(f"    {hexdump(data, 20, 60)}")
            print(f"  Offset annotations (frame_id ends ~28):")
            for off in range(24, min(60, len(data)), 4):
                val_u32 = struct.unpack_from('<I', data, off)[0]
                val_u64 = struct.unpack_from('<Q', data, off)[0] if off+8 <= len(data) else None
                print(f"    [{off:3d}] uint32={val_u32:>12}  uint64={val_u64}")
            print()

        # Count messages with non-zero points
        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (livox_id,))
        total = cur.fetchone()[0]
        print(f"  Total Livox messages: {total}")
        print()

    # ── IMU check ─────────────────────────────────────────────────────────
    if imu_id:
        print("=== First 5 IMU messages ===")
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 5", (imu_id,))
        for ts, raw in cur.fetchall():
            sec, nsec = decode_imu_msg(bytes(raw))
            print(f"  bag_ts={ts*1e-9:.6f}s  header.stamp={sec+nsec*1e-9:.6f}s")

        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (imu_id,))
        print(f"  Total IMU messages: {cur.fetchone()[0]}\n")

    conn_db.close()

if __name__ == '__main__':
    main()

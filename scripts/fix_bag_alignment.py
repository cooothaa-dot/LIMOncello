#!/usr/bin/env python3
"""
Fix CDR alignment in a ROS2 bag written by rosbags library.

rosbags writes uint64 with max-4-byte alignment, but FastDDS (ROS2 runtime)
expects max-8-byte alignment.  For livox_ros_driver/msg/CustomMsg the uint64
`timebase` field ends up at offset 28 instead of 32, shifting all subsequent
fields (point_num, points[]) by 4 bytes → LIMOncello sees 0 points per scan.

Fix: insert 4 zero-padding bytes right after the Header.frame_id string in
every Livox message so that `timebase` lands on an 8-byte boundary.

Usage:
    python3 fix_bag_alignment.py <bag_dir> [<fixed_bag_dir>]

If <fixed_bag_dir> is omitted, a "_fixed" suffix is appended.
"""

import sys, os, sqlite3, struct, shutil

def frame_id_end_offset(data: bytes) -> int:
    """Return the byte offset immediately after Header.frame_id in CDR data."""
    # [0-3]   CDR 4-byte header
    # [4-7]   stamp.sec  (int32)
    # [8-11]  stamp.nanosec (uint32)
    # [12-15] frame_id length (uint32, includes null terminator)
    # [16 ..] frame_id bytes
    fid_len = struct.unpack_from('<I', data, 12)[0]
    return 16 + fid_len          # offset right after the string bytes

def fix_livox_cdr(data: bytes) -> bytes:
    """Insert padding so that uint64 timebase is 8-byte aligned."""
    end = frame_id_end_offset(data)
    padding_needed = (8 - end % 8) % 8
    if padding_needed == 0:
        return data              # already aligned – nothing to do
    fixed = bytearray(data)
    for _ in range(padding_needed):
        fixed.insert(end, 0x00)
    return bytes(fixed)

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_dir> [<fixed_bag_dir>]")
        sys.exit(1)

    src_dir = sys.argv[1].rstrip('/')
    dst_dir = sys.argv[2] if len(sys.argv) > 2 else src_dir + '_fixed'

    if os.path.exists(dst_dir):
        print(f"ERROR: destination already exists: {dst_dir}")
        print("Delete it first or choose a different name.")
        sys.exit(1)

    print(f"Copying bag: {src_dir}  →  {dst_dir}")
    shutil.copytree(src_dir, dst_dir)

    db3_files = [f for f in os.listdir(dst_dir) if f.endswith('.db3')]
    if not db3_files:
        print("No .db3 file found in bag directory!"); sys.exit(1)
    db3_path = os.path.join(dst_dir, db3_files[0])
    print(f"Patching: {db3_path}")

    conn = sqlite3.connect(db3_path)
    cur  = conn.cursor()

    # Find the Livox topic id
    cur.execute("SELECT id, name FROM topics")
    livox_id = None
    for tid, name in cur.fetchall():
        if 'livox' in name.lower() or 'lidar' in name.lower():
            livox_id = tid
            print(f"Found Livox topic: '{name}'  (id={tid})")
            break
    if livox_id is None:
        print("No Livox/LiDAR topic found – nothing to do."); sys.exit(0)

    # Dry-run first message to verify the fix makes sense
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1",
                (livox_id,))
    ts, raw = cur.fetchone()
    orig = bytes(raw)
    end = frame_id_end_offset(orig)
    pad  = (8 - end % 8) % 8
    print(f"\nFirst message: {len(orig)} bytes, frame_id ends at offset {end}, "
          f"padding to insert = {pad} bytes")

    if pad == 0:
        print("Already 8-byte aligned – nothing to fix!")
        conn.close()
        shutil.rmtree(dst_dir)
        sys.exit(0)

    # Verify: point_num before and after
    pn_before = struct.unpack_from('<I', orig, end + 8)[0]   # uint64 + uint32 = +12
    fixed_sample = fix_livox_cdr(orig)
    pn_after  = struct.unpack_from('<I', fixed_sample, end + pad + 8)[0]
    print(f"point_num BEFORE fix = {pn_before}  (should be 0 if misaligned)")
    print(f"point_num AFTER  fix = {pn_after}   (should be ~9984)")
    if pn_after == 0:
        print("WARNING: point_num still 0 after fix – something else may be wrong.")

    # Patch all Livox messages
    cur.execute("SELECT rowid, data FROM messages WHERE topic_id=?", (livox_id,))
    rows = cur.fetchall()
    print(f"\nPatching {len(rows)} messages ...")

    updates = []
    for rowid, raw in rows:
        fixed = fix_livox_cdr(bytes(raw))
        updates.append((fixed, rowid))

    cur.executemany("UPDATE messages SET data=? WHERE rowid=?", updates)
    conn.commit()
    conn.close()

    print(f"\nDone. Fixed bag written to:\n  {dst_dir}")
    print("\nTo verify:")
    print(f"  python3 diagnose_bag.py {dst_dir}")
    print("\nTo use with LIMOncello:")
    print(f"  ros2 bag play {dst_dir} --clock -r 0.5")

if __name__ == '__main__':
    main()

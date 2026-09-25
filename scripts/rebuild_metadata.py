"""Rebuild a rosbag2 metadata.yaml from the sqlite3 db3 itself (recorder was killed before finalizing)."""
import sqlite3
import sys
import os

bag_dir = sys.argv[1]
db3 = [f for f in os.listdir(bag_dir) if f.endswith('.db3')][0]
conn = sqlite3.connect(os.path.join(bag_dir, db3))
cur = conn.cursor()

topics = cur.execute("SELECT id,name,type,serialization_format,offered_qos_profiles FROM topics").fetchall()
stats = {}
tmin, tmax = None, None
for tid, name, typ, fmt, qos in topics:
    rows = cur.execute("SELECT timestamp FROM messages WHERE topic_id=?", (tid,)).fetchall()
    stats[tid] = (name, typ, fmt, qos, len(rows))
    if rows:
        lo, hi = min(r[0] for r in rows), max(r[0] for r in rows)
        tmin = lo if tmin is None else min(tmin, lo)
        tmax = hi if tmax is None else max(tmax, hi)

duration = (tmax - tmin) if (tmin is not None and tmax is not None) else 0
total = sum(s[4] for s in stats.values())

QOS = ("- history: 3\\n  depth: 0\\n  reliability: 1\\n  durability: 2\\n"
       "  deadline:\\n    sec: 9223372036\\n    nsec: 854775807\\n"
       "  lifespan:\\n    sec: 9223372036\\n    nsec: 854775807\\n"
       "  liveliness: 1\\n  liveliness_lease_duration:\\n    sec: 9223372036\\n    nsec: 854775807\\n"
       "  avoid_ros_namespace_conventions: false")

lines = [
    "rosbag2_bagfile_information:",
    "  version: 5",
    "  storage_identifier: sqlite3",
    "  duration:",
    f"    nanoseconds: {duration}",
    "  starting_time:",
    f"    nanoseconds_since_epoch: {tmin if tmin is not None else 0}",
    f"  message_count: {total}",
    "  topics_with_message_count:",
]
for tid, (name, typ, fmt, qos, cnt) in sorted(stats.items()):
    lines += [
        "    - topic_metadata:",
        f"        name: {name}",
        f"        type: {typ}",
        f"        serialization_format: {fmt}",
        f'        offered_qos_profiles: "{QOS}"',
        f"      message_count: {cnt}",
    ]
lines += [
    '  compression_format: ""',
    '  compression_mode: ""',
    "  relative_file_paths:",
    f"    - {db3}",
    "  files:",
    f"    - path: {db3}",
    "      starting_time:",
    f"        nanoseconds_since_epoch: {tmin if tmin is not None else 0}",
    "      duration:",
    f"        nanoseconds: {duration}",
    f"      message_count: {total}",
]

with open(os.path.join(bag_dir, 'metadata.yaml'), 'w') as f:
    f.write('\n'.join(lines) + '\n')

print('rebuilt metadata for', bag_dir)
for tid, (name, typ, fmt, qos, cnt) in sorted(stats.items()):
    print(f'  {name}: {cnt} msgs')

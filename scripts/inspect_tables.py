import json
v = '/mnt/h/datasets/nuscenes-mini/v1.0-mini/'
for f in ['sensor.json', 'calibrated_sensor.json', 'ego_pose.json', 'scene.json']:
    rows = json.load(open(v + f))
    print(f, len(rows), sorted(rows[0].keys()))
    if f == 'sensor.json':
        print(' sensor sample:', json.dumps(rows[0]))
    if f == 'calibrated_sensor.json':
        print(' calib sample:', json.dumps(rows[0])[:300])
    if f == 'scene.json':
        print(' scene sample:', json.dumps(rows[0])[:300])

import json, os
root = '/mnt/h/datasets/nuscenes-mini'
v = root + '/v1.0-mini/'
sd_rows = json.load(open(v + 'sample_data.json'))
calib = json.load(open(v + 'calibrated_sensor.json'))
sensor = {s['token']: s for s in json.load(open(v + 'sensor.json'))}
sample = {s['token']: s for s in json.load(open(v + 'sample.json'))}

lidar_cs = {c['token'] for c in calib if sensor[c['sensor_token']]['channel'] == 'LIDAR_TOP'}
print('lidar calib tokens:', len(lidar_cs))
lidar_sd = [r for r in sd_rows if r['calibrated_sensor_token'] in lidar_cs]
print('lidar sample_data:', len(lidar_sd))
exist = [r for r in lidar_sd if os.path.isfile(os.path.join(root, r['filename']))]
print('existing files:', len(exist))
with_sample = [r for r in exist if r['sample_token'] in sample]
print('with sample:', len(with_sample))
print('example:', exist[0]['filename'] if exist else 'none')

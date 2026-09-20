import csv, math
rows = list(csv.DictReader(open('/root/data/nuscenes_demo_gt.csv')))
pts = [(float(r['x']), float(r['y'])) for r in rows]
total = sum(math.dist(pts[i - 1], pts[i]) for i in range(1, len(pts)))
deltas = [math.dist(pts[i - 1], pts[i]) for i in range(1, len(pts))]
print('frames:', len(pts), 'path len: %.1f m' % total)
print('per-scan delta: min %.2f mean %.2f max %.2f' % (min(deltas), sum(deltas) / len(deltas), max(deltas)))

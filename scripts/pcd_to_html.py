#!/usr/bin/env python3
"""Generate a self-contained HTML 3D point cloud viewer (three.js inlined, clouds base64-embedded).

Usage:
  python3 pcd_to_html.py --out viewer.html --cloud "ICP map|/path/a.pcd|#5ad0ff" --cloud "FAST-LIO|/path/b.pcd|#ff5555"

Open the generated HTML in any browser - no server, no install needed.
"""
import argparse
import base64
import io

import numpy as np


def read_binary_pcd(path):
    with open(path, 'rb') as f:
        n_points = None
        data_mode = None
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('#'):
                continue
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'POINTS':
                n_points = int(parts[1])
            elif parts[0] == 'DATA':
                data_mode = parts[1]
                break
        if data_mode != 'binary':
            raise RuntimeError('only binary PCD supported: ' + path)
        raw = f.read()  # PointXYZI: 16 bytes/pt
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
    arr = np.frombuffer(raw[:len(raw) // 16 * 16], dtype=dt)
    return arr


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--cloud', action='append', required=True,
                        help='"display name|pcd path|#rrggbb(default color)" - repeatable')
    args = parser.parse_args()

    here = __file__.rsplit('/', 1)[0]
    with open(here + '/three.min.js', 'r', encoding='utf-8') as f:
        three_js = f.read()
    with open(here + '/OrbitControls.js', 'r', encoding='utf-8') as f:
        orbit_js = f.read()

    clouds_js = []
    x_cursor = 0.0  # side-by-side layout: each cloud centered, then offset along X
    for spec in args.cloud:
        name, path, color = spec.split('|')
        arr = read_binary_pcd(path)
        # center each cloud on its own centroid
        ctr = np.array([arr['x'].mean(), arr['y'].mean(), arr['z'].mean()])
        xyz = np.empty((len(arr), 3), dtype='<f4')
        xyz[:, 0] = arr['x'] - ctr[0]
        xyz[:, 1] = arr['y'] - ctr[1]
        xyz[:, 2] = arr['z'] - ctr[2]
        inten = arr['i'].astype('<f4')
        # offset so multiple clouds sit side by side instead of overlapping
        half = (xyz[:, 0].max() - xyz[:, 0].min()) / 2
        x_cursor += half
        xyz[:, 0] += x_cursor
        x_cursor += half + 20.0  # 20 m gap between clouds
        b64_xyz = base64.b64encode(xyz.tobytes()).decode('ascii')
        b64_i = base64.b64encode(inten.tobytes()).decode('ascii')
        clouds_js.append(
            '{name: "%s", color: "%s", count: %d, xyz_b64: "%s", i_b64: "%s"}'
            % (name, color, len(arr), b64_xyz, b64_i)
        )

    html = TEMPLATE.replace('/*__THREE_JS__*/', three_js) \
                   .replace('/*__ORBIT_JS__*/', orbit_js) \
                   .replace('__CLOUDS_DATA__', ',\n'.join(clouds_js))

    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(html)
    print('written:', args.out, '(%.1f MB)' % (len(html) / 1e6))
    for spec in args.cloud:
        name, path, _ = spec.split('|')
        print('  embedded:', name, '<-', path)


TEMPLATE = r'''<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>SLAM map viewer</title>
<style>
  html, body { margin: 0; height: 100%; overflow: hidden; background: #16181f; font-family: Segoe UI, sans-serif; }
  #c { width: 100%; height: 100%; display: block; }
  #panel { position: fixed; top: 10px; left: 10px; background: rgba(20,22,30,.85); color: #dde;
           padding: 12px 14px; border-radius: 8px; font-size: 13px; line-height: 1.7; user-select: none; }
  #panel b { color: #fff; }
  #panel label { cursor: pointer; }
  #panel input[type=checkbox] { vertical-align: middle; }
  #panel input[type=range] { vertical-align: middle; width: 120px; }
  #help { position: fixed; bottom: 10px; left: 10px; color: #8892a6; font-size: 12px; }
  .dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 5px; }
</style>
</head>
<body>
<div id="panel">
  <b>点云查看器</b><br>
  <span id="cloudboxes"></span><br>
  <span>点大小 <input id="psize" type="range" min="1" max="6" step="0.5" value="2"></span>
  <span style="margin-left:12px">着色
    <select id="colormode">
      <option value="height" selected>按高度</option>
      <option value="intensity">按强度</option>
      <option value="flat">单色</option>
    </select></span>
</div>
<div id="help">左键旋转 · 滚轮缩放 · 右键平移</div>
<div id="c"></div>
<script>/*__THREE_JS__*/</script>
<script>/*__ORBIT_JS__*/</script>
<script>
const CLOUDS = [
__CLOUDS_DATA__
];

function b64ToF32(b64) {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Float32Array(bytes.buffer);
}

// simple rainbow colormap t in [0,1] -> rgb
function rainbow(t) {
  t = Math.max(0, Math.min(1, t));
  const h = (1.0 - t) * 4.0;               // 4: blue->cyan->green->yellow->red
  const seg = Math.floor(h);
  const f = h - seg;
  let r, g, b;
  if (seg === 0) { r = 0; g = f; b = 1; }
  else if (seg === 1) { r = 0; g = 1; b = 1 - f; }
  else if (seg === 2) { r = f; g = 1; b = 0; }
  else if (seg === 3) { r = 1; g = 1 - f; b = 0; }
  else { r = 1; g = 0; b = 0; }
  return [r, g, b];
}

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x16181f);
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.1, 5000);
const renderer = new THREE.WebGLRenderer({ antialias: false });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
document.getElementById('c').appendChild(renderer.domElement);
const controls = new THREE.OrbitControls(camera, renderer.domElement);

const groups = [];
let colorMode = 'height';
let ptSize = 2;

CLOUDS.forEach((c, idx) => {
  const pos = b64ToF32(c.xyz_b64);
  const inten = b64ToF32(c.i_b64);
  // per-cloud z range + center for camera fit
  let zmin = Infinity, zmax = -Infinity;
  const n = c.count;
  for (let i = 0; i < n; i++) {
    const z = pos[3 * i + 2];
    if (z < zmin) zmin = z;
    if (z > zmax) zmax = z;
  }
  c.zmin = zmin; c.zmax = zmax;

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  const colors = new Float32Array(n * 3);
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));

  const mat = new THREE.PointsMaterial({ size: ptSize, sizeAttenuation: false,
                                         vertexColors: true, color: 0xffffff });
  const points = new THREE.Points(geo, mat);
  scene.add(points);
  groups.push({ def: c, points: points, inten: inten, colors: colors });

  // panel checkbox
  const label = document.createElement('label');
  label.innerHTML = '<span class="dot" style="background:' + c.color + '"></span>' + c.name +
                    ' <input type="checkbox" data-idx="' + idx + '" checked> (' + n.toLocaleString() + ' 点)<br>';
  document.getElementById('cloudboxes').appendChild(label);
});

function recolor() {
  groups.forEach(g => {
    const c = g.def, n = c.count;
    const span = Math.max(1e-6, c.zmax - c.zmin);
    // three.js multiplies material.color with vertex colors - keep white when using vertex colors
    if (colorMode === 'flat') {
      g.points.material.vertexColors = false;
      g.points.material.color.set(c.color);
      return;
    }
    g.points.material.vertexColors = true;
    g.points.material.color.set(0xffffff);
    for (let i = 0; i < n; i++) {
      let r, gg, b;
      if (colorMode === 'intensity') {
        [r, gg, b] = rainbow(g.inten[i]);
      } else {
        const t = (pos_z(g, i) - c.zmin) / span;
        [r, gg, b] = rainbow(t);
      }
      g.colors[3 * i] = r; g.colors[3 * i + 1] = gg; g.colors[3 * i + 2] = b;
    }
    g.points.geometry.attributes.color.needsUpdate = true;
  });
}
function pos_z(g, i) { return g.points.geometry.attributes.position.array[3 * i + 2]; }

// toggle boxes
document.querySelectorAll('#cloudboxes input[type=checkbox]').forEach(cb => {
  cb.addEventListener('change', () => {
    groups[cb.dataset.idx].points.visible = cb.checked;
  });
});
document.getElementById('psize').addEventListener('input', e => {
  ptSize = parseFloat(e.target.value);
  groups.forEach(g => g.points.material.size = ptSize);
});
document.getElementById('colormode').addEventListener('change', e => {
  colorMode = e.target.value;
  groups.forEach(g => g.points.material.vertexColors = (colorMode !== 'flat'));
  recolor();
});

// fit camera to the first (largest) cloud
let center = new THREE.Vector3(0, 0, 0), radius = 100;
{
  let n = 0;
  const acc = new THREE.Vector3();
  let r2max = 0;
  groups.forEach(g => {
    const p = g.points.geometry.attributes.position.array;
    for (let i = 0; i < g.def.count; i += 7) {
      acc.x += p[3 * i]; acc.y += p[3 * i + 1]; acc.z += p[3 * i + 2];
      n++;
    }
  });
  acc.multiplyScalar(1 / Math.max(1, n));
  center.copy(acc);
  groups.forEach(g => {
    const p = g.points.geometry.attributes.position.array;
    for (let i = 0; i < g.def.count; i += 7) {
      const dx = p[3 * i] - acc.x, dy = p[3 * i + 1] - acc.y, dz = p[3 * i + 2] - acc.z;
      r2max = Math.max(r2max, dx * dx + dy * dy + dz * dz);
    }
  });
  radius = Math.sqrt(r2max) * 1.1;
}
camera.position.set(center.x + radius * 0.6, center.y - radius * 0.9, center.z + radius * 0.55);
controls.target.copy(center);
controls.update();

recolor();
// render on demand only - a continuous rAF loop would burn CPU/GPU while idle
function renderOnce() { renderer.render(scene, camera); }
controls.addEventListener('change', renderOnce);
renderOnce();
window.addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
  renderOnce();
});
document.getElementById('psize').addEventListener('input', renderOnce);
</script>
</body>
</html>
'''

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Render a binary PCD (PointXYZI, produced by pcl::io::savePCDFileBinary) to a BEV image."""
import argparse
import struct

import numpy as np


def read_binary_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('#'):
                continue
            parts = line.split()
            if parts[0] == 'FIELDS':
                header['fields'] = parts[1:]
            elif parts[0] == 'POINTS':
                header['points'] = int(parts[1])
            elif parts[0] == 'SIZE':
                header['size'] = [int(v) for v in parts[1:]]
            elif parts[0] == 'TYPE':
                header['type'] = parts[1:]
            elif parts[0] == 'COUNT':
                header['count'] = [int(v) for v in parts[1:]]
            elif parts[0] == 'DATA':
                header['data'] = parts[1]
                break
        if header['data'] != 'binary':
            raise RuntimeError('only binary PCD supported, got: ' + header['data'])
        point_step = sum(header['size'])
        raw = f.read(point_step * header['points'])
    # PointXYZI: x,y,z float32, padding float32, intensity float32 (pcl stores intensity in 4th channel slot)
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
    arr = np.frombuffer(raw[:len(raw) // 16 * 16], dtype=dt)
    return np.array([arr['x'], arr['y'], arr['z'], arr['i']]).T


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--title', default='hdl_graph_slam global map')
    parser.add_argument('--stride', type=int, default=3, help='point decimation')
    args = parser.parse_args()

    pts = read_binary_pcd(args.pcd)
    print(f'loaded {len(pts)} points from {args.pcd}')
    print(f'extent x: [{pts[:,0].min():.1f}, {pts[:,0].max():.1f}] y: [{pts[:,1].min():.1f}, {pts[:,1].max():.1f}] z: [{pts[:,2].min():.1f}, {pts[:,2].max():.1f}]')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    p = pts[::args.stride]

    # BEV colored by height
    zmin, zmax = np.percentile(p[:, 2], 2), np.percentile(p[:, 2], 98)
    sc = axes[0].scatter(p[:, 0], p[:, 1], c=np.clip(p[:, 2], zmin, zmax), s=0.3, cmap='turbo', linewidths=0)
    axes[0].set_aspect('equal')
    axes[0].set_title(f'{args.title} (BEV, color=height)')
    plt.colorbar(sc, ax=axes[0], shrink=0.8)

    # side view
    axes[1].scatter(p[:, 0], p[:, 2], s=0.3, c='darkgreen', linewidths=0)
    axes[1].set_aspect('equal')
    axes[1].set_title('side view (x-z)')

    for ax in axes:
        ax.grid(True, alpha=0.2)
        ax.set_xlabel('x [m]')
    axes[0].set_ylabel('y [m]')
    axes[1].set_ylabel('z [m]')

    fig.savefig(args.out, dpi=130, bbox_inches='tight')
    print(f'saved: {args.out}')


if __name__ == '__main__':
    main()

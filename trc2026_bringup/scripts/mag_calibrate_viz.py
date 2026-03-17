#!/usr/bin/env python3
import argparse
import csv
import sys
from pathlib import Path

import numpy as np


def read_csv(path):
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)

    with path.open() as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames or []

        candidates = [
            ('x', 'y', 'z'),
            ('mag_x', 'mag_y', 'mag_z'),
            ('mx', 'my', 'mz'),
        ]

        keys = None
        for c in candidates:
            if all(k in headers for k in c):
                keys = c
                break

        if keys is None:
            raise ValueError(f'CSV must contain x,y,z columns. found={headers}')

        data = []
        for row in reader:
            try:
                data.append([float(row[keys[0]]), float(row[keys[1]]), float(row[keys[2]])])
            except ValueError:
                continue

    if len(data) < 10:
        raise ValueError('Not enough samples in CSV.')

    return np.array(data, dtype=float)


def hard_iron_offset_simple(x):
    mins = x.min(axis=0)
    maxs = x.max(axis=0)
    return (maxs + mins) / 2.0


def axis_scale_matrix(x, offset):
    centered = x - offset
    mins = centered.min(axis=0)
    maxs = centered.max(axis=0)
    ranges = (maxs - mins) / 2.0

    mags = np.linalg.norm(centered, axis=1)
    avg_mag = float(np.mean(mags))

    scales = np.ones(3, dtype=float)
    for i in range(3):
        if ranges[i] > 1e-12:
            scales[i] = avg_mag / ranges[i]
        else:
            scales[i] = 1.0

    matrix = np.diag(scales)
    return matrix, avg_mag, ranges


def ellipsoid_fit(x):
    X = x[:, 0]
    Y = x[:, 1]
    Z = x[:, 2]

    D = np.column_stack([
        X * X,
        Y * Y,
        Z * Z,
        2.0 * X * Y,
        2.0 * X * Z,
        2.0 * Y * Z,
        2.0 * X,
        2.0 * Y,
        2.0 * Z,
        np.ones_like(X),
    ])

    _, _, vh = np.linalg.svd(D, full_matrices=False)
    v = vh[-1, :]

    A = np.array([
        [v[0], v[3], v[4]],
        [v[3], v[1], v[5]],
        [v[4], v[5], v[2]],
    ], dtype=float)
    b = np.array([v[6], v[7], v[8]], dtype=float)
    c = float(v[9])

    if np.linalg.det(A) == 0:
        raise ValueError('Ellipsoid fit failed: singular matrix.')

    center = -0.5 * np.linalg.inv(A).dot(b)
    k = center.T.dot(A).dot(center) - c

    if k <= 0:
        A = -A
        b = -b
        c = -c
        center = -0.5 * np.linalg.inv(A).dot(b)
        k = center.T.dot(A).dot(center) - c
        if k <= 0:
            raise ValueError('Ellipsoid fit failed: invalid scale.')

    W = A / k

    eigvals, eigvecs = np.linalg.eigh(W)
    if np.any(eigvals <= 0):
        raise ValueError('Ellipsoid fit failed: non-positive eigenvalues.')

    A_sqrt = eigvecs.dot(np.diag(np.sqrt(eigvals))).dot(eigvecs.T)

    mags = np.linalg.norm(x - center, axis=1)
    avg_mag = float(np.mean(mags))
    A_scaled = A_sqrt * avg_mag

    return center, A_scaled, avg_mag


def format_vec(v):
    return '[' + ', '.join(f'{x:.10g}' for x in v) + ']'


def format_mat(m):
    flat = m.reshape(-1)
    return '[' + ', '.join(f'{x:.10g}' for x in flat) + ']'


def write_yaml(path, offset, matrix):
    with Path(path).open('w') as f:
        f.write('mag_relay_node:\n')
        f.write('  ros__parameters:\n')
        f.write(f'    hard_iron_offset: {format_vec(offset)}\n')
        f.write(f'    soft_iron_matrix: {format_mat(matrix)}\n')


def plot_3d(ax, data, title):
    ax.scatter(data[:, 0], data[:, 1], data[:, 2], s=5, alpha=0.6)
    ax.set_title(title)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_box_aspect([1, 1, 1])


def main():
    parser = argparse.ArgumentParser(description='Magnetometer calibration + visualization.')
    parser.add_argument('csv', help='CSV file with x,y,z columns')
    parser.add_argument('--write-yaml', dest='write_yaml', help='Output YAML path')
    parser.add_argument('--no-ellipsoid', action='store_true', help='Skip ellipsoid fit')
    parser.add_argument('--show', action='store_true', help='Show plots interactively')
    parser.add_argument('--save', help='Save plot image (png)')
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print('ERROR: matplotlib is required for visualization.')
        print('Install: python3 -m pip install matplotlib')
        return 1

    x = read_csv(args.csv)

    simple = hard_iron_offset_simple(x)
    axis_matrix, axis_avg, ranges = axis_scale_matrix(x, simple)
    simple_corrected = (x - simple) @ axis_matrix.T

    ellipsoid_ok = False
    if not args.no_ellipsoid:
        try:
            center, matrix, avg_mag = ellipsoid_fit(x)
            ellipsoid_ok = True
            ellipsoid_corrected = (x - center) @ matrix.T
        except ValueError as e:
            print(f'WARNING: {e}')
            print('Falling back to simple offset + axis scaling.')
            center = simple
            matrix = axis_matrix
            avg_mag = axis_avg
            ellipsoid_corrected = simple_corrected
    else:
        center = simple
        matrix = axis_matrix
        avg_mag = axis_avg
        ellipsoid_corrected = simple_corrected

    print('=== Simple hard-iron offset (min/max) ===')
    print(f'hard_iron_offset: {format_vec(simple)}')
    print(f'axis_half_ranges: {format_vec(ranges)}')
    print(f'soft_iron_matrix (axis scale): {format_mat(axis_matrix)}')
    print(f'avg_mag: {axis_avg:.10g}')
    print('')
    if ellipsoid_ok:
        print('=== Ellipsoid fit (recommended) ===')
        print(f'hard_iron_offset: {format_vec(center)}')
        print(f'soft_iron_matrix: {format_mat(matrix)}')
        print(f'avg_mag: {avg_mag:.10g}')
    else:
        print('=== Ellipsoid fit ===')
        print('not available (insufficient coverage or degenerate data)')

    if args.write_yaml:
        write_yaml(args.write_yaml, center, matrix)
        print(f'\nWrote YAML to {args.write_yaml}')

    fig = plt.figure(figsize=(12, 10))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    plot_3d(ax1, x, 'Raw')

    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    plot_3d(ax2, simple_corrected, 'Simple Offset + Axis Scale')

    ax3 = fig.add_subplot(2, 2, 3, projection='3d')
    plot_3d(ax3, ellipsoid_corrected, 'Ellipsoid Fit')

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(np.linalg.norm(x, axis=1), label='Raw |B|')
    ax4.plot(np.linalg.norm(simple_corrected, axis=1), label='Simple |B|')
    ax4.plot(np.linalg.norm(ellipsoid_corrected, axis=1), label='Ellipsoid |B|')
    ax4.set_title('Magnitude Over Samples')
    ax4.set_xlabel('sample')
    ax4.set_ylabel('|B|')
    ax4.legend()

    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f'Saved plot to {args.save}')

    if args.show or not args.save:
        plt.show()

    return 0


if __name__ == '__main__':
    sys.exit(main())

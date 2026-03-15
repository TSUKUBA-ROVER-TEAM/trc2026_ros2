#!/usr/bin/env python3
import argparse
import csv
import math
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


def ellipsoid_fit(x):
    # Fit x^T A x + b^T x + c = 0
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
        # Flip sign if needed
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

    # Scale to preserve average magnitude after hard-iron correction
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


def main():
    parser = argparse.ArgumentParser(description='Magnetometer calibration from CSV.')
    parser.add_argument('csv', help='CSV file with x,y,z columns')
    parser.add_argument('--write-yaml', dest='write_yaml', help='Output YAML path')
    args = parser.parse_args()

    x = read_csv(args.csv)

    simple = hard_iron_offset_simple(x)
    center, matrix, avg_mag = ellipsoid_fit(x)

    print('=== Simple hard-iron offset (min/max) ===')
    print(f'hard_iron_offset: {format_vec(simple)}')
    print('')
    print('=== Ellipsoid fit (recommended) ===')
    print(f'hard_iron_offset: {format_vec(center)}')
    print(f'soft_iron_matrix: {format_mat(matrix)}')
    print(f'avg_mag: {avg_mag:.10g}')

    if args.write_yaml:
        write_yaml(args.write_yaml, center, matrix)
        print(f'\nWrote YAML to {args.write_yaml}')

    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python
import argparse, yaml
import matplotlib.pyplot as plt

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Plot the primitives in a primitives file")
    parser.add_argument('--filename', type=str, default='base_planner_parameters.yaml')

    args = parser.parse_args()

    with open(args.filename, 'r') as f:
        doc = yaml.load(f.read())

    actions = doc['actions']
    for angle in actions:
        ang = angle['angle']
        primitives = angle['primitives']
        xvals = []
        yvals = []
        tvals = []
        for primitive in primitives:
            poses = primitive['poses']
            for coord in poses:
                xvals.append(coord[0])
                yvals.append(coord[1])
                tvals.append(coord[2])

        plt.plot(xvals, yvals, '.b')
        title_str = 'Angle %d' % ang
        plt.title(title_str)
        plt.show()

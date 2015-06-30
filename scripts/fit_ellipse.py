#!/usr/bin/env python

import sys
import numpy

from math import sin,cos,atan2,pi,sqrt
import matplotlib.pyplot as plt

from ellipse2d import Ellipse2d


def load_data(filename):
    points = []
    try:
        with open(filename, 'r') as f:
            for line in f:
                numbers = map(float, line.split(','))
                points.append((numbers[0], numbers[1]))
    except:
        print 'Could not open file:', filename
        exit(1)

    print 'Got', len(points), 'points from', filename

    return points


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print 'Usage:', sys.argv[0], '<filename>'
        exit(1)

    points = load_data(sys.argv[1])
    e2 = Ellipse2d()
    e2.fit(points)
    print e2.a, e2.b, e2.theta, e2.center
    e2.plot()

    a_points = numpy.array(points)
    x = a_points[:, 0]
    y = a_points[:, 1]
    plt.scatter(x,y)

    plt.show()


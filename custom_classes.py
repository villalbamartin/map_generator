import math
from dataclasses import dataclass
from enum import Enum


class PointState(Enum):
    """ Enum for identifying what should happen to a suggested point. It can be
    rejected, accepted, or accepted but with no further branching allowed.

    """
    ACCEPTED = 0
    REJECTED = 1
    NO_BRANCH = 2


@dataclass(order=True)
class Point:
    """ Class representing a 2-D point
    """
    x: int
    y: int

    def dist(self, other):
        return math.sqrt(math.pow(other.x - self.x, 2) + math.pow(other.y - self.y, 2))


def ccw(a: Point, b: Point, c: Point) -> bool:
    """ Auxiliar class that identifies whether three points are placed in
    counter-clockwise order from each other. Required for the overlap calculation.

    Parameters
    ----------
    a : Point
        First point
    b : Point
        Second point
    c : Point
        Third point

    Returns
    -------
    bool
        True if the three points are counter-clockwise from each other
    """
    return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x)


def intersect(a: Point, b: Point, c: Point, d:Point) -> bool:
    """ Returns True if the segment AB intersects CD.

    Parameters
    ----------
    a : Point
        First point
    b : Point
        Second point
    c : Point
        Third point
    d : Point
        Fourth point

    Returns
    -------
    bool
        True if the segment AB intersects the segment CD.

    References
    ----------
    Algorithm obtained from https://stackoverflow.com/a/9997374 with an extra
    check for overlapping endpoints.
    """
    if (a.x == c.x and a.y == c.y) or (a.x == d.x and a.y == d.y) or \
       (b.x == c.x and b.y == c.y) or (b.x == d.x and b.y == d.y):
        # Shared vertices don't count
        return False
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)


@dataclass(order=True)
class RoadQuery:
    t: int
    ra: (Point, Point)  # Can be used to build a segment

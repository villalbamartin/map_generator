from dataclasses import dataclass
from typing import Tuple
import heapq
import math
import pygame
import random
import time

from collections import namedtuple

# https://nothings.org/gamedev/l_systems.html


@dataclass(order=True)
class Point:
    x: int
    y: int

    def dist(self, other):
        return math.sqrt(math.pow(other.x - self.x, 2) + math.pow(other.y - self.y, 2))


def ccw(a:Point, b:Point, c:Point) -> bool:
    # SO: 9997374
    return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x)


def intersect(a, b, c, d):
    """ Returns True if AB intersects CD

    Parameters
    ----------
    a : Point
    b : Point
    c : Point
    d : Point

    Returns
    -------
    bool
    """
    if (a.x == c.x and a.y == c.y) or (a.x == d.x and a.y == d.y) or \
        (b.x == c.y and b.y == c.y) or (b.x == d.x and b.y == d.y):
        # Shared vertex doesn't count
        return False
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)


@dataclass(order=True)
class RoadQuery:
    t: int
    ra: (Point, Point)  # Can be used to build a segment
    qa: str


def localConstraints(nqa, nra: Tuple[Point, Point],
                     segments: [Tuple[Point, Point]],
                     show_reason: bool = False):
    """

    Parameters
    ----------
    nqa : ??
        Parameter used for... something?
    nra : [Tuple[Point, Point]]

    Returns
    -------
    Point, bool
        The (perhaps modified) input based on local constraints, and a boolean
        to indicate whether the new point is to be accepted or discarded.

    Notes
    -----
    Currently the second parameter is state, which is either
    "Succeed" or "Fail". We use a boolean instead.

    """
    # Restrictions to add:
    #  - Do not cross over an existing segment
    #  -
    reason = None
    retval = True
    if len(segments) == 0:
        return nra, True
    if nra[1].x < 0 or nra[1].x > 500 or nra[1].y < 0 or nra[1].y > 500:
        if show_reason:
            print("Original point outside of boundaries")
        return nra, False

    nnra = (nra[0], nra[1])
    radius = 50
    relevant_segments = list(filter(lambda x: nra[1].dist(x[0]) < 5*radius or
                                              nra[1].dist(x[1]) < 5*radius,
                                    segments))
    # Filter 1: if the new point is too close to another one then unify that
    # point with the existing one.
    # Find the closest point to the final point and, if it's less than radius,
    # then unify them
    closest_point = None
    closest_dist = radius
    for (s0, s1) in relevant_segments:
        # Check that it's not too close to the root point
        if s0 == nnra[0] or s1 == nnra[0]:
            continue
        d0 = nnra[1].dist(s0)
        d1 = nnra[1].dist(s1)
        if d0 < closest_dist:
            if d1 < closest_dist:
                closest_point = s1
                closest_dist = d1
            else:
                closest_point = s0
                closest_dist = d0
        elif d1 < closest_dist:
            if d0 < closest_dist:
                closest_point = s0
                closest_dist = d0
            else:
                closest_point = s1
                closest_dist = d1
    if 0 < closest_dist < radius:
        nnra = (nnra[0], closest_point)
        reason = "Point unified due to being too close to another one"

    # Filter 2: no overlaps
    for (s0, s1) in relevant_segments:
        if intersect(s0, s1, nnra[0], nnra[1]):
            # Too bad
            retval = False
            reason = f"Point deleted due to overlap: {s0},{s1} overlaps {nnra[0]},{nnra[1]}"
            break
    """
    # New rule: the starting point cannot be too close to an existing one,
    # and the end point can be affixed to an existing one
    # A starting point can overlap other points, but no more than one.
    num_incoming_edges = sum(map(lambda x: nnra[0].dist(x[0]) < 1, segments))
    num_incoming_edges += sum(map(lambda x: nnra[0].dist(x[1]) < 1, segments))

    if num_incoming_edges > 2:
        retval = False
    else:
    """
    if not (0 <= nra[1].x <= 500 and 0 <= nra[1].y <= 500):
        reason = "Point outside boundaries"
        retval = False
    if reason is not None and show_reason:
        print(reason)
    return nra, retval


def globalGoals(rq: RoadQuery):
    prev_point, curr_point = rq.ra
    radius = 40
    angle = math.atan2(curr_point.y - prev_point.y, curr_point.x - prev_point.x)
    # Param 0: production deletion - setting it to -1 stops the iteration here
    # Param 1: unclear
    # Param 2: three new points. It's going to be the result of going straight,
    # branching left, and branching right

    # Simple algorithm: branch at 90 degrees without checking (for now) if the
    # points are fine globally
    branches = []
    curr_angle = angle - math.pi/2
    for _ in range(3):
        delta = (int(radius*math.cos(curr_angle)), int(radius*math.sin(curr_angle)))
        tentative_point = Point(int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
        branches.append((curr_point, tentative_point))
        curr_angle += math.pi/2 + (random.random()/2 - 0.25)
    return [rq.t]*3, [None]*3, branches


if __name__ == '__main__':
    width = 500
    height = 500
    random.seed(1)

    pygame.init()
    screen = pygame.display.set_mode((width, height))
    canvas = pygame.Surface((width, height))
    old_canvas = pygame.Surface((width, height))

    pqueue = [RoadQuery(0, (Point(250, 250), Point(270, 270)), "qa")]  # roadq: list of road queries - replaced by prior. queue
    segment_list = []  # S

    counter = 0
    while len(pqueue) > 0:
        print(counter)
        counter += 1
        canvas.blit(old_canvas, (0, 0))
        # r contains t, ra, qa
        smallest_rq = heapq.heappop(pqueue)
        nqa, state = localConstraints(smallest_rq.qa, smallest_rq.ra, segment_list, show_reason=True)
        if state:
            segment_list.append(nqa)
            pygame.draw.line(canvas, 'white', (nqa[0].x, nqa[0].y), (nqa[1].x, nqa[1].y))
            #pygame.draw.circle(canvas, color='blue', center=(nqa[1].x, nqa[1].y), radius=5)
            # Make sure that the current change to the canvas is permanent
            old_canvas.blit(canvas, (0, 0))

            pdel, nqa, nra = globalGoals(smallest_rq)
            for i in range(3):
                if 7 >= pdel[i] >= 0:
                    heapq.heappush(pqueue, RoadQuery(smallest_rq.t + 1 + pdel[i],
                                                     nra[i],
                                                     nqa[i]))
        else:
            # Make sure that the current change to the canvas lasts only one frame
            old_canvas.blit(canvas, (0, 0))
            pygame.draw.line(canvas, (100, 100, 100), (nqa[0].x, nqa[0].y), (nqa[1].x, nqa[1].y))
            pygame.draw.circle(canvas, color='red', center=(nqa[1].x, nqa[1].y), radius=5)

        screen.blit(canvas, (0, 0))
        pygame.display.flip()
        move_on = False
        while not move_on:
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.KEYDOWN:
                    move_on = True
    screen.blit(canvas, (0, 0))
    pygame.display.flip()
    import time
    time.sleep(1)

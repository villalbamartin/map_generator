from typing import Tuple
import heapq
import math
import pygame
import random
import time

from custom_classes import PointState, Point, RoadQuery, intersect


# https://nothings.org/gamedev/l_systems.html


def localConstraints(nra: Tuple[Point, Point],
                     segments: [Tuple[Point, Point]],
                     show_reason: bool = False) -> [Point, PointState]:
    """


    Parameters
    ----------
    nqa : ??
        Parameter used for... something?
    nra : [Tuple[Point, Point]]

    Returns
    -------
    Point, PointState
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
    retval = PointState.ACCEPTED
    if len(segments) == 0:
        return nra, PointState.ACCEPTED
    if nra[1].x < 0 or nra[1].x > 500 or nra[1].y < 0 or nra[1].y > 500:
        if show_reason:
            print("Original point outside of boundaries")
        return nra, PointState.REJECTED

    nnra = (nra[0], nra[1])
    radius = 30
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
        # We stop the branching here because, whatever branching could happen
        # in this corner, it has already been done
        retval = PointState.NO_BRANCH

    # Filter 2: no overlaps
    for (s0, s1) in relevant_segments:
        if intersect(s0, s1, nnra[0], nnra[1]):
            # Too bad
            retval = PointState.REJECTED
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
        retval = PointState.REJECTED
    if reason is not None and show_reason:
        print(reason)
    return nnra, retval


def globalGoals(rq: RoadQuery):
    prev_point, curr_point = rq.ra
    radius = 50
    angle = math.atan2(curr_point.y - prev_point.y, curr_point.x - prev_point.x)
    # Param 0: production deletion - setting it to -1 stops the iteration here
    # Param 1: unclear
    # Param 2: three new points. It's going to be the result of going straight,
    # branching left, and branching right

    house_x = 350
    house_y = 100
    hradius = 70

    # Simple algorithm: branch at 90 degrees without checking (for now) if the
    # points are fine globally
    branches = []
    curr_angle = angle - math.pi/2
    for _ in range(3):
        nradius = radius
        delta = (int(radius*math.cos(curr_angle)), int(radius*math.sin(curr_angle)))
        tentative_point = Point(int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
        while math.dist((house_x, house_y),
                        (int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))) < hradius and nradius > 0:
            #curr_angle += 0.1
            #delta = (int(radius * math.cos(curr_angle)), int(radius * math.sin(curr_angle)))
            nradius -= 1
            delta = (int(nradius * math.cos(curr_angle)), int(nradius * math.sin(curr_angle)))
            tentative_point = Point(int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))

        if nradius > 0:
            branches.append((curr_point, tentative_point))
        curr_angle += math.pi/2 + (random.random()/2 - 0.125)
    return [rq.t]*len(branches), branches


if __name__ == '__main__':
    width = 500
    height = 500
    random.seed(1)

    pygame.init()
    screen = pygame.display.set_mode((width, height))
    canvas = pygame.Surface((width, height))
    menu_font = pygame.font.SysFont('Px ATI 8x16', 30)

    pqueue = [RoadQuery(0, (Point(250, 250), Point(270, 270)))]  # roadq: list of road queries - replaced by prior. queue
    segment_list = []  # S
    segment_weight_list = []

    counter = 0
    while len(pqueue) > 0:
        canvas.fill('black')
        text_surface = menu_font.render(str(counter), False, (255, 255, 255))
        canvas.blit(text_surface, (0, 0))
        counter += 1

        # Draw the current city. Yes, this is inefficient, shut up.
        for rq in pqueue:
            s = rq.ra
            pygame.draw.line(canvas, (150, 150, 150),
                             (s[0].x, s[0].y), (s[1].x, s[1].y),
                             width=1)
        for s, sw in zip(segment_list, segment_weight_list):
            pygame.draw.line(canvas, 'white',
                             (s[0].x, s[0].y), (s[1].x, s[1].y),
                             width=max(0, 7-sw))
            #pygame.draw.circle(canvas, color='blue', center=(nqa[1].x, nqa[1].y), radius=5)

        # r contains t, ra, qa
        smallest_rq = heapq.heappop(pqueue)
        nra, state = localConstraints(smallest_rq.ra, segment_list, show_reason=True)
        if state == PointState.ACCEPTED or state == PointState.NO_BRANCH:
            segment_list.append(nra)
            segment_weight_list.append(smallest_rq.t//20)
            pygame.draw.line(canvas, 'white', (nra[0].x, nra[0].y), (nra[1].x, nra[1].y))

            if state == PointState.ACCEPTED:
                pdel, nra = globalGoals(smallest_rq)
                for npdel, nnra in zip(pdel, nra):
                    if 1400 >= npdel >= 0:
                        heapq.heappush(pqueue, RoadQuery(smallest_rq.t + 1 + npdel,
                                                         nnra))
        else:
            pygame.draw.line(canvas, (100, 100, 100), (nra[0].x, nra[0].y), (nra[1].x, nra[1].y))
            pygame.draw.circle(canvas, color='red', center=(nra[1].x, nra[1].y), radius=5)

        screen.blit(canvas, (0, 0))
        pygame.display.flip()
        time.sleep(.001)
        move_on = True
        while not move_on:
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.KEYDOWN:
                    move_on = True
    screen.blit(canvas, (0, 0))
    pygame.display.flip()
    time.sleep(5)

from typing import Tuple
import heapq
import math
import networkx as nx
import pygame
import random
import time

from custom_classes import PointState, Point, RoadQuery, intersect


# https://nothings.org/gamedev/l_systems.html


def localConstraints(nra: Tuple[Point, Point],
                     segments: [Tuple[Point, Point]],
                     width: int = 500,
                     height: int = 500,
                     radius: int = 30,
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
    if nra[1].x < 0 or nra[1].x > width or nra[1].y < 0 or nra[1].y > height:
        if show_reason:
            print("Original point outside of boundaries")
        return nra, PointState.REJECTED

    nnra = (nra[0], nra[1])
    relevant_segments = list(filter(lambda x: nra[1].dist(x[0]) <= 2 * radius or
                                              nra[1].dist(x[1]) <= 2 * radius,
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
    if not (0 <= nra[1].x <= width and 0 <= nra[1].y <= height):
        reason = "Point outside boundaries"
        retval = PointState.REJECTED
    if reason is not None and show_reason:
        print(reason)
    return nnra, retval


def globalGoals(rq: RoadQuery, isles: [Point], radius: int = 30, isle_radius = 60,
                river = None):

    prev_point, curr_point = rq.ra
    #radius = random.randint(30, 60)
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
        nradius = radius
        delta = (int(radius*math.cos(curr_angle)), int(radius*math.sin(curr_angle)))
        tentative_point = Point(int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
        for house in isles:
            while math.dist((house.x, house.y),
                            (int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))) < isle_radius and nradius > 0:
                curr_angle += 0.1
                delta = (int(radius * math.cos(curr_angle)), int(radius * math.sin(curr_angle)))
                #nradius -= 1
                #delta = (int(nradius * math.cos(curr_angle)), int(nradius * math.sin(curr_angle)))
                tentative_point = Point(int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
        if nradius > 0:
            branches.append((curr_point, tentative_point))
        curr_angle += math.pi/2 + (random.random()/4 - 0.125)
    return [rq.t]*len(branches), branches


def visualize_system(canvas, desc_msg, pqueue, segment_list, segment_weight_list):
    """ Plots the city in its current state.

    Parameters
    ----------
    canvas
        The surface where we'll draw the current state of the city
    desc_msg : str
        A message to display on screen
    pqueue :
        Priority queue with proposed roads
    segment_list :
        List with approved segments
    segment_weight_list : [int]
        List of weights for the segment list
    """
    # Debugging: show the canvas
    canvas.fill('white')
    text_surface = menu_font.render(desc_msg, False, (255, 255, 255))
    canvas.blit(text_surface, (0, 0))

    # Draw the current city. Yes, this is inefficient, shut up.
    for requested_road in pqueue:
        s = requested_road.ra
        pygame.draw.line(canvas, (150, 150, 150),
                         (s[0].x, s[0].y), (s[1].x, s[1].y),
                         width=1)
    try:
        max_weight = 1+max(segment_weight_list)
    except ValueError:
        max_weight = 1
    for s, s_weight in zip(segment_list, segment_weight_list):
        pygame.draw.line(canvas, 'black',
                         (s[0].x, s[0].y), (s[1].x, s[1].y),
                         width=16)
                         #width=int(8-8*(s_weight/max_weight)))
        # pygame.draw.circle(canvas, color='blue', center=(nqa[1].x, nqa[1].y), radius=5)


#if __name__ == '__main__':
def main2():
    # Initial parameters of the simulation
    width = 1500
    height = 500
    radius_for_jump = 75
    radius_for_collision = 55
    max_branching = 99999999999
    random.seed(1)
    # We want to have a certain number of "isles" for the sections of our map.
    isles = [Point(30, 80), Point(130, 400), Point(700, 250), Point(1000, 20), Point(1300, 400)]
    isle_radius = 90

    # Initialize PyGame
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    canvas = pygame.Surface((width, height))
    menu_font = pygame.font.SysFont('Px ATI 8x16', 30)

    river = [Point(0, 0)]
    # Build a river
    while river[-1].x < 1000:
        new_point = Point(river[-1].x + random.randint(10, isle_radius),
                          river[-1].y + random.randint(-int(isle_radius/5), isle_radius))
        river.append(new_point)


    # Initialize the queue with a single segment
    pqueue = [RoadQuery(0, (Point(250, 250), Point(270, 270)))]
    segment_list = []
    segment_weight_list = []

    counter = 0
    while len(pqueue) > 0:
        if counter % 250 == 0 and False:
            visualize_system(canvas, str(counter), pqueue, segment_list, segment_weight_list)
        counter += 1

        # r contains t, ra, qa
        smallest_rq = heapq.heappop(pqueue)
        nra, state = localConstraints(smallest_rq.ra, segment_list,
                                      width=width, height=height,
                                      radius=radius_for_collision,
                                      show_reason=False)
        if state == PointState.ACCEPTED or state == PointState.NO_BRANCH:
            segment_list.append(nra)
            segment_weight_list.append(smallest_rq.t)
            pygame.draw.line(canvas, 'black', (nra[0].x, nra[0].y), (nra[1].x, nra[1].y))

            if state == PointState.ACCEPTED:
                pdel, nra = globalGoals(smallest_rq, isles,
                                        radius=radius_for_jump,
                                        isle_radius=isle_radius,
                                        river=None)
                for npdel, nnra in zip(pdel, nra):
                    if max_branching >= npdel >= 0:
                        heapq.heappush(pqueue, RoadQuery(smallest_rq.t + 1 + npdel,
                                                         nnra))
        else:
            pygame.draw.line(canvas, (100, 100, 100), (nra[0].x, nra[0].y), (nra[1].x, nra[1].y))
            pygame.draw.circle(canvas, color='red', center=(nra[1].x, nra[1].y), radius=7)

        screen.blit(canvas, (0, 0))
        pygame.display.flip()
        move_on = True
        while not move_on:
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.KEYDOWN:
                    move_on = True
        print(len(pqueue))
    screen.blit(canvas, (0, 0))
    pygame.display.flip()

    print("Calculating polygons")
    canvas.fill('white')
    # Build the set of polygons that will make the final map
    G = nx.Graph()
    coord2id = dict()
    id2coord = dict()
    for s0, s1 in segment_list:
        key0 = (s0.x, s0.y)
        key1 = (s1.x, s1.y)
        try:
            v0 = coord2id[key0]
        except KeyError:
            v0 = coord2id[key0] = len(coord2id)
            id2coord[v0] = key0
        try:
            v1 = coord2id[key1]
        except KeyError:
            v1 = coord2id[key1] = len(coord2id)
            id2coord[v1]= key1
        G.add_edge(v0, v1)
    print(G)
    for cycle in nx.minimum_cycle_basis(G):
        xs = []
        ys = []
        for v0, v1 in zip(cycle, cycle[1:] + [cycle[0]]):
            p0 = id2coord[v0]
            p1 = id2coord[v1]
            xs.append(p0[0])
            xs.append(p1[0])
            ys.append(p0[1])
            ys.append(p1[1])
        center = (0.5 * min(xs) + 0.5 * max(xs), 0.5 * min(ys) + 0.5 * max(ys))
        xs = [(i - center[0]) * (1 - 0.3) + center[0] for i in xs]
        ys = [(i - center[1]) * (1 - 0.3) + center[1] for i in ys]
        polygon = list(zip(xs, ys))
        if len(polygon) > 2:
            if random.random() < 0.9:
                pygame.draw.polygon(canvas, (238, 219, 151), polygon)
                pygame.draw.lines(canvas, 'black', False, polygon)
            else:
                pygame.draw.polygon(canvas, 'black', polygon)

    # Draw the river
    pygame.draw.lines(canvas, (122, 175, 225), False, list(map(lambda p: (p.x, p.y), river)), width=50)
    screen.blit(canvas, (0, 0))
    pygame.display.flip()

if __name__ == '__main__':
    import cProfile
    cProfile.run('main2()', 'restats.prof')
    time.sleep(10)

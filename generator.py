# Algorithm for streets based on this post:
# https://nothings.org/gamedev/l_systems.html
import heapq
import math
import networkx as nx
import pygame
import random
import time
from custom_classes import PointState, Point, RoadQuery, intersect, GlobalState, Segment
from typing import Tuple


def localConstraints(nra: Segment,
                     segments: [Tuple[Segment]],
                     width: int = 500,
                     height: int = 500,
                     radius: int = 30,
                     show_reason: bool = False) -> [Point, PointState]:
    """


    Parameters
    ----------
    nra : Segment
        Parameter used to know where I'm coming from and in which direction I
        should branch (if any)
    nqa : Segment
        Parameter used for... something?

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
    if nra.p1.x < -50 or nra.p1.x > width + 2*radius or nra.p1.y < -50 or nra.p1.y > height + 2*radius:
        if show_reason:
            print("Original point outside of boundaries")
        return nra, PointState.REJECTED

    nnra = Segment(nra.p0, nra.p1)
    relevant_segments = list(filter(lambda x: nra.p1.dist(x.p0) <= 2 * radius or
                                              nra.p1.dist(x.p1) <= 2 * radius,
                                    segments))
    # Filter 1: if the new point is too close to another one then unify that
    # point with the existing one.
    # Find the closest point to the final point and, if it's less than radius,
    # then unify them
    closest_point = None
    closest_dist = radius
    for segment in relevant_segments:
        # Check that it's not too close to the root point
        if segment.p0 == nnra.p0 or segment.p1 == nnra.p0:
            continue
        d0 = nnra.p1.dist(segment.p0)
        d1 = nnra.p1.dist(segment.p1)
        if d0 < closest_dist:
            if d1 < closest_dist:
                closest_point = segment.p1
                closest_dist = d1
            else:
                closest_point = segment.p0
                closest_dist = d0
        elif d1 < closest_dist:
            if d0 < closest_dist:
                closest_point = segment.p0
                closest_dist = d0
            else:
                closest_point = segment.p1
                closest_dist = d1
    if 0 < closest_dist < radius:
        nnra = Segment(nnra.p0, closest_point)
        reason = "Point unified due to being too close to another one"
        # We stop the branching here because, whatever branching could happen
        # in this corner, it has already been done
        retval = PointState.NO_BRANCH

    # Filter 2: no overlaps
    for segment in relevant_segments:
        if intersect(segment.p0, segment.p1, nnra.p0, nnra.p1):
            # Too bad
            retval = PointState.REJECTED
            reason = f"Point deleted due to overlap: {segment.p0},{segment.p1} overlaps {nnra.p0},{nnra.p1}"
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
    if not (-50 <= nra.p1.x <= width and -50 <= nra.p1.y <= height):
        reason = "Point outside boundaries"
        retval = PointState.REJECTED
    if reason is not None and show_reason:
        print(reason)
    return nnra, retval


#def globalGoals(rq: RoadQuery, isles: [Point], radius: int = 30, isle_radius = 60,
#                river = None):
def globalGoals(rq: RoadQuery, world: GlobalState, radius: int = 30):
    """ Function that determines in which direction the search is supposed to
    go.

    Parameters
    ----------
    rq : RoadQuery
        Starting state of the generation process.
    world : GlobalState
        State of the world used to controle the generation.
    radius : int
        Distance from existing points to use to jump when creating new points.

    Returns
    -------
    int, [Tuple(Point, Point)]

    """
    prev_point = rq.ra.p0
    curr_point = rq.ra.p1
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
        for house, house_radius in world.houses:
            new_point = (int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
            while (math.dist((house.x, house.y), new_point) < house_radius and
                   radius > 0):
                curr_angle += 0.1
                delta = (int(radius * math.cos(curr_angle)), int(radius * math.sin(curr_angle)))
                tentative_point = Point(int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
                if curr_angle > 4 * math.pi:
                    break
        if curr_angle <= 4 * math.pi:
            branches.append(Segment(curr_point, tentative_point))
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
    menu_font = pygame.font.SysFont('Px ATI 8x16', 30)
    canvas.fill('white')
    text_surface = menu_font.render(desc_msg, False, (255, 255, 255))
    canvas.blit(text_surface, (0, 0))

    # Draw the current city. Yes, this is inefficient, shut up.
    for requested_road in pqueue:
        s = requested_road.ra
        pygame.draw.line(canvas, (150, 150, 150),
                         (s.p0.x, s.p0.y), (s.p1.x, s.p1.y),
                         width=1)
    try:
        max_weight = 1+max(segment_weight_list)
    except ValueError:
        max_weight = 1
    for s, s_weight in zip(segment_list, segment_weight_list):
        pygame.draw.line(canvas, 'black',
                         (s.p0.x, s.p0.y), (s.p1.x, s.p1.y),
                         width=3)
                         #width=int(8-8*(s_weight/max_weight)))
        # pygame.draw.circle(canvas, color='blue', center=(nqa[1].x, nqa[1].y), radius=5)


#if __name__ == '__main__':
def main2():
    # Initial parameters of the simulation
    width = 600
    height = 600
    radius_for_jump = 50
    radius_for_collision = 35
    max_branching = 99999999999
    random.seed(1)

    # We want to have a certain number of "houses" for the sections of our map.
    houses = [(Point(30, 80), 80), (Point(130, 400), 80), (Point(700, 250), 80),
              (Point(1000, 20), 80), (Point(1300, 400), 80)]
    world = GlobalState(houses=houses, river=None)

    # Initialize PyGame
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    canvas = pygame.Surface((width, height))

    river = [Point(0, 0)]
    """
    # Build a river
    while river[-1].x < 1000:
        new_point = Point(river[-1].x + random.randint(10, isle_radius),
                          river[-1].y + random.randint(-int(isle_radius/3), isle_radius))
        river.append(new_point)
    """
    # Initialize the queue with a single segment
    pqueue = [RoadQuery(0, Segment(Point(250, 250), Point(270, 270)))]
    segment_list = []
    segment_weight_list = []

    counter = 0
    while len(pqueue) > 0:
        if counter % 250 == 0:
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
            pygame.draw.line(canvas, 'black', (nra.p0.x, nra.p0.y), (nra.p1.x, nra.p1.y))

            if state == PointState.ACCEPTED:
                pdel, nra = globalGoals(smallest_rq, world,
                                        radius=radius_for_jump)
                for npdel, nnra in zip(pdel, nra):
                    if max_branching >= npdel >= 0:
                        heapq.heappush(pqueue, RoadQuery(smallest_rq.t + 1 + npdel,
                                                         nnra))
        else:
            pygame.draw.line(canvas, (100, 100, 100), (nra.p0.x, nra.p0.y), (nra.p1.x, nra.p1.y))
            pygame.draw.circle(canvas, color='red', center=(nra.p1.x, nra.p1.y), radius=7)
        screen.blit(canvas, (0, 0))
        pygame.display.flip()
        move_on = True
        while not move_on:
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.KEYDOWN:
                    move_on = True
    screen.blit(canvas, (0, 0))
    pygame.display.flip()

    print("Calculating polygons")
    canvas.fill('white')
    from collections import Counter
    edge2count = Counter()

    # Build the set of polygons that will make the final map
    G = nx.Graph()
    coord2id = dict()
    id2coord = dict()
    for segment in segment_list:
        key0 = (segment.p0.x, segment.p0.y)
        key1 = (segment.p1.x, segment.p1.y)
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
        # By splitting the graph in 4 we can speed results up, but we still
        # have to calculate the nodes that got lost in the middle.
        # How do we do this recursively?
        if (id2coord[v0][0] < width/2 and
            id2coord[v0][1] < height/2 and
            id2coord[v1][0] < width/2 and
            id2coord[v1][1] < height/2):
            G.add_edge(v0, v1)
        elif (id2coord[v0][0] < width/2 and
            id2coord[v0][1] > height/2 and
            id2coord[v1][0] < width/2 and
            id2coord[v1][1] > height/2):
            G.add_edge(v0, v1)
        elif (id2coord[v0][0] > width/2 and
            id2coord[v0][1] < height/2 and
            id2coord[v1][0] > width/2 and
            id2coord[v1][1] < height/2):
            G.add_edge(v0, v1)
        elif (id2coord[v0][0] > width/2 and
            id2coord[v0][1] > height/2 and
            id2coord[v1][0] > width/2 and
            id2coord[v1][1] > height/2):
            G.add_edge(v0, v1)

    print(G)
    start_ts = time.time()
    cycles = nx.minimum_cycle_basis(G)
    print(f"Elapsed time: {time.time() - start_ts:.3}")
    start_ts = time.time()
    for cycle in cycles:
        xs = []
        ys = []
        if len(cycle) < 3:
            continue
        for v0, v1 in zip(cycle, cycle[1:] + [cycle[0]]):
            p0 = id2coord[v0]
            p1 = id2coord[v1]
            xs.append(p0[0])
            xs.append(p1[0])
            ys.append(p0[1])
            ys.append(p1[1])
            # Update the polygon count too.
            # The "if" here is because I need to normalize the segments.
            # Otherwise they can be inserted twice.
            if Point(p0[0], p0[1]).dist(Point(0, 0)) < Point(p1[0], p1[1]).dist(Point(0, 0)):
                edge2count[(p0[0], p0[1], p1[0], p1[1])] += 1
            else:
                edge2count[(p1[0], p1[1], p0[0], p0[1])] += 1
        center = (0.5 * min(xs) + 0.5 * max(xs), 0.5 * min(ys) + 0.5 * max(ys))
        xs = [(i - center[0]) * (1 - 0.3) + center[0] for i in xs]
        ys = [(i - center[1]) * (1 - 0.3) + center[1] for i in ys]
        polygon = list(zip(xs, ys))
        if len(polygon) > 2:
            if random.random() < 0.95:
                pygame.draw.polygon(canvas, (238, 219, 151), polygon)
            else:
                pygame.draw.polygon(canvas, (181, 238, 151), polygon)
            pygame.draw.lines(canvas, 'black', False, polygon)
            """
                if random.random() < 0.5:
                    img = pygame.image.load('tree.png').convert_alpha()
                else:
                    img = pygame.image.load('tower.png').convert_alpha()
                canvas.blit(img, (polygon[0][0]+30, polygon[0][1]))
            """
    for outer_edge, outer_edge_count in edge2count.items():
        if outer_edge_count == 1:
            pygame.draw.line(canvas, 'red', (outer_edge[0], outer_edge[1]), (outer_edge[2], outer_edge[3]))
    print(f"Elapsed time for drawing only: {time.time() - start_ts:.3}")
    # Draw the river
    # pygame.draw.lines(canvas, (122, 175, 225), False, list(map(lambda p: (p.x, p.y), river)), width=50)
    screen.blit(canvas, (0, 0))
    pygame.display.flip()

if __name__ == '__main__':
    import cProfile
    cProfile.run('main2()', 'restats.prof')
    time.sleep(10)

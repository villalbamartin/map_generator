# Algorithm for streets based on this post:
# https://nothings.org/gamedev/l_systems.html
import heapq
import math
import networkx as nx
import pygame
import random
import time
from collections import Counter, defaultdict
from custom_classes import PointState, Point, RoadQuery, intersect, GlobalState, Segment


def local_constraints(nra: Segment,
                      segments: [Segment],
                      width: int = 500,
                      height: int = 500,
                      radius: int = 30,
                      show_reason: bool = False) -> [Point, PointState]:
    """ Function that decides whether a point can be placed at a give place.
    Sometimes the function will move the point somewhere else to satisfy the
    constraints.

    Parameters
    ----------
    nra : Segment
        Parameter used to know where I'm coming from and in which direction I
        should branch (if any).
    segments : [Segment]
        List of known segments for the world. Useful for knowing where I am not
        allowed to place a point.
    width : int
        Width of the world. Used to know where the boundaries are.
    height : int
        Height of the world. Used to know where the boundaries are.
    radius : int
        Radius around the point that I'm supposed to take into account when
        placing the new point. If the new point is closer than this to an
        existing point, we snap the new point to the old one.
    show_reason : bool
        If True it will print messages showing why a certain point was placed.

    Returns
    -------
    Point, PointState
        The (perhaps modified) input based on local constraints, and a boolean
        to indicate whether the new point is to be accepted or discarded.
    """
    reason = None
    retval = PointState.ACCEPTED
    if len(segments) == 0:
        return nra, PointState.ACCEPTED
    if (nra.p1.x < -50 or nra.p1.x > width + 2 * radius or
        nra.p1.y < -50 or nra.p1.y > height + 2 * radius):
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
    # Check how close I am to the nearest point and, if less than the desired
    # radius, then both points are unified and the return value instructs the
    # system not to continue branching from here (because there's already a
    # point there).
    if 0 < closest_dist < radius:
        nnra = Segment(nnra.p0, closest_point)
        reason = "Point unified due to being too close to another one"
        # We stop the branching here because, whatever branching could happen
        # in this corner, it has already been done
        retval = PointState.NO_BRANCH

    # Filter 2: no overlaps with existing segments
    for segment in relevant_segments:
        if intersect(segment.p0, segment.p1, nnra.p0, nnra.p1):
            # Too bad
            retval = PointState.REJECTED
            reason = f"Point deleted due to overlap: {segment.p0},{segment.p1} overlaps {nnra.p0},{nnra.p1}"
            break
    # We check (again) that the point is not outside the boundaries.
    if not (-50 <= nra.p1.x <= width and -50 <= nra.p1.y <= height):
        reason = "Point outside boundaries"
        retval = PointState.REJECTED
    if reason is not None and show_reason:
        print(reason)
    return nnra, retval


def global_goals(rq: RoadQuery, world: GlobalState, radius: int = 30):
    """ Function that determines in which direction the expansion is supposed to
    continue.

    Parameters
    ----------
    rq : RoadQuery
        Starting state of the generation process.
    world : GlobalState
        State of the world used to control the generation.
    radius : int
        Distance from existing points to use to jump when creating new points.

    Returns
    -------
    [int], [Segment]
        A t parameter used to penalize the generation (the higher the number,
        the fewer branches there will be) and a list of newly-suggested segments
        to evaluate for the next iteration.
    """
    prev_point = rq.ra.p0
    curr_point = rq.ra.p1
    angle = math.atan2(curr_point.y - prev_point.y, curr_point.x - prev_point.x)

    # Simple algorithm: branch at 90 degrees without checking (for now) if the
    # points are fine globally. We achieve this branching by adding 90° to the
    # angle at every iteration.
    # We have 3 iterations because, at 90° each, a fourth iteration would
    # overlap with the starting one.
    branches = []
    curr_angle = angle - math.pi/2
    for _ in range(3):
        delta = (int(radius*math.cos(curr_angle)), int(radius*math.sin(curr_angle)))
        tentative_point = Point(int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
        # We don't want to get too close to the houses - if that happens we
        # fiddle with the point until it's far enough.
        for house, house_radius in world.houses:
            new_point = (int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
            while (math.dist((house.x, house.y), new_point) < house_radius and
                   radius > 0):
                curr_angle += 0.1
                delta = (int(radius * math.cos(curr_angle)), int(radius * math.sin(curr_angle)))
                tentative_point = Point(int(curr_point.x + delta[0]), int(curr_point.y + delta[1]))
                if curr_angle > 4 * math.pi:
                    # If I performed a full turn then there's no angle that will
                    # work and we give up on this point.
                    break
        # In combination with the above check, this part ensures that we add a
        # new point but only if we could place it around all the existing
        # houses.
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


def get_perimeter(polygon_list: [[Point]]) -> [Point]:
    """ Obtains the list of segments that correspond to those in the periphery
    of this graph. The graph is assumed to be connected and representing a
    single polygon - if you have more than one component, I don't know what to
    tell you.

    Parameters
    ----------
    polygon_list : [[Point]]
        List of known points that, when put together, form a mostly-connected
        graph.

    Returns
    -------
    [[Segments]]
        List of segments that represent the outer perimeter of this graph.
    """

    ## This should return a single list of segments. Yes, really!
    edge2count = Counter()
    retval = []
    for polygon in polygon_list:
        for p0, p1 in zip(polygon, polygon[1:] + [polygon[0]]):
            #edge2count[(p0.x, p0.y, p1.x, p1.y)] += 1
            edge2count[Segment(p0, p1).as_norm_tuple()] += 1
    for polygon in polygon_list:
        for p0, p1 in zip(polygon, polygon[1:] + [polygon[0]]):
            #if edge2count[(p0.x, p0.y, p1.x, p1.y)] == 1:
            # This is "<=1" instead of "==1" to deal with lost edges.
            # It doesn't work.
            if edge2count[Segment(p0, p1).as_norm_tuple()] <= 1:
                retval.append(Segment(p0, p1))
    return retval


def edges_to_blocks(segment_list: [Segment], x0: int, y0: int, x1: int, y1: int,
                    coord2id: dict[Point, int], id2coord: dict[int, Point],
                    max_segments: int=100, split_horizontal=False,
                    screen=None, canvas=None) -> [[Point]]:
    """ Takes a series of blocks and returns the polygons that correspond to
    those segments plus the edges that do not belong to any polygon.

    Parameters
    ----------
    segment_list : [Segment]
        Segments to turn into polygons
    x0 : int
        X-coordinate of the left margin of the area to analyze.
    y0 : int
        Y-coordinate of the bottom margin of the area to analyze.
    x1 : int
        X-coordinate of the right margin of the area to analyze.
    y1 : int
        Y-coordinate of the top margin of the area to analyze.s
    coord2id :
        Dictionary that turns 2d coordinates into a unique integer ID. It is the
        reciprocal function of id2coord.
    id2coord:
        Dictionary that turns an integer ID into 2d coordinates. It is the
        reciprocal function of coord2id.
    split_horizontal : True
        If True the recursive step will split the space horizontally, and
        otherwise it will do so vertically. This is done to prevent the space
        from being split into thin slices - the alternative would be to perform
        the recursive step in 4 segments, but this way makes the code cleaner.

    Returns
    -------
    [[Point]], [Segment]
        List of polygons, where every polygon is a list of 2D points that
        connect to each other, plus a list of segments that do not correspond
        to any existing polygon.
    """
    #print(segment_list)
    polygons = []
    incomplete = set(map(lambda p: p.as_norm_tuple(), segment_list))

    if len(segment_list) < max_segments:
        # The number of segments is small enough that we can calculate it as
        # it is.
        g = nx.Graph()
        for segment in segment_list:
            v0 = coord2id[segment.p0]
            v1 = coord2id[segment.p1]
            g.add_edge(v0, v1)
        # We obtain the list of polygons in this list.
        cycles = nx.minimum_cycle_basis(g)
        for cycle in cycles:
            polygon = []
            assert len(cycle) >= 3, f"Cycles of size {len(cycle)} should be impossible"
            for v0, v1 in zip(cycle, cycle[1:] + [cycle[0]]):
                p0 = id2coord[v0]
                p1 = id2coord[v1]
                polygon.append(p0)
                polygon.append(p1)
                incomplete.discard((p0.x, p0.y, p1.x, p1.y))
                incomplete.discard((p1.x, p1.y, p0.x, p0.y))
            polygons.append(polygon)
    else:
        # We need to apply divide and conquer. For geometric reasons we split
        # the area in 4 pieces - splitting it in 2 leads to thin stripes of
        # the area, but I'd rather have squares
        middle_x = int(x0/2 + x1/2)
        middle_y = int(y0/2 + y1/2)
        # The algorithm will split the space into two halves plus some leftover
        # edges in the middle. These variables will hold the segments required
        # for every subdivision.
        first_segments = []
        second_segments = []
        middle_segments = []
        if split_horizontal:
            pygame.draw.line(canvas, 'green', (middle_x, y0), (middle_x, y1))
            screen.blit(canvas, (0, 0))
            pygame.display.flip()

            for s in segment_list:
                if max(s.p0.x, s.p1.x) < middle_x:
                    first_segments.append(s)
                elif middle_x < min(s.p0.x, s.p1.x):
                    second_segments.append(s)
                else:
                    middle_segments.append(s)
            first_polygon_set, first_extra = edges_to_blocks(first_segments,
                                                             x0, y0,
                                                             middle_x, y1,
                                                             coord2id, id2coord,
                                                             max_segments,
                                                             not split_horizontal,
                                                             screen, canvas)
            second_polygon_set, second_extra = edges_to_blocks(second_segments,
                                                               middle_x, y0,
                                                               x1, y1,
                                                               coord2id, id2coord,
                                                               max_segments,
                                                               not split_horizontal,
                                                               screen, canvas)
        else:
            pygame.draw.line(canvas, 'green', (x0, middle_y), (x1, middle_y))
            screen.blit(canvas, (0, 0))
            pygame.display.flip()

            for s in segment_list:
                if max(s.p0.y, s.p1.y) < middle_y:
                    first_segments.append(s)
                elif middle_y < min(s.p0.y, s.p1.y):
                    second_segments.append(s)
                else:
                    middle_segments.append(s)
            first_polygon_set, first_extra = edges_to_blocks(first_segments,
                                                             x0, y0,
                                                             x1, middle_y,
                                                             coord2id, id2coord,
                                                             max_segments,
                                                             not split_horizontal,
                                                             screen, canvas)
            second_polygon_set, second_extra = edges_to_blocks(second_segments,
                                                               x0, middle_y,
                                                               x1, y1,
                                                               coord2id, id2coord,
                                                               max_segments,
                                                               not split_horizontal,
                                                               screen, canvas)
        assert (len(first_segments) + len(second_segments) +
                len(middle_segments) == len(segment_list)),\
               "Lost polygons in partitioning"
        # Now the last part: add the middle polygons into the calculation.
        first_perimeter = get_perimeter(first_polygon_set)
        second_perimeter = get_perimeter(second_polygon_set)
        # Debug printing
        for s in first_perimeter + second_perimeter:
            p0 = s.p0.as_tuple()
            p1 = s.p1.as_tuple()
            pygame.draw.line(canvas, 'black', p0, p1, width=2)
        for s in middle_segments:
            p0 = s.p0.as_tuple()
            p1 = s.p1.as_tuple()
            pygame.draw.line(canvas, (200, 200, 200), p0, p1, width=2)
        screen.blit(canvas, (0, 0))
        pygame.display.flip()

        g = nx.Graph()
        for segment in first_perimeter + second_perimeter + middle_segments + first_extra + second_extra:
            p0, p1 = segment.as_points()
            g.add_edge(coord2id[p0], coord2id[p1])

        # Calculate the new polygons. Note that the two perimeters are also
        # included in this graph and make a giant polygon - we remove those with
        # a heuristic.
        cycles = nx.minimum_cycle_basis(g)
        polygons = first_polygon_set + second_polygon_set
        for cycle in cycles:
            polygon = []
            if len(cycle) < 2 or len(cycle) > 6:
                continue
            for v0, v1 in zip(cycle, cycle[1:] + [cycle[0]]):
                p0 = id2coord[v0]
                p1 = id2coord[v1]
                polygon.append(p0)
                polygon.append(p1)
                incomplete.discard((p0.x, p0.y, p1.x, p1.y))
                incomplete.discard((p1.x, p1.y, p0.x, p0.y))
            polygons.append(polygon)
    print(f'({x0},{y0})-({x1},{y1}): {len(polygons)}')
    a = 2
    for pol in polygons:
        as_list = list(map(lambda p: (p.x, p.y), pol))
        pygame.draw.lines(canvas, 'red', False, as_list)
        screen.blit(canvas, (0, 0))
        pygame.display.flip()
        #time.sleep(.01)
    return polygons, list(map(lambda p: Segment(Point(p[0], p[1]), Point(p[2], p[3])), incomplete))


def polygons_from_edges(edges: list[Segment], screen=None, canvas=None) -> list[list[Point]]:
    # Given a vertex, returns the edges that include that vertex.
    point2edge = defaultdict(set)
    # Counts the number of times a given edge has been used in a polygon
    edge_count = Counter()
    # Verified polygons
    polygons = []
    # We calculate the edges iteratively - edge (k-1) was added to the list at
    # a time in which edge k was unknown and so no information about those
    # edges is needed.
    canvas.fill('white')
    screen.blit(canvas, (0, 0))
    pygame.display.flip()

    for idx, edge in enumerate(edges):
        screen.blit(canvas, (0, 0))
        pygame.display.flip()
        #time.sleep(0.1)
        print(idx)
        move_on = True
        while not move_on:
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.KEYDOWN:
                    move_on = True

        print(f'Following {edge}')
        point2edge[edge.p0].add(edge.as_norm())
        point2edge[edge.p1].add(edge.as_norm())
        # We begin BFS over the known neighbors until we either find edge.p0
        # (which means a loop) or we run out of neighbors.

        #pygame.draw.line(canvas, (200, 200, 200), (edge.p0.x, edge.p0.y), (edge.p1.x, edge.p1.y))
        # The queue contains lists of vertices representing partially-built
        # polygons that we extend one vertex at a time
        queue = [[edge.p1]]
        visited = {edge}
        while len(queue) > 0:
            next_partial_polygon = queue.pop(0)
            if len(next_partial_polygon) > 6:
                continue
            elif len(next_partial_polygon) == 1:
                curr_edge = Segment(edge.p0, edge.p1)
            else:
                curr_edge = Segment(next_partial_polygon[-2], next_partial_polygon[-1])
            print(f"   Evaluating partial polygon {next_partial_polygon}")
            neighbors = point2edge[next_partial_polygon[-1]]

            # Remember: these are edges
            for n in neighbors:
                print(f"      Next neighbor: {n}")
                if (n.p0 == curr_edge.p0 and n.p1 == curr_edge.p1) or (n.p0 == curr_edge.p1 and n.p1 == curr_edge.p0):
                    # I iterated over myself - ignore and move on
                    continue
                    print(f"      Iterating over same edge - ignored")
                elif edge_count[n.as_norm()] > 1 or n.as_norm() in visited:
                    # I've visited this edge two times, which is the maximum
                    # we allow for a planar graph
                    print(f"      Edge visited two or more times")
                    continue
                elif edge.p0 == n.p1:
                    # Made a loop!
                    print(f"      Loop!: {next_partial_polygon} + {n.p1}")
                    polygon = next_partial_polygon + [n.p1]
                    polygon.append(polygon[0])
                    polygons.append(polygon)

                    for p0, p1 in zip(polygon, polygon[1:] + [polygon[0]]):
                        edge_count[Segment(p0, p1).as_norm()] += 1
                        pygame.draw.line(canvas, 'red', (p0.x, p0.y), (p1.x, p1.y))
                    queue = []
                    break
                elif edge.p0 == n.p0:
                    # Made a loop!
                    print(f"      Loop!: {next_partial_polygon} + {n.p0}")
                    polygon = next_partial_polygon + [n.p0]
                    polygon.append(polygon[0])
                    polygons.append(polygon)

                    for p0, p1 in zip(polygon, polygon[1:] + [polygon[0]]):
                        edge_count[Segment(p0, p1).as_norm()] += 1
                        pygame.draw.line(canvas, 'blue', (p0.x, p0.y), (p1.x, p1.y))
                    queue = []
                    break
                else:
                    # Keep collecting neighbors
                    if n.p0 == next_partial_polygon[-1]:
                        if n.p1 not in next_partial_polygon:
                            print(f"      Adding {n.p1} to the queue")
                            queue.append(next_partial_polygon + [n.p1])
                    else:
                        if n.p0 not in next_partial_polygon:
                            print(f"      Adding {n.p0} to the queue")
                            queue.append(next_partial_polygon + [n.p0])
    print(edge_count)
    return polygons



#if __name__ == '__main__':
def main2():
    # Initial parameters of the simulation
    width = 600
    height = 600
    radius_for_jump = 50
    radius_for_collision = 35
    # If we don't set this parameter to anything deccent, the iteration will
    # stop when all the points have reached the boundaries of the world.
    # This is the winning strategy right now, but I keep the parameter in case
    # I need to stop earlier for some reason.
    max_branching = 99999999999
    random.seed(1)

    # We want to have a certain number of "houses" for the sections of our map.
    # We also want a river, but that part of the code is currently taken out
    # until I get a better algorithm for generating it.
    river = [Point(0, 0)]
    """
    # Build a river
    while river[-1].x < 1000:
        new_point = Point(river[-1].x + random.randint(10, isle_radius),
                          river[-1].y + random.randint(-int(isle_radius/3), isle_radius))
        river.append(new_point)
    """

    houses = [(Point(30, 80), 80), (Point(130, 400), 80), (Point(700, 250), 80),
              (Point(1000, 20), 80), (Point(1300, 400), 80)]
    world = GlobalState(houses=houses, river=None)

    # Initialize PyGame
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    canvas = pygame.Surface((width, height))

    # Initialize the priority queue with a single segment.
    pqueue = [RoadQuery(0, Segment(Point(250, 250), Point(270, 270)))]
    segment_list = []
    segment_weight_list = []

    counter = 0
    while len(pqueue) > 0:
        if counter % 250 == 0:
            visualize_system(canvas, str(counter), pqueue, segment_list, segment_weight_list)
        counter += 1
        # Extract the next segment I should pay attention to, and see if the
        # local constraints allow for it.
        smallest_rq = heapq.heappop(pqueue)
        nra, state = local_constraints(smallest_rq.ra, segment_list,
                                       width=width, height=height,
                                       radius=radius_for_collision,
                                       show_reason=False)
        # If the segment was accepted we add it to the list of accepted vertices
        # and (maybe) continue the iteration
        if state == PointState.ACCEPTED or state == PointState.NO_BRANCH:
            # We want to keep the polygon in the direction in which it was found,
            # but we also need to check that it isn't already present in the
            # segment list in the other direction. Therefore this check.
            # TODO: is this check too slow?
            if nra not in segment_list and Segment(nra.p1, nra.p0) not in segment_list:
                segment_list.append(nra)
            segment_weight_list.append(smallest_rq.t)
            pygame.draw.line(canvas, 'black', (nra.p0.x, nra.p0.y), (nra.p1.x, nra.p1.y))

            # We continue the iteration if the local conditions allowed for it.
            if state == PointState.ACCEPTED:
                # Obtain the list of new points based on the global constraints.
                pdel, nra = global_goals(smallest_rq, world,
                                         radius=radius_for_jump)
                # Check that we haven't reached the maximum number of iterations
                # with this segment.
                for npdel, nnra in zip(pdel, nra):
                    if max_branching >= npdel >= 0:
                        heapq.heappush(pqueue, RoadQuery(smallest_rq.t + 1 + npdel,
                                                         nnra))
        else:
            # Segment not accepted. We show some information that hopefully
            # will show why it wasn't accepted.
            pygame.draw.line(canvas, (100, 100, 100), (nra.p0.x, nra.p0.y), (nra.p1.x, nra.p1.y))
            pygame.draw.circle(canvas, color='red', center=(nra.p1.x, nra.p1.y), radius=7)
        screen.blit(canvas, (0, 0))
        pygame.display.flip()
        # Setting move_on = False ensures that the code will only iterate one
        # frame at the time based on keypresses.
        move_on = True
        while not move_on:
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.KEYDOWN:
                    move_on = True
    screen.blit(canvas, (0, 0))
    pygame.display.flip()

    city_blocks = polygons_from_edges(segment_list, canvas=canvas, screen=screen)
    canvas.fill('white')

    """
    # Part II: Calculate the individual polygons that make up the blocks.
    print("Calculating polygons")
    canvas.fill('white')

    from collections import defaultdict
    segment_counter = Counter()
    vertex2segment = defaultdict(set)
    for segment in segment_list:
        # Adjacency list to make the BFS algorithm faster
        vertex2segment[segment.p0].add(segment)
        vertex2segment[segment.p1].add(segment)
        # We begin the iteration through neighbor edges
        # PRE: the edge is created in order [p0, p1]
        queue = [(segment.p1, 5)]
        while len(queue) > 0:
            curr_vertex, jumps_left = queue.pop(0)
            if jumps_left <= 0:
                continue
            for next_segment in vertex2segment[curr_vertex]:
                if next_segment.p0 == segment.p0:
                    # Same edge as the one I'm standing on
                    continue
                elif segment_counter[next_segment.as_norm] >1:
                    # THis edge is used up
                    continue
                # I iterate through neighboring edges
                raise NotImplementedError("Still working on it")

                queue.append((next_segment, jumps_left - 1))

    coord2id = dict()
    id2coord = dict()
    for segment in segment_list:
        key0 = Point(segment.p0.x, segment.p0.y)
        key1 = Point(segment.p1.x, segment.p1.y)
        try:
            _ = coord2id[key0]
        except KeyError:
            v0 = coord2id[key0] = len(coord2id)
            id2coord[v0] = key0
        try:
            _ = coord2id[key1]
        except KeyError:
            v1 = coord2id[key1] = len(coord2id)
            id2coord[v1] = key1
    city_blocks, extra_edges = edges_to_blocks(segment_list, 0, 0, width, height,
                                               coord2id, id2coord,
                                               max_segments=50, screen=screen, canvas=canvas)
    if len(extra_edges) > 0:
        print(f"Missing {len(extra_edges)} edges from graph")
    canvas.fill('white')
    """
    for block in city_blocks:
        print(len(block))
        xs = list(map(lambda p: p.x, block))
        ys = list(map(lambda p: p.y, block))
        center = (0.5 * min(xs) + 0.5 * max(xs), 0.5 * min(ys) + 0.5 * max(ys))
        xs = [(i - center[0]) * (1 - 0.3) + center[0] for i in xs]
        ys = [(i - center[1]) * (1 - 0.3) + center[1] for i in ys]
        polygon = list(zip(xs, ys))
        pygame.draw.lines(canvas, (122, 175, 225), False, polygon, width=1)
    # Draw the river
    # pygame.draw.lines(canvas, (122, 175, 225), False, list(map(lambda p: (p.x, p.y), river)), width=50)
    screen.blit(canvas, (0, 0))
    pygame.display.flip()


if __name__ == '__main__':
    import cProfile
    cProfile.run('main2()', 'restats.prof')
    time.sleep(5)

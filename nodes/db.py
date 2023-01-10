from peewee import *
import numpy as np
import time
from math import sqrt
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.distances import great_circle_distance_matrix
from python_tsp.distances import euclidean_distance_matrix

# Absolute Import
database = SqliteDatabase('/home/lennart/catkin_ws/src/aro/db/segments.sqlite')
# Relative Import
# database = SqliteDatabase('./db/segments.sqlite')


class UnknownField(object):
    def __init__(self, *_, **__): pass


class BaseModel(Model):
    class Meta:
        database = database


class Segments(BaseModel):
    height = FloatField(null=True)
    sm_x = FloatField(null=True)
    sm_y = FloatField(null=True)

    class Meta:
        table_name = 'segments'


class SqliteSequence(BaseModel):
    name = BareField(null=True)
    seq = BareField(null=True)

    class Meta:
        table_name = 'sqlite_sequence'
        primary_key = False


def create_segments_in_db(spalten):
    # First delete all rows in current DB
    q = Segments.delete()
    q.execute()
    # Then Bulk-Create all rows (Factor 10 faster than commented variant)
    for spalte in spalten:
        Segments.insert_many(
            spalte, fields=[Segments.sm_x, Segments.sm_y]).execute()
        # for sektor in spalte:
        #    Segments.create(sm_x=sektor[0], sm_y=sektor[1])


def get_constants():
    min_x = Segments.select().order_by(Segments.sm_x).get().sm_x
    max_x = Segments.select().order_by(Segments.sm_x.desc()).get().sm_x
    min_y = Segments.select().order_by(Segments.sm_y).get().sm_y
    max_y = Segments.select().order_by(Segments.sm_y.desc()).get().sm_y
    min_x_plus_1 = Segments.select().where(
        Segments.sm_x > min_x).order_by(Segments.sm_x).get().sm_x
    segmentsize = min_x_plus_1 - min_x # soll die segmentgröße wircklich so berechnet werden, wenn wir die sowieso in hector node definieren??
    tolerance = segmentsize / 2
    constants = {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y, "segmentsize": segmentsize, "tolerance": tolerance}
    print(constants)
    return constants


def get_current_segment(current_x, current_y, tolerance):
    # get current segment
    current_segment = Segments.select().where(Segments.sm_x.between(current_x - tolerance, current_x + tolerance),
                                              Segments.sm_y.between(current_y - tolerance, current_y + tolerance)).get()
    return current_segment


def get_segment_for_id(id):
    # get current segment
    segment = Segments.select().where(Segments.id == id).get()
    return segment


def save_heights_for_segment(segment, heights):
    mean = sum(heights) / len(heights)
    segment.height = round(mean, 2)
    segment.save()
    return segment

def save_heights_after_measurement(heights, segmentsize):
    heights_this_segment = []
    last_segment = None
    for measurement in heights:
        try:
            current_segment = get_current_segment(measurement[0], measurement[1], segmentsize / 2)
        except:
            # no current segment
            if (len(heights_this_segment) > 0):
                save_heights_for_segment(last_segment, heights_this_segment)
                heights_this_segment = []
                last_segment = None
            continue
        if current_segment == last_segment or len(heights_this_segment) == 0:
            # append heights to current array
            heights_this_segment.append(measurement[2])
            last_segment = current_segment
        else:
            # save heights for last segment
            print("Saving" + str(len(heights_this_segment)) + " Heights:" + str(heights_this_segment) + " for Segment: " + str(last_segment.id))
            save_heights_for_segment(last_segment, heights_this_segment)
            heights_this_segment = [measurement[2]]
            last_segment = current_segment

def calculate_first_point(min_x, min_y, max_y, margin, segment_size):
    # start on segment with minimal x and minimal y
    #min_x =  min_x + (segment_size / 2)
    print("First Point:", min_x, min_y - (segment_size / 2 + margin))
    return [min_x, min_y - (segment_size / 2 + margin)]


def calculate_next_point(min_x, max_x, min_y, max_y, segmentsize, tolerance, margin, current_x, current_y, debug=False):
    if debug:
        print("Current Point:", current_x, current_y)
        print("Min X:", min_x)
        print("Max X:", max_x)
        print("Min Y:", min_y)
        print("Max Y:", max_y)
        print("Segment Size:", segmentsize)
        print("Tolerance:", tolerance)
        print("Margin:", margin)

    # called when on start point as test2()
    if (current_y < min_y - tolerance):
        # drone is southern (negative y) of field
        # fly north
        # check if most nothern segment is already measured
        try:
            segment_same_column_north = Segments.select().where(
                (Segments.sm_x.between(current_x - tolerance, current_x + tolerance)) & (Segments.height.is_null())).order_by(Segments.sm_y.desc()).get()
            nextpoint = [segment_same_column_north.sm_x,
                         segment_same_column_north.sm_y + segmentsize / 2 + margin]
            print("Nextpoint 3:", nextpoint)
            return nextpoint
        except Exception as e:
            # all segments in this column are measured
            # fly to next column
            try:
                segment_next_column_south = Segments.select().where(
                    (Segments.sm_x.between(current_x + segmentsize - tolerance, current_x + segmentsize + tolerance)) & (Segments.height.is_null())).order_by(Segments.sm_y).get()
                nextpoint = [segment_next_column_south.sm_x,
                             segment_next_column_south.sm_y - (segmentsize / 2 + margin)]
                print("Nextpoint 4:", nextpoint)
                return nextpoint
            except:
                # No field left to measure
                print("No field left to measure")
                # what should we do?
                return []

    elif (current_y > max_y + tolerance):
        # drone is northern (positive y) of field
        # fly south
        # check if most southern segment is already measured
        try:
            segment_same_column_south = Segments.select().where(
                (Segments.sm_x.between(current_x - tolerance, current_x + tolerance)) & (Segments.height.is_null())).order_by(Segments.sm_y).get()
            nextpoint = [segment_same_column_south.sm_x,
                         segment_same_column_south.sm_y - (segmentsize / 2 + margin)]
            print("Nextpoint 1:", nextpoint)
            return nextpoint
        except:
            # all segments in this column are measured
            # fly to next column
            try:
                segment_next_column_north = Segments.select().where(
                    (Segments.sm_x.between(current_x + segmentsize - tolerance, current_x + segmentsize + tolerance)) & (Segments.height.is_null())).order_by(Segments.sm_y.desc()).get()
                nextpoint = [segment_next_column_north.sm_x,
                             segment_next_column_north.sm_y + segmentsize / 2 + margin]
                print("Nextpoint 2:", nextpoint)
                return nextpoint
            except:
                # No field left to measure
                print("No field left to measure")
                # what should we do?
                return []
    elif (current_y > min_y - tolerance and current_y < max_y + tolerance):
        # drone is on field
        # what should we do?
        return []

def get_segments_for_path(path, xy_list):
    segments = []
    for i in path:
        segments.append(xy_list[i])
    print(segments)
    return segments

def generate_path():
    # function to generate shortest path between points which should be fertilized
    # classic traveling salesman problem
    # solved via Bellman–Held–Karp algorithm
    # this allows us to only fly to points which should be fertilized and not fly over the complete field
    current_position = [0, 0] # retrieve via drone pose
    points = [current_position]
    segments = Segments.select(Segments.sm_x, Segments.sm_y) # should be passed to this function
    for segment in segments:
        points.append([segment.sm_x, segment.sm_y])
    # generate distance matrix
    xy_list = np.asarray(points)
    # Alle Varianten sind ungefähr gleich schnell.. Testen, welche uns bei der Drohne am besten gefällt
    # Variante 1
    dist = lambda p1, p2: sqrt(((p1-p2)**2).sum())
    dm = np.asarray([[dist(p1, p2) for p2 in xy_list] for p1 in xy_list])
    distance_matrix = great_circle_distance_matrix(xy_list)
    permutation, distance = solve_tsp_dynamic_programming(dm)
    v1 = get_segments_for_path(permutation, points)
    # Variante 2
    permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
    v2 = get_segments_for_path(permutation, points)
    # Variante 3
    distance_matrix = euclidean_distance_matrix(xy_list)
    permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
    v3 = get_segments_for_path(permutation, points)
    return v1, v2, v3
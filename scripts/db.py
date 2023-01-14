from peewee import *
import numpy as np
import time
from math import sqrt
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_simulated_annealing
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
    """
    ORM Database Class to present our Segments 

    Each Segment is defined with an unique ID, x and y coordinates of its center and - after inital measurement - a height value
    """

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
    """
    Function to create multiple segments in the database

    Parameters
    ----------
    spalten: list
        List of lists of segments to be created in the database
        like: [[(x1,y1), (x2,y2), (x3,y3)], [(x1,y1), (x2,y2), (x3,y3)]]
    """

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
    """
    Function to generate constants depending on already existing segments in the database

    Returns
    -------
    constants: dict
        Dictionary of constants like min_x, max_x, min_y, max_y, segmentsize and tolerance
    """

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
    """
    Function to get the current segment from the database
    
    Parameters
    ----------
    current_x: float
        x coordinate of the current position
    current_y: float
        y coordinate of the current position
    tolerance: float
        tolerance to find the current segment, normally half of the segmentsize
    
    Returns
    -------
    current_segment: Segments
        segment in which the current position is located
    """

    # get current segment
    current_segment = Segments.select().where(Segments.sm_x.between(current_x - tolerance, current_x + tolerance),
                                              Segments.sm_y.between(current_y - tolerance, current_y + tolerance)).get()
    return current_segment


def get_segment_for_id(id):
    """
    Function to get the segment with the given id from the database
    
    Parameters
    ----------
    id: int
        id of the segment to be returned
        
    Returns
    -------
    segment: Segments
        segment with the given id
    """
    # get current segment
    segment = Segments.select().where(Segments.id == id).get()
    return segment


def save_heights_for_segment(segment, heights):
    """
    Function to save the heights for a given segment

    Parameters
    ----------
    segment: Segments
        segment for which the heights should be saved
    heights: list
        list of heights to be saved for the given segment, like [1.2, 1.3, 1.4]
    
    Returns
    -------
    segment: Segments
        segment with the saved heights
    """
    mean = sum(heights) / len(heights)
    segment.height = round(mean, 2)
    segment.save()
    return segment

def save_heights_after_measurement(heights, segmentsize):
    """
    Function to save the heights for multiple segments after a measurement

    Parameters
    ----------
    heights: list
        list of heights to be saved, like [[x1, y1, 1.2], [x2, y2, 1.3], [x3, y3, 1.4]]
    segmentsize: float
        size of the segments
    
    Returns
    -------
    None
    """
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
    """
    Function to calculate the first point for the measurement
    
    Parameters
    ----------
    min_x: float
        minimal x coordinate of the segments center
    min_y: float
        minimal y coordinate of the segments center
    max_y: float
        maximal y coordinate of the segments center
    margin: float
        margin to the border of the map (northern or southern), in meters
    segment_size: float
        size of the segments
        """
    # start on segment with minimal x and minimal y
    #min_x =  min_x + (segment_size / 2)
    print("First Point:", min_x, min_y - (segment_size / 2 + margin))
    return [min_x, min_y - (segment_size / 2 + margin)]


def calculate_next_point(min_x, max_x, min_y, max_y, segmentsize, tolerance, margin, current_x, current_y, debug=False):
    """
    Function to calculate the next point for the measurement
    
    Parameters
    ----------
    min_x: float
        minimal x coordinate of the segments center
    max_x: float
        maximal x coordinate of the segments center
    min_y: float
        minimal y coordinate of the segments center
    max_y: float
        maximal y coordinate of the segments center
    segmentsize: float
        size of the segments
    tolerance: float
        tolerance to find the current segment, normally half of the segmentsize
    margin: float
        margin to the border of the map (northern or southern), in meters
    current_x: float
        x coordinate of the current position
    current_y: float
        y coordinate of the current position
    debug: bool
        if True, debug information will be printed
    Returns
    -------
    next_point: list
        list with the next point, like [x, y]
    """
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
    """
    Function to get the segments for a given path
    Parameters
    ----------
    path: list
        list with the indices of the segments like [1, 2, 3, 4]
    xy_list: list
        list with the coordinates of the segments like [[1, 2], [3, 4], [5, 6], [7, 8]]
    Returns
    -------
    segments: list
        list with the coordinates of the segments for the given path
    """
    segments = []
    for i in path:
        segments.append(xy_list[i])
    print(segments)
    return segments

def generate_path(homeposition, min_height = 0, max_height = 500):
    """
    Function to generate the shortest path between the points which should be fertilized
    Classic traveling salesman problem
    Originally solved via Bellman–Held–Karp algorithm, but this is too slow for our use case
    Therefore we use the Simulated Annealing algorithm
    This allows us to fly only to segments which need fertilization, not to all segments

    Parameters
    ----------
    homeposition: list
        list with the homeposition like [x, y]
    min_height: int
        minimal height of the segments which should be fertilized
    max_height: int
        maximal height of the segments which should be fertilized
    
    Returns
    -------
    path: list
        list with the indices of the segments like [[1, 2], [3, 4], [5, 6], [7, 8]]

    """

    startpoint = [homeposition[0], homeposition[1]]
    points = [startpoint]
    segments = Segments.select(Segments.sm_x, Segments.sm_y).where(Segments.height > min_height, Segments.height < max_height) # should be passed to this function
    for segment in segments:
        points.append([segment.sm_x, segment.sm_y])
    # generate distance matrix
    xy_list = np.asarray(points)
    # All variants are nearly the same speed, select the one we like the most
    # V1
    dist = lambda p1, p2: sqrt(((p1-p2)**2).sum())
    dm = np.asarray([[dist(p1, p2) for p2 in xy_list] for p1 in xy_list])
    distance_matrix = great_circle_distance_matrix(xy_list)
    permutation, distance = solve_tsp_simulated_annealing(dm)
    v1 = get_segments_for_path(permutation, points)
    # V2
    permutation, distance = solve_tsp_simulated_annealing(distance_matrix)
    v2 = get_segments_for_path(permutation, points)
    # V3
    distance_matrix = euclidean_distance_matrix(xy_list)
    permutation, distance = solve_tsp_simulated_annealing(distance_matrix)
    v3 = get_segments_for_path(permutation, points)

    return v1, v2, v3
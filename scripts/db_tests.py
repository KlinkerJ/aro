from peewee import *
import time

# changed to relative import
database = SqliteDatabase('../db/segments.sqlite')


class UnknownField(object):
    def __init__(self, *_, **__): pass


class BaseModel(Model):
    class Meta:
        database = database


class Segments(BaseModel):
    height = IntegerField(null=True)
    sm_x = IntegerField(null=True)
    sm_y = IntegerField(null=True)

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


def test():
    feldgröße = 2/2 - 0.1
    now_x = 56.5
    now_y = 16.8
    feld = Segments.select().where(Segments.sm_x.between(now_x-feldgröße, now_x+feldgröße),
                                   Segments.sm_y.between(now_y-feldgröße, now_y+feldgröße)).get()
    print(feld.sm_x, feld.sm_y)
    feld.height = 10
    feld.save()


def algorithm():
    min_x = Segments.select().order_by(Segments.sm_x).get().sm_x
    max_x = Segments.select().order_by(Segments.sm_x.desc()).get().sm_x
    min_y = Segments.select().order_by(Segments.sm_y).get().sm_y
    max_y = Segments.select().order_by(Segments.sm_y.desc()).get().sm_y
    min_x_plus_1 = Segments.select().where(
        Segments.sm_x > min_x).order_by(Segments.sm_x).get().sm_x
    segmentsize = min_x_plus_1 - min_x
    tolerance = segmentsize / 2
    margin = 2  # 2m margin, how much southern should the drone start to first segment

    # get first point
    first_point = calculate_first_point(min_x, min_y, max_y, margin)

    finished = False
    current_position = first_point
    while not finished:
        # get next point
        next_point = calculate_next_point(
            min_x, max_x, min_y, max_y, segmentsize, tolerance, margin, current_position[0], current_position[1])
        # fly to next point
        # simulate flight
        print("Flying to next point")
        time.sleep(10)
        current_position = next_point


def calculate_first_point(min_x, min_y, max_y, margin):
    # start on segment with minimal x and minimal y
    print("First Point:", min_x, min_y - (margin + 1))
    return [min_x, min_y - (margin + 1)]


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
                Segments.sm_x.between(current_x - tolerance, current_x + tolerance), Segments.height == None).order_by(Segments.sm_y.desc()).get()
            nextpoint = [segment_same_column_north.sm_x,
                         segment_same_column_north.sm_y + margin + 1]
            print("Nextpoint:", nextpoint)
            return nextpoint
        except Exception as e:
            # all segments in this column are measured
            # fly to next column
            try:
                segment_next_column_south = Segments.select().where(
                    Segments.sm_x.between(current_x + segmentsize - tolerance, current_x + segmentsize + tolerance), Segments.height == None).order_by(Segments.sm_y).get()
                nextpoint = [segment_next_column_south.sm_x,
                             segment_next_column_south.sm_y - (margin + 1)]
                print("Nextpoint:", nextpoint)
                return nextpoint
            except:
                # No field left to measure
                print("No field left to measure")
                # what should we do?
                return

    elif (current_y > max_y + tolerance):
        # drone is northern (positive y) of field
        # fly south
        # check if most southern segment is already measured
        try:
            segment_same_column_south = Segments.select().where(
                Segments.sm_x.between(current_x - tolerance, current_x + tolerance), Segments.height == None).order_by(Segments.sm_y).get()
            nextpoint = [segment_same_column_south.sm_x,
                         segment_same_column_south.sm_y - (margin + 1)]
            print("Nextpoint:", nextpoint)
            return nextpoint
        except:
            # all segments in this column are measured
            # fly to next column
            try:
                segment_next_column_north = Segments.select().where(
                    Segments.sm_x.between(current_x + segmentsize - tolerance, current_x + segmentsize + tolerance), Segments.height == None).order_by(Segments.sm_y.desc()).get()
                nextpoint = [segment_next_column_north.sm_x,
                             segment_next_column_north.sm_y + margin + 1]
                print("Nextpoint:", nextpoint)
                return nextpoint
            except:
                # No field left to measure
                print("No field left to measure")
                # what should we do?
                return
    elif (current_y > min_y - tolerance and current_y < max_y + tolerance):
        # drone is on field
        # what should we do?
        return


if __name__ == '__main__':
    # test2()
    # calculate_next_point()
    algorithm()

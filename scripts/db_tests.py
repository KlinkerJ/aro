from peewee import *

database = SqliteDatabase('/home/lennart/catkin_ws/src/aro/db/segments.sqlite')

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
    for spalte in spalten:
        for sektor in spalte:
            Segments.create(sm_x = sektor[0], sm_y=sektor[1])

def test():
    feldgröße = 2/2 - 0.1
    now_x = 56.5
    now_y = 16.8
    feld = Segments.select().where(Segments.sm_x.between(now_x-feldgröße, now_x+feldgröße) & Segments.sm_y.between(now_y-feldgröße, now_y+feldgröße)).get()
    print(feld.sm_x, feld.sm_y)
    feld.height = 10
    feld.save()

def test2():
    # wir starten unten rechts in der Ecke
    feld_min_x = Segments.select().order_by(Segments.sm_x).get()
    feld_min_y = Segments.select().order_by(Segments.sm_y).get()
    feld_max_y_for_min_x = Segments.select().where(Segments.sm_x == feld_min_x.sm_x).order_by(Segments.sm_y.desc()).get()
    print(feld_min_x.sm_x, feld_min_y.sm_y, feld_max_y_for_min_x.sm_y)
    # erster Punkt: x = feld_min_x.sm_x, y = feld_min_y.sm_y - 2m

test2()
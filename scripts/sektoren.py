import csv

def getSpaltenElements(x, yMin, yMax, maxLength): 
    elements = []
    elements.append([x, yMin + maxLength / 2 ])

    while elements[-1][1] < yMax - maxLength:
        elements.append([ elements[-1][0], elements[-1][1] + maxLength ])
    
    return elements


def getSektorForEckpunkte(p1, p2, p3, p4, maxLength):

    spalten = []
    x = p1['x'] - maxLength / 2
    spalten.append(getSpaltenElements(x, p1['y'], p2['y'], maxLength))

    while x > p4['x'] + maxLength:
        x = x - maxLength
        spalten.append(getSpaltenElements(x, p1['y'], p2['y'], maxLength))

    return spalten

def writeToCSV(spalten):
    f = open('spalten.csv', 'w')

    writer = csv.writer(f)
    writer.writerow(["x", "y"])

    for spalte in spalten:
        for sektor in spalte:
            writer.writerow(sektor)

    f.close()

p1 = {'x': 100, 'y': 0 }
p2 = {'x': 100, 'y': 20 }
p3 = {'x': 50, 'y': 20 }
p4 = {'x': 50, 'y': 0 }

spalten = getSektorForEckpunkte(p1, p2, p3, p4, 2)
print(spalten)
#writeToCSV(spalten)


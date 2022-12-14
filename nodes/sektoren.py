def getSpaltenElements(x, yMin, yMax, maxLength):
    elements = []
    elements.append([x, yMin + maxLength / 2])

    while elements[-1][1] < yMax - maxLength:
        elements.append([elements[-1][0], elements[-1][1] + maxLength])

    return elements


def getSektorForEckpunkte(p1, p2, p3, p4, maxLength):

    spalten = []
    x = p1['x'] - maxLength / 2
    spalten.append(getSpaltenElements(x, p1['y'], p2['y'], maxLength))

    while x > p4['x'] + maxLength:
        x = x - maxLength
        spalten.append(getSpaltenElements(x, p1['y'], p2['y'], maxLength))

    return spalten

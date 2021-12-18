import math

class Point():
    def __init__(self, identity, x, y):
        self.identity = identity
        self.x = x
        self.y = y

def dist(point_1, point_2):
    return math.sqrt(((point_2[1]-point_1[1])**2)+((point_2[0]-point_1[0])**2))

def closest_brute_force(points):
    min_dist = float("inf")
    point_1 = None
    point_2 = None
    for i in range(len(points)):
        for j in range(i+1, len(points)):
            d = dist(points[i], points[j])
            if d < min_dist:
                min_dist = d
                point_1 = points[i]
                point_2 = points[j]
    return point_1, point_2, min_dist


def rec(xsorted, ysorted):
    n = len(xsorted)
    if n <= 3:
        return closest_brute_force(xsorted)
    else:
        midpoint = xsorted[n//2]
        xsorted_left = xsorted[:n//2]
        xsorted_right = xsorted[n//2:]
        ysorted_left = []
        ysorted_right = []
        for point in ysorted:
            ysorted_left.append(point) if (point[0] <= midpoint[0]) else ysorted_right.append(point)
            # print(ysorted_left)
        (p1_left, p2_left, delta_left) = rec(xsorted_left, ysorted_left)
        (p1_right, p2_right, delta_right) = rec(xsorted_right, ysorted_right)
        (p1, p2, delta) = (p1_left, p2_left, delta_left) if (delta_left < delta_right) else (p1_right, p2_right, delta_right)
        in_band = [point for point in ysorted if midpoint[0]-delta < point[0] < midpoint[0]+delta]
        for i in range(len(in_band)):
            for j in range(i+1, min(i+7, len(in_band))):
                d = dist(in_band[i], in_band[j])
                if d < delta:
                    print(in_band[i], in_band[j])
                    (p1, p2, delta) = (in_band[i], in_band[j], d)
        return p1, p2, delta


def closest_final(points):
    xsorted = sorted(points, key=lambda point: point[0])
    ysorted = sorted(points, key=lambda point: point[1])
    id_sorted = sorted(points, key=lambda point: point[2])
    return rec(xsorted, ysorted)

def closest_point(p1,p2):
    # choose tuple or lists accordingly please
    points = [p1,p2]
    # points = ([2,3,"id1"],[3,2,"id2"],[3,5,"id3"],[4,5,"id4"],[1,5,"id5"],[5,1,"id6"]) # for tuple
    print(type(points))
    #print(closest_final(points))
    return closest_final(points)
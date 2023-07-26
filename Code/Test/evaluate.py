import math
def distance_between_points(p, p1):
    return math.sqrt((p1[0] - p[0])**2 + (p1[1] - p[1])**2)

while True:
    a = float(input("a: "))
    b = float(input("a: "))
    c = float(input("a: "))
    d = float(input("a: "))
    e = float(input("a: "))
    f = float(input("a: "))
    print(distance_between_points([a,b],[c,d]))
    print(distance_between_points([a,b],[e,f]))
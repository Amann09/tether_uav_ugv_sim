way = [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7), (1, 7), (1, 8), (2, 8), (3, 8), (4, 9), (4, 10), (4, 11), (4, 12), (3, 12), 
(2, 11), (3, 11), (3, 10), (3, 9), (2, 9), (2, 10), (1, 11), (1, 10), (1, 9), (0, 8), (0, 9), (0, 10), (0, 11), (0, 12), (0, 13), (1, 13), (1, 12), 
(2, 12), (2, 13), (3, 13), (4, 13), (5, 13), (5, 14), (4, 14), (3, 14), (2, 14), (1, 14), (0, 14), (0, 15), (1, 15), (2, 15), (3, 15), (4, 15), 
(5, 15), (6, 15), (6, 14), (6, 13), (5, 12), (6, 12), (7, 12), (8, 12), (9, 12), (10, 12), (11, 12), (10, 11), (11, 11), (11, 10), (10, 10), (9, 11), 
(8, 11), (8, 10), (9, 10), (10, 9), (10, 8), (9, 8), (9, 9), (8, 9), (8, 8), (8, 7), (7, 7), (7, 8), (6, 8), (5, 8), (4, 8), (4, 7), (4, 6), (4, 5), 
(3, 5), (3, 4), (3, 3), (2, 4), (2, 5), (3, 6), (3, 7), (2, 7), (2, 6), (1, 6), (1, 5), (1, 4), (2, 3), (1, 3), (1, 2), (1, 1), (1, 0), (2, 0), (2, 1), 
(2, 2), (3, 2), (4, 3), (5, 3), (4, 4), (5, 5), (5, 6), (5, 7), (6, 7), (7, 6), (6, 6), (6, 5), (5, 4), (6, 4), (7, 4), (6, 3), (6, 2), (6, 1), (5, 1), 
(5, 2), (4, 2), (3, 1), (3, 0), (4, 0), (4, 1), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0), (10, 0), (11, 0), (12, 0), (13, 0), (14, 0), (15, 0), (15, 1), 
(15, 2), (15, 3), (15, 4), (15, 5), (15, 6), (15, 7), (15, 8), (15, 9), (15, 10), (15, 11), (15, 12), (15, 13), (14, 13), (13, 13), (12, 13), (12, 14), 
(13, 14), (14, 14), (15, 14), (15, 15), (14, 15), (13, 15), (12, 15), (11, 15), (10, 15), (11, 14), (10, 14), (9, 14), (8, 14), (9, 15), (8, 15), (7, 15),
(7, 14), (7, 13), (8, 13), (9, 13), (10, 13), (11, 13), (12, 12), (12, 11), (12, 10), (13, 11), (13, 12), (14, 12), (14, 11), (13, 10), (14, 10), (14, 9), 
(14, 8), (14, 7), (14, 6), (13, 6), (13, 5), (14, 5), (14, 4), (14, 3), (14, 2), (14, 1), (13, 1), (12, 1), (11, 1), (10, 1), (9, 1), (8, 1), (7, 1), (7, 2), 
(7, 3), (8, 2), (8, 3), (8, 4), (7, 5), (8, 6), (8, 5), (9, 5), (9, 6), (9, 7), (10, 7), (10, 6), (10, 5), (11, 6), (12, 6), (12, 5), (12, 4), (13, 4), (13, 3),
(12, 3), (13, 2), (12, 2), (11, 3), (10, 3), (11, 2), (10, 2), (9, 2), (9, 3), (9, 4), (10, 4), (11, 4), (11, 5)]

drone, rover = [], []

for i in range(len(way)):
    if i%2 == 0:
        rover.append(way[i])
    else:
        drone.append(way[i])

print(f"DRONE WAY: {drone}")
print(f"ROVER WAY: {rover}")

if (1, 0) in drone:
    print(True)
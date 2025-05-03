import math
import waypoints_to_plan_conversion

# Reference point in Cartesian coordinates and corresponding latitude and longitude
reference_cartesian = (1, 0)
reference_geo = (47.397970132629425, 8.546176058903313)

# Conversion factors
meters_per_degree_latitude = 111000
meters_per_degree_longitude = 111000 * math.cos(math.radians(reference_geo[0]))

def convert_cartesian_to_geo(cartesian_points):
    converted_points = []
    
    for point in cartesian_points:
        # Calculate differences in meters
        delta_x = point[0] - reference_cartesian[0]
        delta_y = point[1] - reference_cartesian[1]
        
        # Convert to changes in latitude and longitude
        delta_lat = delta_x / meters_per_degree_latitude
        delta_lon = delta_y / meters_per_degree_longitude
        
        # Calculate new latitude and longitude
        new_lat = reference_geo[0] + delta_lat
        new_lon = reference_geo[1] + delta_lon
        
        converted_points.append((new_lat, new_lon))
    
    return converted_points

# Example usage:
cartesian_points = waypoints_to_plan_conversion.initially
# cartesian_points = [(0, 2), (0, 4), (0, 6), (1, 7), (2, 8), (4, 9), (4, 11), (3, 12), (3, 11), (3, 9), (2, 10), (1, 10), (0, 8), 
#                     (0, 10), (0, 12), (1, 13), (2, 12), (3, 13), (5, 13), (4, 14), (2, 14), (0, 14), (1, 15), (3, 15), (5, 15), 
#                     (6, 14), (5, 12), (7, 12), (9, 12), (11, 12), (11, 11), (10, 10), (8, 11), (9, 10), (10, 8), (9, 9), (8, 8), 
#                     (7, 7), (6, 8), (4, 8), (4, 6), (3, 5), (3, 3), (2, 5), (3, 7), (2, 6), (1, 5), (2, 3), (1, 2), (1, 0), (2, 1), 
#                     (3, 2), (5, 3), (5, 5), (5, 7), (7, 6), (6, 5), (6, 4), (6, 3), (6, 1), (5, 2), (3, 1), (4, 0), (5, 0), (7, 0), 
#                     (9, 0), (11, 0), (13, 0), (15, 0), (15, 2), (15, 4), (15, 6), (15, 8), (15, 10), (15, 12), (14, 13), (12, 13), 
#                     (13, 14), (15, 14), (14, 15), (12, 15), (10, 15), (10, 14), (8, 14), (8, 15), (7, 14), (8, 13), (10, 13), (12, 12), 
#                     (12, 10), (13, 12), (14, 11), (14, 10), (14, 8), (14, 6), (13, 5), (14, 4), (14, 2), (13, 1), (11, 1), (9, 1), (7, 1), 
#                     (7, 3), (8, 3), (7, 5), (8, 5), (9, 6), (10, 7), (10, 5), (12, 6), (12, 4), (13, 3), (13, 2), (11, 3), (11, 2), (9, 2), 
#                     (9, 4), (11, 4)]
# print(cartesian_points)
angles = waypoints_to_plan_conversion.angles
# print(angles)

geo_points = convert_cartesian_to_geo(cartesian_points)

# print(cartesian_points[71:])
# print(geo_points)

# for cartesian, geo in zip(cartesian_points, geo_points):
#     print(f"Cartesian: {cartesian} -> Geo: {geo}")

# Combine the points with their corresponding angles
final_list = [(geo_points[i][0], geo_points[i][1], angles[i]) for i in range(len(geo_points))]
print(final_list)

import coordinate_to_long_lat
coordinates = coordinate_to_long_lat.final_list
# print(coordinate_to_long_lat)

pre_template = """
{
  "fileType": "Plan",
  "geoFence": {
    "circles": [],
    "polygons": [],
    "version": 2
  },
  "groundStation": "QGroundControl",
  "mission": {
    "cruiseSpeed": 15,
    "firmwareType": 12,
    "globalPlanAltitudeMode": 1,
    "hoverSpeed": 5,
    "items": [
        {
            "autoContinue": true,
            "command": 178,
            "doJumpId": 1,
            "frame": 2,
            "params": [
            0,
            0.5,
            -1,
            0,
            0,
            0,
            0
            ],
            "type": "SimpleItem"
        },
"""

post_template = """
    ],
    "plannedHomePosition": [
      47.39798846525458,
      8.546177170566494,
      null
    ],
    "vehicleType": 2,
    "version": 2
  },
  "rallyPoints": {
    "points": [],
    "version": 2
  },
  "version": 1
}
"""

# Template string with coma
template_1 = """
        {{
            "autoContinue": true,
            "command": 16,
            "doJumpId": 4,
            "frame": 3,
            "params": [
                5,
                0,
                0,
                {c},
                {a}, 
                {b},
                3
            ],
            "type": "SimpleItem"
        }},
"""

# Template string without coma
template_2 = """
        {{
            "autoContinue": true,
            "command": 16,
            "doJumpId": 4,
            "frame": 3,
            "params": [
                5,
                0,
                0,
                {c},
                {a}, 
                {b},
                3
            ],
            "type": "SimpleItem"
        }}
"""

# Generate the full string by replacing (a, b, c) with each pair from the list
result_string = ""
for a, b, c in coordinates:
    if (a, b, c) == coordinates[-1]:
        result_string += template_2.format(a=a, b=b, c=c)
    else:
        result_string += template_1.format(a=a, b=b, c=c)


# Write the result to a txt file
# file_path = '/output.txt'
# file_path = '/home/moonlab/Desktop/New-Tether/tether_drone_rover_sim-main/scripts/output.txt'
file_path = '/home/moonlab/Desktop/New-Tether/tether_drone_rover_sim-main/scripts/output.plan'
with open(file_path, 'w') as file:
    file.write(pre_template + result_string + post_template)


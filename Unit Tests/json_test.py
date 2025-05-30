import json
import os
import sys


file_path = os.path.join(os.path.dirname(__file__), 'markers_example.json')

try:
    with open(file_path, 'r') as file:
        data = json.load(file)

    print("Successfully read markers_example.json")
    print(f"Markers loaded: {len(data)}")
    print("For simplicity, we are testing 2-D locations")

    # Cast to int for indexing
    max_x = max(int(marker['x']) for marker in data.values())
    max_y = max(int(marker['y']) for marker in data.values())

    # Create empty grid
    grid = [['*' for _ in range(max_x + 1)] for _ in range(max_y + 1)]

    # Place marker IDs on grid
    for marker_id, marker in data.items():
        x = int(marker['x'])
        y = int(marker['y'])
        grid[y][x] = marker_id


    print("\nMarker locations on grid (Y-down, X-right):")
    for row in grid:
        print(' '.join(row))

except Exception as e:
    print(f"Failed to read JSON file: {e}")

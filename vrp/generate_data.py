import numpy as np
import random

def generate_synthetic_data(center_lat, center_lon, area_radius_degrees=0.05, num_customers=25, num_vehicles=5, capacity=35,avg_demand=6):
    print(f"Generating data centered at ({center_lat:.4f}, {center_lon:.4f})")
    data = {}

    depot_coords = (center_lat, center_lon) 

    lat_bounds = (center_lat - area_radius_degrees, center_lat + area_radius_degrees)
    lon_bounds = (center_lon - area_radius_degrees, center_lon + area_radius_degrees)

    data['num_vehicles'] = num_vehicles
    data['depot'] = 0 
    data['vehicle_capacities'] = [capacity] * num_vehicles

    # --- Generate Locations ---
    locations = [depot_coords]
    for _ in range(num_customers):
        lat = random.uniform(lat_bounds[0], lat_bounds[1])
        lon = random.uniform(lon_bounds[0], lon_bounds[1])
        locations.append((lat, lon))
    data['locations'] = locations

    # --- Generate Demands ---
    demands = [0] # Depot demand is 0
    min_d = max(1, round(avg_demand * 0.5)) # Roughly 50% of avg, min 1
    max_d = max(min_d + 1, round(avg_demand * 1.5)) # Roughly 150% of avg, ensure max > min

    print(f"Generating demands between {min_d} and {max_d} (based on avg: {avg_demand})")

    for _ in range(num_customers):
        demands.append(random.randint(min_d, max_d)) 

    data['demands'] = demands

    print(f"Generated {len(locations)} locations ({num_customers} customers + 1 depot).")
    print(f"Depot Coordinates: {locations[0]}")
    print(f"Number of vehicles: {data['num_vehicles']}")
    print(f"Vehicle capacity: {capacity}")
    print(f"Total customer demand: {sum(demands)}")

    return data

if __name__ == '__main__':
    test_center_lat = 12.9165
    test_center_lon = 79.1325 
    test_avg_demand = 10
    print(f"Testing data generation centered at ({test_center_lat}, {test_center_lon})...with avg demand {test_avg_demand}...")
    generated_data = generate_synthetic_data(
        center_lat=test_center_lat, 
        center_lon=test_center_lon, 
        num_customers=25,
        num_vehicles=5,
        capacity=35,
        avg_demand=test_avg_demand
    )

    if generated_data:
        print("\n--- Sample Generated Data ---")
        print(f"Depot: {generated_data['locations'][0]}")
        print(f"First 3 Customers: {generated_data['locations'][1:4]}")
        print(f"First 10 Demands: {generated_data['demands'][:10]}")
        print(f"Vehicle Capacities: {generated_data['vehicle_capacities']}")
        print("--- End Test ---")
    else:
        print("Data generation failed.")
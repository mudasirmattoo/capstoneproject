import os
import folium
import webbrowser
import openrouteservice
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from dotenv import load_dotenv
from .generate_data import generate_synthetic_data

def compute_ors_distance_matrix(locations, api_key):
    """Creates a distance matrix using OpenRouteService API."""
    num_locations = len(locations)
    coords_ors_format = [[lon, lat] for lat, lon in locations]
    client = openrouteservice.Client(key=api_key)
    print(f"Requesting distance matrix for {num_locations} locations from OpenRouteService...")
    try:
        matrix_response = client.distance_matrix(
            locations=coords_ors_format,
            metrics=['distance'],
            units='m',
            profile='driving-car'
        )
        if 'distances' not in matrix_response:
            print("Error: 'distances' key not found in ORS response.")
            print("Response:", matrix_response)
            return None
        distance_matrix = [[int(round(d)) for d in row] for row in matrix_response['distances']]
        print("ORS distance matrix retrieved successfully.")
        return distance_matrix
    except openrouteservice.exceptions.ApiError as e:
        print(f"ORS API Error: {e}")
        if '403' in str(e): print("Check API key validity or subscription quota.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during ORS request: {e}")
        return None

# --- Helper Function: Visualization ---
def visualize_solution_map(data, manager, routing, solution, filename="vellore_routes.html"):
    """Saves a Folium map visualization of the routes to an HTML file."""
    print("\nGenerating map visualization...")
    locations = data['locations']
    depot_coords = locations[data['depot']]
    # Center map on depot coords (which should be the geocoded center)
    map_center = [depot_coords[0], depot_coords[1]]
    route_map = folium.Map(location=map_center, zoom_start=13)
    # Depot marker
    folium.Marker(
        location=depot_coords, popup="Depot (Node 0)", tooltip="Depot",
        icon=folium.Icon(color='red', icon='industry', prefix='fa')
    ).add_to(route_map)
    # Customer markers
    for node_index in range(len(locations)):
        if node_index == data['depot']: continue
        coords = locations[node_index]
        demand = data['demands'][node_index]
        folium.Marker(
            location=coords, popup=f"Customer {node_index}<br>Demand: {demand}",
            tooltip=f"Customer {node_index}", icon=folium.Icon(color='blue', icon='user', prefix='fa')
        ).add_to(route_map)
    # Route lines
    colors = ['green', 'purple', 'orange', 'darkred', 'lightred', 'beige', 'darkblue', 'darkgreen',
              'cadetblue', 'darkpurple', 'pink', 'lightblue', 'lightgreen', 'gray', 'black', 'lightgray']
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_coords = []
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_coords.append(locations[node_index])
            index = solution.Value(routing.NextVar(index))
        node_index = manager.IndexToNode(index)
        route_coords.append(locations[node_index])
        if len(route_coords) > 2:
            folium.PolyLine(
                route_coords, color=colors[vehicle_id % len(colors)], weight=3,
                opacity=0.8, tooltip=f"Vehicle {vehicle_id}"
            ).add_to(route_map)
    try:
        route_map.save(filename)
        print(f"Map saved to {filename}")
        # Optional: Try opening map (might not work on servers)
        # try:
        #     filepath = os.path.realpath(filename)
        #     webbrowser.open('file://' + filepath)
        # except Exception as e_open: print(f"Could not automatically open map file: {e_open}")
    except Exception as e_save: print(f"Error saving map file: {e_save}")
    return filename

# --- Refactored Steps ---

# **MODIFIED** to accept center_lat, center_lon
def prepare_data(api_key, center_lat, center_lon, num_customers=25, num_vehicles=5, capacity=35,avg_demand=6):
    """Generates data centered on coords and calculates ORS distance matrix."""
    print("--- Preparing Data ---")
    # 1. Generate Synthetic Locations & Demands using center coords
    generated_data = generate_synthetic_data( # Call the imported function
        center_lat=center_lat, # Pass coords down
        center_lon=center_lon, # Pass coords down
        num_customers=num_customers,
        num_vehicles=num_vehicles,
        capacity=capacity,
        avg_demand=avg_demand
    )
    if not generated_data:
        print("Failed to generate synthetic data.")
        return None
    data = generated_data # Use the generated dictionary directly

    # 2. Check API Key (ensure it's provided)
    if not api_key:
         print("Error: ORS API key was not provided.")
         return None

    # 3. Calculate Distance Matrix via ORS
    print("Calculating ORS distance matrix...")
    distance_matrix = compute_ors_distance_matrix(data['locations'], api_key)
    if distance_matrix is None:
        print("Failed to get distance matrix from ORS.")
        return None
    data['distance_matrix'] = distance_matrix
    print("Distance matrix calculated.")
    print("--- Data Preparation Complete ---")
    return data

def solve_routing_problem(data):
    """Sets up and solves the CVRP using OR-Tools."""
    if not data: return None, None, None
    print("--- Solving Routing Problem ---")
    # 1. Setup Routing Model
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    # 2. Define Callbacks & Costs (No changes needed here)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        if 0 <= from_node < len(data['distance_matrix']) and 0 <= to_node < len(data['distance_matrix'][0]): # Check column bound too
             return data['distance_matrix'][from_node][to_node]
        else:
             print(f"Warning: Invalid node index in distance_callback ({from_node}, {to_node})")
             return 9999999
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        if 0 <= from_node < len(data['demands']): return data['demands'][from_node]
        else:
             print(f"Warning: Invalid node index in demand_callback ({from_node})")
             return 999
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    # 3. Add Constraints (No changes needed here)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index, 0, data['vehicle_capacities'], True, 'Capacity')

    # 4. Set Search Parameters (No changes needed here)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # search_parameters.time_limit.SetSeconds(30)

    # 5. Solve
    print("Running OR-Tools solver...")
    solution = routing.SolveWithParameters(search_parameters)
    print("--- Solver Finished ---")

    if solution:
        return solution, manager, routing
    else:
        return None, None, None

def extract_solution_details(data, manager, routing, solution):
    """Extracts key details from the OR-Tools solution object."""
    # (No changes needed in this function's logic)
    if not solution: return None
    print("--- Extracting Solution Details ---")
    output = {
        'status': 'success', 'objective_distance_meters': solution.ObjectiveValue(),
        'total_load_delivered': 0, 'routes': [], 'route_details': []
    }
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_coords = []
        route_nodes = []
        route_load = 0
        route_distance = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_nodes.append(node_index)
            route_coords.append(data['locations'][node_index])
            route_load += data['demands'][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            if previous_index is not None and index is not None:
                 arc_cost = routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
                 if arc_cost is not None: route_distance += arc_cost
        node_index = manager.IndexToNode(index)
        route_nodes.append(node_index)
        route_coords.append(data['locations'][node_index])
        if route_distance > 0:
            output['routes'].append(route_coords)
            output['route_details'].append({
                'vehicle_id': vehicle_id, 'nodes_visited': route_nodes,
                'distance_meters': route_distance, 'load': route_load
            })
            total_load += route_load
    output['total_load_delivered'] = total_load
    print("--- Solution Details Extracted ---")
    return output

# --- Main Orchestrator Function ---

# **MODIFIED** to accept center_lat, center_lon
def run_vellore_solver(api_key=None, center_lat=None, center_lon=None, num_customers=25, num_vehicles=5, capacity=35,avg_demand=6, visualize=True):
    """Runs the full CVRP process for a given center location."""
    print("--- Starting CVRP Solver ---")
    load_dotenv() # Load .env file

    # --- Get API Key ---
    # Use passed key if available, otherwise get from environment
    if api_key is None:
        api_key = os.environ.get('ORS_API_KEY')
        if not api_key:
             print("ERROR: ORS API Key not provided and not found in environment variable ORS_API_KEY.")
             return {'status': 'error', 'message': 'API Key missing'}

    # --- Check for coordinates ---
    if center_lat is None or center_lon is None:
        print("ERROR: Center coordinates not provided to solver.")
        return {'status': 'error', 'message': 'Center location coordinates missing'}

    # --- 1. Prepare Data ---
    # Pass coordinates down to prepare_data
    data = prepare_data(
        api_key=api_key,
        center_lat=center_lat,
        center_lon=center_lon,
        num_customers=num_customers,
        num_vehicles=num_vehicles,
        capacity=capacity,
        avg_demand=avg_demand 
    )
    if data is None:
        # Error message already printed in prepare_data
        return {'status': 'error', 'message': 'Data preparation failed'}

    # --- 2. Solve ---
    solution, manager, routing = solve_routing_problem(data)
    if solution is None:
         return {'status': 'error', 'message': 'Solver failed to find a solution'}

    # --- 3. Extract Results ---
    results = extract_solution_details(data, manager, routing, solution)
    if results is None:
         return {'status': 'error', 'message': 'Failed to extract solution details'}

    # --- 4. Add necessary info for display ---
    results['locations'] = data['locations'] # Needed for markers
    results['demands'] = data['demands'] # Needed for popups
    results['center_coords'] = [center_lat, center_lon] # Needed for map centering in template

    # --- 5. Visualize (Optional) ---
    map_file = None
    if visualize:
        # Pass a unique filename if needed, maybe based on timestamp or params
        map_file = visualize_solution_map(data, manager, routing, solution)
    results['map_html_file'] = map_file

    print("--- CVRP Solver Finished Successfully ---")
    return results

# --- Test Execution Block ---
if __name__ == '__main__':
    import pprint
    # load_dotenv() is called inside run_vellore_solver now

    # --- Configuration for Direct Run ---
    test_api_key = os.environ.get('ORS_API_KEY') # Read key here for check/pass
    test_lat = 12.9165 # Example for Vellore
    test_lon = 79.1325 # Example for Vellore
    num_cust = 15
    num_veh = 4
    veh_capacity = 40
    test_avg_demand = 8
    # --- End Configuration ---

    if not test_api_key:
        print("API Key not found in environment variable ORS_API_KEY for testing.")
    else:
        print(f"--- Running Test with Center: ({test_lat}, {test_lon}) ---")
        # Call the main orchestrator function, passing coordinates
        solution_results = run_vellore_solver(
            api_key=test_api_key, # Pass key explicitly here
            center_lat=test_lat,
            center_lon=test_lon,
            num_customers=num_cust,
            num_vehicles=num_veh,
            capacity=veh_capacity,
            avg_demand=test_avg_demand,
            visualize=True # Set to True to generate map when testing directly
        )

        print("\n--- Solver Result Summary (Direct Run) ---")
        if solution_results and solution_results['status'] == 'success':
            pprint.pprint(solution_results)
            print(f"\nFind map in: {solution_results.get('map_html_file', 'Not generated')}")
        else:
            print("Solver did not complete successfully.")
            pprint.pprint(solution_results)
        print("--- End of Direct Execution ---")
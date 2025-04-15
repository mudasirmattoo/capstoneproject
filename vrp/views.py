from django.shortcuts import render
from django.http import HttpRequest
from geopy.geocoders import Nominatim
from geopy.exc import GeocoderTimedOut, GeocoderServiceError
from .solve_cvrp import run_vellore_solver
import pprint

def optimize_routes_view(request: HttpRequest):
    context = {
        'result': None,
        'error': None,
        'submitted_location': request.POST.get('location_name', '') 
    }
    center_lat, center_lon = None, None 

    if request.method == 'POST':
        print("POST request received") 

        lat_str = request.POST.get('latitude')
        lon_str = request.POST.get('longitude')
        location_name = request.POST.get('location_name', '')
        try:
            num_customers = int(request.POST.get('num_customers', 25))
            num_vehicles = int(request.POST.get('num_vehicles', 5))
            capacity = int(request.POST.get('capacity', 35))
            avg_demand = int(request.POST.get('avg_demand', 6))
        except (ValueError, TypeError):
            context['error'] = "Invalid numeric input for customers, vehicles, or capacity."
            return render(request, 'vrp/optimizer.html', context)

        print(f"Raw Input: Loc='{location_name}', Lat='{lat_str}', Lon='{lon_str}', Cust={num_customers}, Veh={num_vehicles}, Cap={capacity}") # Server log

        if lat_str and lon_str:
            try:
                center_lat = float(lat_str)
                center_lon = float(lon_str)
                print(f"Using coordinates from form: ({center_lat:.4f}, {center_lon:.4f})")
                if location_name == '(Current Location)' or not location_name:
                     context['submitted_location'] = f"(Current Location ~{center_lat:.4f}, {center_lon:.4f})"
            except ValueError:
                context['error'] = "Invalid coordinate values received from form."
                print("Error converting submitted coordinates to float.")
                center_lat, center_lon = None, None # Reset on error

        elif location_name:
            print(f"No valid coordinates in form, geocoding: '{location_name}'")
            try:
                geolocator = Nominatim(user_agent="my_vrp_capstone_app_v1_contact@example.com")
                location = geolocator.geocode(location_name, timeout=10)
                if location:
                    center_lat, center_lon = location.latitude, location.longitude
                    print(f"Geocoded '{location_name}' to: ({center_lat:.4f}, {center_lon:.4f})")
                else:
                    context['error'] = f"Could not find coordinates for location: '{location_name}'. Please check the name or try broader terms."
                    print(f"Geocoding failed for: {location_name}")
            except (GeocoderTimedOut, GeocoderServiceError) as e:
                context['error'] = f"Geocoding service error: {e}. Please try again later."
                print(f"Geocoding error: {e}")
            except Exception as e:
                context['error'] = f"An unexpected error occurred during geocoding: {e}"
                print(f"Unexpected geocoding error: {e}")

        else:
            context['error'] = "Please enter a location name or use 'Use Current Location'."
            print("No location information provided.")


        if center_lat is not None and center_lon is not None and context['error'] is None:
            solution_results = run_vellore_solver(
                api_key=None,
                center_lat=center_lat,
                center_lon=center_lon,
                num_customers=num_customers,
                num_vehicles=num_vehicles,
                capacity=capacity,
                avg_demand=avg_demand,
                visualize=False 
            )

            if solution_results and solution_results.get('status') == 'success':
                print("Solver successful.")
                try:
                    distance_m = solution_results.get('objective_distance_meters', 0)
                    solution_results['distance_km'] = distance_m / 1000.0
                except (TypeError, ValueError):
                    solution_results['distance_km'] = 0 

                solution_results['center_coords'] = [center_lat, center_lon]
                context['result'] = solution_results

            elif context['error'] is None: 
                context['error'] = solution_results.get('message', 'VRP Solver failed.')
                print(f"Solver failed: {context['error']}")

    return render(request, 'vrp/optimizer.html', context)

def home_view(request: HttpRequest):
    context = {}
    return render(request, 'vrp/home.html', context)
from .vehicle_generator import VehicleGenerator
from .geometry.quadratic_curve import QuadraticCurve
from .geometry.cubic_curve import CubicCurve
from .geometry.segment import Segment
from .vehicle import Vehicle


class Simulation:
    def __init__(self):
        self.segments = []
        self.vehicles = {}
        self.traffic_lights = []
        self.vehicle_generator = []

        self.t = 0.0
        self.frame_count = 0
        self.dt = 1 / 60

    def add_vehicle(self, veh):
        self.vehicles[veh.id] = veh
        if len(veh.path) > 0:
            self.segments[veh.path[0]].add_vehicle(veh)

    def add_traffic_light(self, traffic_light):
        self.traffic_lights.append(traffic_light)

    def add_segment(self, seg):
        self.segments.append(seg)

    def create_traffic_light(self, duration):
        from src.trafficSimulator.core.traffic_light import TrafficLight
        tl = TrafficLight(duration)
        self.traffic_lights.append(tl)

    def add_vehicle_generator(self, gen):
        self.vehicle_generator.append(gen)

    def create_vehicle(self, **kwargs):
        veh = Vehicle(kwargs)
        self.add_vehicle(veh)

    def create_segment(self, *args):
        seg = Segment(args)
        self.add_segment(seg)

    def create_quadratic_bezier_curve(self, start, control, end):
        cur = QuadraticCurve(start, control, end)
        self.add_segment(cur)

    def create_cubic_bezier_curve(self, start, control_1, control_2, end):
        cur = CubicCurve(start, control_1, control_2, end)
        self.add_segment(cur)

    def create_vehicle_generator(self, **kwargs):
        gen = VehicleGenerator(kwargs)
        self.add_vehicle_generator(gen)

    def run(self, steps):
        for _ in range(steps):
            self.update()

    def update(self):
        # Update lead vehicles for all vehicles
        for segment in self.segments:
            for i in range(1, len(segment.vehicles)):
                current_vehicle = self.vehicles[segment.vehicles[i]]
                lead_vehicle = self.vehicles[segment.vehicles[i - 1]]

                if lead_vehicle.x - current_vehicle.x < 5:
                    current_vehicle.lead_vehicle = lead_vehicle
                else:
                    current_vehicle.lead_vehicle = None

                # Check if the vehicle should stop for a traffic light
                if current_vehicle.should_stop_for_traffic_light(self.traffic_lights):
                    current_vehicle.v = 0
                    current_vehicle.a = 0

        # Update traffic lights
        for tl in self.traffic_lights:
            tl.update(self.dt)

        # Update vehicles
        for segment in self.segments:
            if len(segment.vehicles) != 0:
                self.vehicles[segment.vehicles[0]].update(None, self.dt, self.traffic_lights)
            for i in range(1, len(segment.vehicles)):
               self.vehicles[segment.vehicles[i]].update(self.vehicles[segment.vehicles[i - 1]], self.dt, self.traffic_lights)

        # Check roads for out of bounds vehicle
        for segment in self.segments:
            # If road has no vehicles, continue
            if len(segment.vehicles) == 0: continue
            # If not
            vehicle_id = segment.vehicles[0]
            vehicle = self.vehicles[vehicle_id]
            # If first vehicle is out of road bounds
            if vehicle.x >= segment.get_length():
                # If vehicle has a next road
                if vehicle.current_road_index + 1 < len(vehicle.path):
                    # Update current road to next road
                    vehicle.current_road_index += 1
                    # Add it to the next road
                    next_road_index = vehicle.path[vehicle.current_road_index]
                    self.segments[next_road_index].vehicles.append(vehicle_id)
                # Reset vehicle properties
                vehicle.x = 0
                # In all cases, remove it from its road
                segment.vehicles.popleft()

        # Update vehicle generators
        for gen in self.vehicle_generator:
            gen.update(self)
        # Increment time
        self.t += self.dt
        self.frame_count += 1



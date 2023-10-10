import uuid
import numpy as np

class Vehicle:
    def __init__(self, config={}):
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()
        self.lead_vehicle = None
        
    def set_default_config(self):    
        self.id = uuid.uuid4()

        self.l = 4
        self.s0 = 4
        self.T = 1
        self.v_max = 16.6
        self.a_max = 1.44
        self.b_max = 4.61

        self.path = []
        self.current_road_index = 0

        self.x = 0
        self.y = 0
        self.v = 0
        self.a = 0
        self.stopped = False

    def init_properties(self):
        self.sqrt_ab = 2*np.sqrt(self.a_max*self.b_max)
        self._v_max = self.v_max

    def update_position(self):
            self.y += self.v

    def should_stop_for_traffic_light(self, traffic_lights):
        for tl in traffic_lights:
            if self.x < tl.position[0] + 10 and self.x > tl.position[0] - 10 and \
                    abs(self.y - tl.position[1]) < 2 and \
                    tl.state == 'RED':
                return True
        return False

    def update(self, lead, dt, traffic_lights):
        # Check for traffic lights
        if self.should_stop_for_traffic_light(traffic_lights):
            self.v = 0
            return

        if self.lead_vehicle and self.x - self.lead_vehicle.x < 5:
            self.v = 0
        else:
            # Update position and velocity
            if self.v + self.a * dt < 0:
                self.x -= 1 / 2 * self.v * self.v / self.a
                self.v = 0
            else:
                self.v += self.a * dt
                self.y += self.v * dt + self.a * dt * dt / 2

            # Update acceleration
            alpha = 0
            if lead:
                delta_x = lead.x - self.x - lead.l
                delta_v = self.v - lead.v

                alpha = (self.s0 + max(0, self.T * self.v + delta_v * self.v / self.sqrt_ab)) / delta_x

            self.a = self.a_max * (1 - (self.v / self.v_max) ** 4 - alpha ** 2)

            if self.stopped:
                self.a = -self.b_max * self.v / self.v_max


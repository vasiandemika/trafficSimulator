import uuid
import numpy as np

from src.trafficSimulator.core import traffic_light


class Vehicle:
    def __init__(self, config={}):
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()

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
        self.v = 0
        self.a = 0
        self.stopped = False

    def init_properties(self):
        self.sqrt_ab = 2 * np.sqrt(self.a_max * self.b_max)
        self._v_max = self.v_max

    def update(self, lead, dt, traffic_light=None):

        if traffic_light and traffic_light.is_red() and self.x + self.s0 >= traffic_light.position:
            self.stopped = True
        else:
            self.stopped = False
        # If the lead vehicle is not moving and is in front within a close distance
        if lead and lead.v == 0 and (lead.x - self.x) < self.s0:
            self.stopped = True

        # Update position and velocity
        if self.stopped:
            self.a = 0
            self.v = 0
        else:
            if self.v + self.a * dt < 0:
                self.x -= 1 / 2 * self.v * self.v / self.a
                self.v = 0
            else:
                self.v += self.a * dt
                self.x += self.v * dt + self.a * dt * dt / 2

        # Update acceleration based on IDM model (or any other logic)
        if not self.stopped:
            alpha = 0
            if lead:
                delta_x = lead.x - self.x - lead.l
                delta_v = self.v - lead.v

                alpha = (self.s0 + max(0, self.T * self.v + delta_v * self.v / self.sqrt_ab)) / delta_x

            self.a = self.a_max * (1 - (self.v / self.v_max) ** 4 - alpha ** 2)

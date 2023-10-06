class TrafficLight:
    def __init__(self, position, cycle_time=10):
        self.position = position  # Position of the traffic light along the road
        self.cycle_time = cycle_time  # Time in seconds for each state (red or green)
        self.time_elapsed = 0  # Time passed since the last state change
        self.state = "red"  # Initial state
        self.segment = None

    def update(self, dt):
        """Update the state of the traffic light."""
        self.time_elapsed += dt
        if self.time_elapsed >= self.cycle_time:
            # Switch the state of the traffic light
            self.state = "green" if self.state == "red" else "red"
            self.time_elapsed = 0  # Reset the timer

    def is_red(self):
        return self.state == "red"


    def set_segment(self, segment):
        """Assign a segment to the traffic light."""
        self.segment = segment

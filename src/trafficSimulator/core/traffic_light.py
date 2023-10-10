class TrafficLight:
    def __init__(self, duration, position=(0, 0), state='RED'):
        self.position = position
        self.state = state
        self.timer = 0
        self.duration = duration
        self.detection_range = 5

    def update(self, dt):
        self.timer += dt
        if self.timer > self.duration:
            self.timer = 0
            if self.state == 'RED':
                self.state = 'GREEN'
            else:
                self.state = 'RED'

class TargetTrackerPath():
    def __init__(self, node, initial_xyz):
        
        self.qs = [initial_xyz]
        self.qs_dots = [[0,0,0]]
        self.N = 0
        self.tf = 0
        
        self.node = node

        self.cur_waypoint = 0
        self.waiting = False
        self.wait_time = 0.0 # seconds
        self.wait_end = 0
        self.allowed_pose_error = 0.2
        self.get_logger = node.get_logger

    def interpolate(self, t):
        offset=0
        self.dt = self.tf/self.N
        if t > self.tf:
            return self.qs[-1], [0,0,0]
        # Interpolate
        lower_index = int(t/self.dt)
        lower_index_time = int(t/self.dt)
        upper_index = lower_index_time + 1
        if lower_index_time >= (self.N-1):
            return self.qs[self.N-1], self.qs_dots[self.N-1]
        if upper_index >= (self.N-1):
            return self.qs[lower_index], self.qs_dots[lower_index]
        lower_time = lower_index_time*self.dt
        upper_time = (lower_index_time +1)*self.dt
        
        # look ahead to account for lagging controller, should be less for aggressive motions!!! TODO: adjust as needed
        # velocity look ahead
        lower_index = min(self.N-1, lower_index+int(0.2/self.dt))
        upper_index = min(self.N-1, upper_index+int(0.2/self.dt))
        q_dot = self.qs_dots[lower_index] + (self.qs_dots[upper_index] - self.qs_dots[lower_index])*(t - lower_time)/(upper_time - lower_time)

        # position look ahead
        lower_index = min(self.N-1, lower_index+int(0.4/self.dt))
        upper_index = min(self.N-1, upper_index+int(0.4/self.dt))

        q = self.qs[lower_index] + (self.qs[upper_index] - self.qs[lower_index])*(t - lower_time)/(upper_time - lower_time)
        return q, q_dot
    
    def get_next_target(self, current_position):
        """
        Returns the next waypoint to travel to.
        Moves to the next waypoint if within 20 cm of the current target.
        """
        if self.cur_waypoint >= len(self.qs):
            return self.qs[-1]  # Stay at the last waypoint

        target = self.qs[self.cur_waypoint]

        # Compute distance to the current waypoint
        distance = ((current_position[0] - target[0]) ** 2 +
                    (current_position[1] - target[1]) ** 2 +
                    (current_position[2] - target[2]) ** 2) ** 0.5

        # If within 20 cm, move to the next waypoint
        if distance < 0.2 and self.cur_waypoint < len(self.qs) - 1:
            self.cur_waypoint += 1
            target = self.qs[self.cur_waypoint]

        return target 
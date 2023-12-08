'''Module containing the Predator class'''
from math import sqrt
from boid import Boid

class Predator(Boid):
    '''The Predator class implements agents that hunt birds'''

    def __init__(self):
        super(Predator, self).__init__()
        self.nd = 0.25 # Hunting distance

    def update(self, others):
        ''' Updates velocity and updates position
        '''
        self.chase(others)
        self.move()

    def chase(self, others):
        ''' Calculates velocity for hunting others
        '''
        coh = self.cohesion(others)
        self.vx = self.vx + coh[0]
        self.vy = self.vy + coh[1]
        # Normalize to maxspeed
        v = sqrt(self.vx**2+self.vy**2)
        if v > self.maxspeed:
            self.vx = self.vx/v*self.maxspeed
            self.vy = self.vy/v*self.maxspeed

    def cohesion(self, others):
        ''' Returns acceleration towards center of mass of nearby others
        '''
        count = 0
        steerx = 0
        steery = 0
        xavg = 0
        yavg = 0
        for n in range(0, len(others)):
            dist = self.distance(others[n])
            if dist < self.nd and dist > 0.0:
                count = count + 1
                xavg = xavg + others[n].x
                yavg = yavg + others[n].y
        if count > 0:
            xavg = xavg / count
            yavg = yavg / count
            # Create ghost boid representing the others
            ghost = Boid()
            ghost.x = xavg
            ghost.y = yavg
            steerx, steery = self.distanceVector(ghost)
            length = sqrt(steerx**2+steery**2)
            steerx = steerx/length*self.maxforce
            steery = steery/length*self.maxforce
            steerx = steerx - self.vx
            steery = steery - self.vy
        return steerx, steery
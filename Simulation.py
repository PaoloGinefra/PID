import numpy as np
import math
import random
from numpy.lib.arraysetops import isin

from numpy.random.mtrand import rand



class Circle:
    def __init__(self, radius: float, initial_position: np.array, dP = 290, dR = 10) -> None:
        #Initialize Position and Radius
        self.position = initial_position
        self.radius = radius
        self.isColliding = False
        self.dP = dP
        self.dR = dR

        #Set BasePosition
        self.basePosition = initial_position
        self.baseRadius = radius

        #Initialize Previous Position and Velocity
        self.prevPosition = self.position
        self.velocity = np.zeros(2)
        pass

    def Stop(self):
        #Freez the Circle
        self.prevPosition = self.position
        self.velocity = np.zeros(2)
        pass

    def Randomize(self, center_X = None):
        #Randomizes the Circle Position's X component and the radius 
        if center_X == None:
            center_X = self.basePosition[0]

        self.position[0] = center_X + random.uniform(-self.dP, self.dP)
        self.position[1] = self.basePosition[1]
        self.radius = self.baseRadius + random.randint(-self.dR, self.dR)

class Stick:
    def __init__(self, position : np.array, length : float, thickness : float, initial_theta : float, maxTheta = 90, friction_coeff = 0):
        #Initialize parameters
        self.position = position
        self.length = length
        self.thickness = thickness
        self.maxTheta = maxTheta
        self.friction_coeff = friction_coeff

        #Set Base Theta
        self.baseTheta = initial_theta

        #Bring the stick to the initial condition
        self.SetAngle(initial_theta)

    def SetAngle(self, a):
        #Contrain the angle to 180 deg for semplicity's sake
        self.theta = a % 180

        #Apply MaxAngle constraint
        if self.maxTheta < self.theta <= 90:
            self.theta = self.maxTheta
        elif 90 < self.theta < 180 - self.maxTheta:
            self.theta = 180 - self.maxTheta

        #Compute tangent vector
        self.direction = Simulation.Versor(self.theta)
        self.direction *= np.sign(self.direction[0])

        #Compute Normal Vector
        self.normal = Simulation.Versor((self.theta + 90) % 180)

        #Compute the positions of the Right and Left Bound of the Stick
        self.Bound1 = np.add(self.position, (self.length / 2) * self.direction)
        self.Bound2 = np.add(self.position, -(self.length / 2) * self.direction)

        if self.Bound1[0] >= self.Bound2[0]:
            self.RightBound, self.LeftBound = self.Bound1[0], self.Bound2[0]
        else:
            self.RightBound, self.LeftBound = self.Bound2[0], self.Bound1[0]

        if self.Bound1[1] >= self.Bound2[1]:
            self.UpperBound, self.LowerBound = self.Bound1[1], self.Bound2[1]
        else:
            self.UpperBound, self.LowerBound = self.Bound2[1], self.Bound1[1]
        pass

class Simulation:
    g = 9.81
    DOWN = np.array([0, 1])

    def Versor(angle : float):
        #returns a Vector with magnitude = 1 and arg = angle
        angle = math.radians(angle)
        return np.array([np.cos(angle), np.sin(angle)])

    def __init__(self, stick : Stick, circle : Circle, dt : float, randomize = False) -> None:
        #Initialize parameters
        self.stick = stick
        self.circle = circle
        self.dt = dt
        self.t = 0
        self.randomize = randomize
        pass

    def Reset(self):
        #Reset Circle position
        if self.randomize:
            self.circle.Randomize(center_X = self.stick.position[0])
        else:
            self.circle.position = self.circle.basePosition

        #Freezes the Circle
        self.circle.Stop()

        #Reset Stick
        self.stick.theta = self.stick.baseTheta

        #Reset Time
        self.t = 0
        pass

    def ApplyVel(self):
        #Apply velocity to position as an Euler Integration
        self.circle.prevPosition = np.copy(self.circle.position)
        self.circle.position = np.add(self.circle.position, self.circle.velocity  * self.dt)

    def ApplyGravity(self):
        #Recompute Velocity and Aplly Gravity
        self.circle.velocity = (self.circle.position - self.circle.prevPosition)/self.dt
        self.circle.velocity = np.add(self.circle.velocity, Simulation.DOWN * Simulation.g * self.dt)

    def Collision(self):
        #Compute tollerable distance
        TollDist = self.stick.thickness / 2 + self.circle.radius

        #Compute whwther the circle is in the stick's bounding box
        InBaoundingBox = (self.stick.LeftBound <= self.circle.position[0] <= self.stick.RightBound)
        InBaoundingBox = InBaoundingBox and (self.stick.LowerBound - TollDist <= self.circle.position[1] <= self.stick.UpperBound + TollDist)
        
        #Handle Collition
        if InBaoundingBox:
            #Compute distance between Circle and stick
            dist = np.dot(np.add(self.stick.position, -self.circle.position), self.stick.normal)
            if (0 <= dist <= TollDist):
                self.circle.isColliding = True
                self.circle.position = np.add(self.circle.position, (dist - TollDist) * self.stick.normal)

                #Remove normal component of velocity

                tangent = np.dot(self.circle.velocity, self.stick.direction)
                s = np.sign(tangent)
                friction = np.dot(Simulation.DOWN, self.stick.normal) * self.stick.friction_coeff * s * self.dt

                if np.sign(tangent - friction) != s:
                    result = 0
                else:
                    result = tangent - friction
                self.circle.velocity = result * self.stick.direction

    def isInFrame(self):
        #Compute whether the Circle is over the Stick
        return self.circle.position[1] > self.stick.UpperBound or not (self.stick.LeftBound <= self.circle.position[0] <= self.stick.RightBound)

    def PacMan(self):
        if self.isInFrame():
            self.Reset()

    def Step(self):
        #A step of the simulation
        self.ApplyGravity()
        self.Collision()
        self.ApplyVel()

        self.t += self.dt
        pass

class Sensor:
    def __init__(self, simulation : Simulation, position_coeff : float, radius = 15) -> None:
        #initialize parameters
        self.sim = simulation
        self.posK = position_coeff
        self.circle = Circle(radius, np.zeros(2))
        self.updatePosition()
        pass

    def updatePosition(self):
        #Update Sensor's position
        DisplaceVector = self.sim.stick.direction * (self.posK * self.sim.stick.length - self.sim.stick.length / 2)
        self.circle.position = np.add(self.sim.stick.position, DisplaceVector)

    def Evaluate(self):
        #Evaluate the tangent distance between the Circle and the Sensor
        diff = np.add(self.sim.circle.position, - self.circle.position)
        dist = np.dot(diff, self.sim.stick.direction)
        return dist


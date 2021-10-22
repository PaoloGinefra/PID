import numpy as np
import math
import random

from numpy.random.mtrand import rand


class Circle:
    isColliding = False

    def __init__(self, radius: float, initial_position: np.array) -> None:
        self.basePosition = initial_position
        self.position = initial_position
        self.radius = radius
        self.baseRadius = radius
        self.prevPosition = self.position
        self.velocity = np.zeros(2)
        pass

    def Stop(self):
        self.prevPosition = self.position
        self.velocity = np.zeros(2)

class Stick:
    def __init__(self, position : np.array, length : float, thickness : float, initial_theta : float, maxTheta = 90) -> None:
        self.position = position
        self.length = length
        self.thickness = thickness
        self.baseTheta = initial_theta
        self.maxTheta = maxTheta
        self.SetAngle(initial_theta)

    def SetAngle(self, a):
        self.theta = a % 180

        if self.maxTheta < self.theta <= 90:
            self.theta = self.maxTheta
        elif 90 < self.theta < 180 - self.maxTheta:
            self.theta = 180 - self.maxTheta

        self.direction = Simulation.Versor(self.theta)
        self.direction *= np.sign(self.direction[0])
        self.normal = Simulation.Versor((self.theta + 90) % 180)

        self.RightBound = np.add(self.position, (self.length / 2) * self.direction)
        self.LeftBound = np.add(self.position, -(self.length / 2) * self.direction)

        if self.RightBound[0] < self.LeftBound[0]:
            self.RightBound, self.LeftBound = self.LeftBound, self.RightBound

        pass

class Simulation:
    g = 9.81

    DOWN = np.array([0, 1])

    def Versor(angle : float):
        return np.array([np.cos(math.radians(angle)), np.sin(math.radians(angle))])

    def __init__(self, stick : Stick, circle : Circle, dt : float) -> None:
        self.stick = stick
        self.circle = circle
        self.dt = dt
        self.t = 0
        pass

    def Reset(self):
        self.RandomizeCircle()
        self.circle.position = self.circle.basePosition
        self.circle.Stop()

        self.stick.theta = self.stick.baseTheta

        self.t = 0
        pass

    def RandomizeCircle(self, dp = 100, dr = 10):
        self.circle.basePosition[0] = self.stick.position[0] + random.uniform(-dp, dp)
        self.circle.radius = self.circle.baseRadius + random.randint(-dr, dr)

    def ApplyVel(self):
        self.circle.prevPosition = np.copy(self.circle.position)
        self.circle.position = np.add(self.circle.position, self.circle.velocity  * self.dt)

    def ApplyGravity(self):
        self.circle.velocity = (self.circle.position - self.circle.prevPosition)/self.dt
        self.circle.velocity = np.add(self.circle.velocity, Simulation.DOWN * Simulation.g * self.dt)

    def Collision(self):
        up = self.stick.thickness / 2 + self.circle.radius
        if (self.stick.LeftBound[0] <= self.circle.position[0] <= self.stick.RightBound[0]) and (min(self.stick.LeftBound[1], self.stick.RightBound[1]) - up <= self.circle.position[1] <= max(self.stick.LeftBound[1], self.stick.RightBound[1]) + up):
            dist = np.dot(np.add(self.stick.position, -self.circle.position), self.stick.normal)
            if (0 <= dist <= up):
                self.circle.isColliding = True
                self.circle.position = np.add(self.circle.position, (dist - up) * self.stick.normal)
                self.circle.velocity = np.dot(self.circle.velocity, self.stick.direction) * self.stick.direction

    def PacMan(self):
        if self.isInFrame():
            self.Reset()

    def isInFrame(self):
        return self.circle.position[1] > max(self.stick.RightBound[1], self.stick.LeftBound[1])

    def Step(self):
        self.ApplyGravity()
        self.Collision()
        self.ApplyVel()

        self.t += self.dt
        pass

class Sensor:
    def __init__(self, simulation : Simulation, position_coeff : float, radius = 15) -> None:
        self.sim = simulation
        self.posK = position_coeff
        self.circle = Circle(radius, np.zeros(2))
        self.updatePosition()
        pass

    def updatePosition(self):
        self.position = np.add(self.sim.stick.position, self.sim.stick.direction * (self.posK * self.sim.stick.length - self.sim.stick.length / 2))
        self.circle.position = self.position

    def Evaluate(self):
        self.updatePosition()
        diff = np.add(self.sim.circle.position, - self.position)
        dist = np.dot(diff, self.sim.stick.direction)
        return dist


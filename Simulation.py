import numpy as np
import math

from pygame.draw import circle

class Circle:
    def __init__(self, radius: float, initial_position: np.array) -> None:
        self.position = initial_position
        self.radius = radius
        self.prevPosition = self.position
        self.velocity = np.zeros(2)
        pass

    def Stop(self):
        self.prevPosition = self.position
        self.velocity = np.zeros(2)

class Stick:
    def __init__(self, position : np.array, length : float, thickness : float, initial_theta : float) -> None:
        self.position = position
        self.length = length
        self.thickness = thickness
        self.SetAngle(initial_theta)

    def SetAngle(self, a):
        self.theta = a % 180

        self.direction = Simulation.Versor(self.theta)
        self.normal = Simulation.Versor((self.theta + 90) % 180)

        self.RightBound = np.add(self.position, (self.length / 2) * self.direction)
        self.LeftBound = np.add(self.position, -(self.length / 2) * self.direction)

        if self.RightBound[0] < self.LeftBound[0]:
            self.RightBound, self.LeftBound = self.LeftBound, self.RightBound

        pass

class Simulation:
    g = 9.81
    dt = 0.001

    DOWN = np.array([0, 1])

    def Versor(angle : float):
        return np.array([np.cos(math.radians(angle)), np.sin(math.radians(angle))])

    def __init__(self, stick : Stick, circle : Circle) -> None:
        self.stick = stick
        self.circle = circle
        pass

    def ApplyVel(self):
        self.circle.prevPosition = np.copy(self.circle.position)
        self.circle.position = np.add(self.circle.position, self.circle.velocity  * Simulation.dt)

    def ApplyGravity(self):
        self.circle.velocity = (self.circle.position - self.circle.prevPosition)/Simulation.dt
        self.circle.velocity = np.add(self.circle.velocity, Simulation.DOWN * Simulation.g * Simulation.dt)

    def Collision(self):
        up = self.stick.thickness / 2 + self.circle.radius
        if (self.stick.LeftBound[0] <= self.circle.position[0] <= self.stick.RightBound[0]) and (min(self.stick.LeftBound[1], self.stick.RightBound[1]) - up <= self.circle.position[1] <= max(self.stick.LeftBound[1], self.stick.RightBound[1]) + up):
            dist = np.dot(np.add(self.stick.position, -self.circle.position), self.stick.normal)
            if (0 <= dist <= up):
                self.circle.position = np.add(self.circle.position, (dist - up) * self.stick.normal)
                self.circle.velocity = np.dot(self.circle.velocity, self.stick.direction) * self.stick.direction

    def PacMan(self, height):
        if self.circle.position[1] > height:
            self.circle.position = np.array([500, 100])
            self.circle.Stop()

    def Step(self):
        self.ApplyGravity()
        self.Collision()
        self.ApplyVel()

        pass

import pygame
import numpy as np
from pygame.version import ver
from pygame import gfxdraw
from Simulation import Simulation, Circle, Stick, Sensor

class Visual:
    BgColour = (0, 0, 0)
    CircleColour = (68, 135, 242)
    StickColour = (255, 255, 255)
    SensorColour = (50, 168, 82)

    Random = False
    Visualize = True

    def __init__(self, width, height):
        #initialize Parameters
        self.width = width
        self.height = height
        self.isActive = True

        #initialize pygame window
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pass

    def DrawCircle(self, c : Circle, color = CircleColour):
        gfxdraw.circle(self.screen, int(c.position[0]), int(c.position[1]), c.radius, color)

    def DrawStick(self, s : Stick):
        #Compute the vertex of the rectangle
        vert = []
        ThickDisplace = (s.thickness / 2) * s.normal
        vert.append(np.add(s.Bound1, ThickDisplace))
        vert.append(np.add(s.Bound2, ThickDisplace))
        vert.append(np.add(s.Bound2, -ThickDisplace))
        vert.append(np.add(s.Bound1, -ThickDisplace))

        #Draws the rectangle
        gfxdraw.aapolygon(self.screen, tuple(vert), Visual.StickColour)
        return 0

    def DrawSim(self, simulation: Simulation):
        #Rest the Screen
        self.screen.fill(Visual.BgColour)

        #Draw the Circle and the Stick
        self.DrawCircle(simulation.circle)
        self.DrawStick(simulation.stick)

    def DrawSensor(self, sensor : Sensor):
        #Draw the sensor
        self.DrawCircle(sensor.circle, Visual.SensorColour)

    def Draw(self):
        #Update the screen
        pygame.display.flip()
        
    def HandleInput(self):
        #Input handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.isActive = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    Visual.Random = not Visual.Random
                    print('Randomization: ' + 'ON' * Visual.Random + 'OFF' * (not Visual.Random))

                elif event.key == pygame.K_SPACE:
                    Visual.Visualize = not Visual.Visualize
                    print('Visualization: ' + 'ON' * Visual.Visualize + 'OFF' * (not Visual.Visualize))

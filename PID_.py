from random import randrange
from Graphics import Visual
from Simulation import Simulation, Circle, Stick, Sensor
import random
import numpy as np

class Controller():
    def output(self, error : float, integral : float, derivative : float):
        return 0

class PID_controller(Controller):
    def Random(scale = 0.01, id = 0):
        return PID_controller(P = random.random() * scale, D = random.random()*0, I = random.random()*0.01, id = id)

    def __init__(self, P : float, I : float, D : float, id : int) -> None:
        #Initialize parameters
        self.P, self.I, self.D = P, I, D
        self.id = id
        pass

    def output(self, error : float, integral : float, derivative : float):
        #Compute the output as a linear combination of error, error_integral and error_derivative
        return self.P * error + self.I * integral + self.D * derivative
    
    def __hash__(self) -> int:
        return hash((self.P, self.I, self.D))

    def __eq__(self, o: Controller) -> bool:
        return (self.P == o.P) and (self.D == o.D) and (self.I == o.I)

    def Print(self):
        #print the controller
        print(f'Controller_id:{self.id}, P:{self.P}, I:{self.I}, D:{self.D}')

class Simulator:
    def __init__(self, simulation:Simulation, sensor:Sensor, target_distance:float,  n_steps:int, visualizer:Visual, showEvery = 1, interpolationFactor = 1, randomize = False) -> None:
        #initialize Parameters
        self.simulation = simulation
        self.sensor = sensor
        self.target_distance = target_distance
        self.n_steps = n_steps
        self.visualizer = visualizer
        self.showEvery = showEvery
        self.interpolationFactor = interpolationFactor
        self.randomize = randomize
        pass

    def Lerp(a : float, b : float, factor : float):
        return factor * b + (1 - factor) * a

    def Simulate(self, contr : Controller):
        #Update Randomization
        self.simulation.randomize = self.randomize

        #Rest Simulation at the beginning
        self.simulation.Reset()

        #Initialize parameters
        error, error_previous, error_integtal, error_integral_abs, error_derivative = 0, 0, 0, 0, 0
        ang = self.simulation.stick.theta
        TollRad = self.simulation.circle.radius + self.simulation.stick.thickness / 2

        for frame_n in range(self.n_steps):
            error = self.sensor.Evaluate() - self.target_distance - TollRad
            error_integtal += error * self.simulation.dt
            error_integral_abs += np.abs(error)  * self.simulation.dt
            error_derivative = (error - error_previous) / self.simulation.dt

            #Compute Target Angle
            Target = -contr.output(error, error_integtal, error_derivative)

            #Interpolate to Target Angle
            ang = Simulator.Lerp(ang, Target, self.interpolationFactor)

            #Apply Newly computed angle
            self.simulation.stick.SetAngle(ang)

            #Compute a step of the simulation
            self.simulation.Step()
            self.sensor.updatePosition()

            #If visualization is On, visualize
            self.visualizer.HandleInput()
            if(self.showEvery > 0 and frame_n % self.showEvery == 0 and Visual.Visualize):
                #Draw Simulation
                self.visualizer.DrawSim(self.simulation)
                self.visualizer.DrawSensor(self.sensor)
                self.visualizer.Draw()
                
                if not self.visualizer.isActive:
                    return 0

            if(self.simulation.isInFrame()):
                return (error_integral_abs + 10000 * (self.n_steps - frame_n)) / self.n_steps

        return error_integral_abs / self.n_steps

        



if __name__ == '__main__':
    vis = Visual(800, 800)

    st = Stick(np.array([400, 600]), 600, 20, -179, 30, friction_coeff=0.01)
    cir = Circle(25, np.array([500, 100]))

    sim = Simulation(st, cir, dt = 0.001)

    sens = Sensor(sim, position_coeff = 0, radius = 10)

    pid = PID_controller(P = 10, I = 10, D = 10, id = 0)

    simulator = Simulator(sim, sens, target_distance = 0, n_steps = 100000, visualizer = vis, showEvery = 10, interpolationFactor = 1)

    for _ in range(10):
        print(simulator.Simulate(pid))
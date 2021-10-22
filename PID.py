from math import sin
from Graphics import Visual
from Simulation import Simulation, Circle, Stick, Sensor
import numpy as np

class Controller():
    def u(self, error : float, integral : float, derivative : float):
        return 0

class PID_controller(Controller):
    def __init__(self, P : float, I : float, D : float) -> None:
        self.P, self.I, self.D = P, I, D
        pass

    def u(self, error : float, integral : float, derivative : float):
        return self.P * error + self.I * integral + self.D * derivative

class Simulator:
    def __init__(self, simulation : Simulation, sensor : Sensor) -> None:
        self.simulation = simulation
        self.sensor = sensor
        pass

    def Lerp(a : float, b : float, factor : float):
        return factor * a + (1 - factor) * b

    def Simulate(self, controller : Controller, target_distance :float,  n_steps : int, visualizer : Visual, showEvery = 1, interpolationFactor = 1, randomize = False):
        self.simulation.randomize = randomize
        self.simulation.Reset()

        error, error_previous, error_integtal, error_integral_abs, error_derivative = 0, 0, 0, 0, 0
        u = self.simulation.stick.theta

        for frame_n in range(n_steps):
            error = self.sensor.Evaluate() - target_distance
            error_integtal += error * self.simulation.dt
            error_integral_abs += np.abs(error * self.simulation.dt)
            error_derivative = (error - error_previous) / self.simulation.dt

            u = Simulator.Lerp(-controller.u(error, error_integtal, error_derivative), u, interpolationFactor)
            self.simulation.stick.SetAngle(u)

            self.simulation.Step()

            if(visualizer != None and frame_n % showEvery == 0):
                visualizer.DrawSim(self.simulation)
                visualizer.DrawSensor(self.sensor)
                visualizer.Draw()
                visualizer.HandleInput()
                if not visualizer.isActive: return

            if(self.simulation.isInFrame()):
                return error_integral_abs

        return error_integral_abs

        




vis = Visual(800, 800)

st = Stick(np.array([400, 600]), 600, 20, -179, 30)
cir = Circle(25, np.array([500, 100]))

sim = Simulation(st, cir, dt = 0.001)

sens = Sensor(sim, position_coeff = 0, radius = 10)

pid = PID_controller(P = 0.5, I = 0.0001, D = 0)

simulator = Simulator(sim, sens)

for _ in range(10):
    print(simulator.Simulate(pid, target_distance = 0, n_steps = 100000, visualizer = vis, showEvery = 10, interpolationFactor = 0.01))
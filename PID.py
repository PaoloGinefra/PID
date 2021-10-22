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

    def Simulate(self, controller : Controller, n_steps : int, visualizer : Visual, showEvery = 1):
        self.simulation.Reset()

        error, error_previous, error_integtal, error_integral_abs, error_derivative = 0, 0, 0, 0, 0

        for frame_n in range(n_steps):
            error = self.sensor.Evaluate()
            error_integtal += error * self.simulation.dt
            error_integral_abs += np.abs(error * self.simulation.dt)
            error_derivative = (error - error_previous) / self.simulation.dt

            u = -controller.u(error, error_integtal, error_derivative)
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

sens = Sensor(sim, 0.5, radius = 10)

pid = PID_controller(0.01, 0.05, 0)

simulator = Simulator(sim, sens)

for _ in range(10):
    print(simulator.Simulate(pid, 100000, vis, 10))
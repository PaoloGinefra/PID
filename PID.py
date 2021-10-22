from random import randrange
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
    
    def __hash__(self) -> int:
        return hash((self.P, self.I, self.D))

    def __eq__(self, o: Controller) -> bool:
        return (self.P == o.P) and (self.D == o.D) and (self.I == o.I)

    def Print(self):
        print(f'P:{self.P}, I:{self.I}, D:{self.D}')

class Simulator:
    def __init__(self, simulation : Simulation, sensor : Sensor, target_distance :float,  n_steps : int, visualizer : Visual, showEvery = 1, interpolationFactor = 1, randomize = False) -> None:
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
        return factor * a + (1 - factor) * b

    def Simulate(self, contr : Controller):
        self.simulation.randomize = self.randomize
        self.simulation.Reset()

        error, error_previous, error_integtal, error_integral_abs, error_derivative = 0, 0, 0, 0, 0
        ang = self.simulation.stick.theta

        for frame_n in range(self.n_steps):
            error = self.sensor.Evaluate() - self.target_distance
            error_integtal += error * self.simulation.dt
            error_integral_abs += np.abs(error * self.simulation.dt)
            error_derivative = (error - error_previous) / self.simulation.dt

            ang = Simulator.Lerp(-contr.u(error, error_integtal, error_derivative), ang, self.interpolationFactor)
            self.simulation.stick.SetAngle(ang)

            self.simulation.Step()

            if(self.visualizer != None and frame_n % self.showEvery == 0):
                self.visualizer.DrawSim(self.simulation)
                self.visualizer.DrawSensor(self.sensor)
                self.visualizer.Draw()
                self.visualizer.HandleInput()
                if not self.visualizer.isActive: return

            if(self.simulation.isInFrame()):
                return error_integral_abs + 800 * (self.n_steps - frame_n)

        return error_integral_abs

        



if __name__ == '__main__':
    vis = Visual(800, 800)

    st = Stick(np.array([400, 600]), 600, 20, -179, 30)
    cir = Circle(25, np.array([500, 100]))

    sim = Simulation(st, cir, dt = 0.001)

    sens = Sensor(sim, position_coeff = 0, radius = 10)

    pid = PID_controller(P = 0.01704885471205992, I = 2.6393660544879552e-05, D = -0.0006596460648538618)

    simulator = Simulator(sim, sens, target_distance = 0, n_steps = 100000, visualizer = vis, showEvery = 10, interpolationFactor = 0.01)

    for _ in range(10):
        print(simulator.Simulate(pid))
import numpy as np
from simulator import Simulator, ActuatorMotor
from pathlib import Path


def controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    """Joint space PD controller.
    
    Args:
        q: Current joint positions [rad]
        dq: Current joint velocities [rad/s]
        t: Current simulation time [s]
        
    Returns:
        tau: Joint torques command [Nm]
    """
    # Control gains tuned for UR5e
    kp = np.array([2, 10])
    kd = 2 * np.sqrt(kp)
    
    # Target joint configuration
    qdes = np.array([0.0, 0.0])
    
    # PD control law
    tau = -(kp @ (qdes - q) - kd @ dq)
    return tau

def main():
    torque_control = ActuatorMotor(torque_range=[-1.0, 1.0])
    
    sim = Simulator(
        xml_path="robots/inverted_pendulum/scene.xml",
        actuator= torque_control,
        fps=30,
        heigth=600,
        width=1024,
        record_video=False
    )

    sim.set_controller(controller)
    sim.run(time_steps=20.0, q0=[0.0, 0.1])

    sim.plot_results()

if __name__ == "__main__":
    main() 
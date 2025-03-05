import numpy as np
from simulator import Simulator, ActuatorMotor
from pathlib import Path
import pinocchio as pin

# Load the robot model from scene XML
model = pin.buildModelFromMJCF("robots/inverted_pendulum/inverted_pendulum.xml")
pin_data = model.createData()


def controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    """Joint space PD controller.
    
    Args:
        q: Current joint positions [rad]
        dq: Current joint velocities [rad/s]
        t: Current simulation time [s]
        
    Returns:
        tau: Joint torques command [Nm]
    """
    # Control gains
    kp = np.array([4, 14])
    kd = 2 * np.sqrt(kp)

    # Target joint configuration
    q_des = np.array([0.0, 0.0])
    dq_des = np.array([0.0, 0.0])
    ddq_des = np.array([0.0, 0.0])

    pin.computeAllTerms(model, pin_data, q, dq)

    M = pin_data.M
    #print(M)
    nle = pin_data.nle
    #print(nle)

    u = -(M @ (kp @ (q_des - q) + kd @ (dq_des - dq) + ddq_des) + nle)
    print(q_des - q)
    
    # PD control law
    # u = -(kp @ (q_des - q) - kd @ dq)

    return u[1]

def main():
    torque_control = ActuatorMotor(torque_range=[-1.0, 1.0])
    
    sim = Simulator(
        xml_path="robots/inverted_pendulum/scene.xml",
        actuator= torque_control,
        fps=30,
        #heigth=600,
        #width=1024,
        record_video=True
    )

    sim.set_controller(controller)
    sim.run(time_steps=20.0, q0=[0.0, 0.3])

    sim.plot_results()

if __name__ == "__main__":
    main() 
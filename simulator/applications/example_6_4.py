
import numpy as np
from systems.lti_state_space import LTIStateSpace
from systems.system_state import SysState
from ode_integrators.forward_euler import ODEVectorFWDEuler
from plot.two_d_plotter import TwoDPlotter





if __name__ == '__main__':
    print("Sarting application: Vehicle Steering")

    time_steps = 1500
    dt = 0.01
    time = 0.0
    r = 1.0




    gamma = 0.5
    omega_c = [0.5, 1.0, 2.0]
    zeta_c = 0.7
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([gamma, 1.])
    C = np.array([1.0, 0.0])
    B.shape = (2,1)
    print(B.shape[1])



    tarray1 = []
    yarray1 = []

    tarray2 = []
    yarray2 = []

    tarray3 = []
    yarray3 = []

    kwargs = dict()

    for omega in range(len(omega_c)):

        state = SysState(2)
        state.add_state("y", 0)
        state.add_state("theta", 1)
        lti = LTIStateSpace(state, ODEVectorFWDEuler(step_size=dt))
        old_state = np.zeros(shape=state.shape)
        lti.set_old_state(idx=0, old_state=old_state)

        # set the matrices
        lti.set_A(A)
        lti.set_B(B)
        lti.set_C(C)

        omega_val = omega_c[omega]

        time = 0.0
        for step in range(time_steps):
            print("At time %s "%time)

            y = state.get_state_value_by_name("y")
            theta = state.get_state_value_by_name("theta")

            # compute the control
            u = (omega_val**2)*(r - y) - (2*zeta_c*omega_val - gamma*(omega_val**2))*theta
            kwargs['u'] = u
            lti.execute(**kwargs)

            time += dt

            if omega == 0:
                tarray1.append(time)
                yarray1.append(state.get_state_value_by_name("y"))
            elif omega == 1:
                tarray2.append(time)
                yarray2.append(state.get_state_value_by_name("y"))
            elif omega == 2:
                tarray3.append(time)
                yarray3.append(state.get_state_value_by_name("y"))


    plotter = TwoDPlotter(xlabel="time (sec)", ylabel="Lateral Position", title="")
    plotter.plot(tarray1, yarray1, label='omega = 0.5')
    plotter.plot(tarray2, yarray2, label='omega = 1.0')
    plotter.plot(tarray3, yarray3, label='omega = 2.0')
    plotter.show_plots()

    print("End simulation....")

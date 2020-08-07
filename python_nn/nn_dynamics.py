import numpy as np
import scipy.io
import matplotlib.pyplot as plt

def load_params(path):
    params = {}
    with np.load(path) as data:
       items = data.files
       # print(items)
       for item in items:
           # print(data[item].shape)
           params[item] = data[item]
    # print(params)
    return params


def enforce_constraints(state, control):
    # control_min = np.ones(control.shape)*np.nan
    # control_max = np.ones(control.shape)*np.inf
    # for ii in range(len(control)):
    #     if control[ii] < control_min[ii]:
    #         control[ii] = control_min[ii]
    #     elif control[ii] > control_max[ii]:
    #         control[ii] = control_max[ii]
    return control


def compute_kinematics(state):
    state_der = np.zeros((3,1))
    state_der[0] = np.cos(state[2]) * state[4] - np.sin(state[2]) * state[5]
    state_der[1] = np.sin(state[2]) * state[4] + np.cos(state[2]) * state[5]
    state_der[2] = -state[6]
    return state_der


def compute_dynamics(state, control, params):
    acts = np.zeros((6,1))
    acts[0:4,:] = state[3:,:]
    # print(acts)
    acts[4:,:] = control
    # print(acts.shape)
    h1 = np.dot(params['dynamics_W1'], acts) + params['dynamics_b1'].reshape((32,1))
    # print(h1.shape)
    n1 = np.tanh(h1)
    h2 = np.dot(params['dynamics_W2'], n1) + params['dynamics_b2'].reshape((32,1))
    n2 = np.tanh(h2)
    o = np.dot(params['dynamics_W3'], n2) + params['dynamics_b3'].reshape((4,1))
    return o


# state = [x y yaw roll vx vy yaw_rate]
def update_state(state, control, params):
    dt = 0.001
    control = enforce_constraints(state, control)
    state_der1 = compute_kinematics(state)
    state_der2 = compute_dynamics(state, control, params)
    # print(state_der2.shape)
    state_der = np.vstack((state_der1, state_der2))
    # print(state_der)
    state += state_der * dt
    return state


if __name__ == '__main__':
    path = "C:/Users/jknaup3/Downloads/autorally_nnet_09_12_2018.npz"
    params = load_params(path)
    mat = scipy.io.loadmat('full_vehicle_states_inputs.mat')
    full_veh_states = mat['states']
    _, N = full_veh_states.shape
    states = np.zeros((7,N))
    states[0,:] = full_veh_states[3,:]
    states[1,:] = full_veh_states[4,:]
    states[2,:] = full_veh_states[5,:]
    states[3,:] = full_veh_states[10,:]
    states[4,:] = full_veh_states[0,:]
    states[5,:] = full_veh_states[1,:]
    states[6,:] = full_veh_states[2,:]
    inputs = mat['inputs']
    inputs[0,:] *= -41
    print(inputs.shape)
    state = np.zeros((7,1))
    state = states[:,1].reshape((7,1))
    control = np.zeros((2,1))
    states_out = np.zeros((7,N))
    for ii in range(N):
        states_out[:,ii] = state.reshape((7,))
        state = update_state(state, inputs[:,ii].reshape((2,1)), params)
    states[3,:] *= -1
    states_out[6, :] *= -1
    print(state)
    time = range(N)
    plt.figure()
    n, N = states.shape
    for ii in range(n):
        plt.subplot(4,2,ii+1)
        plt.plot(time, states[ii,:])
        plt.plot(time, states_out[ii,:])
    plt.show()
    scipy.io.savemat('states_out.mat', {'nn_states': states_out})

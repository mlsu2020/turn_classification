import torch
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import pandas
from numpy import sin, cos, tan, arctan as atan, sqrt, arctan2 as atan2, zeros, zeros_like, abs, pi


class LinearNetwork:

    def __init__(self, idim, hdims, odim, learning_rate=1e-3, regularization=1e-1):
        self.cuda = torch.device('cpu')

        self.layers = []
        self.layers.append(torch.nn.Linear(idim, hdims[0]))
        for ii in range(1, len(hdims)):
            self.layers.append(torch.nn.Tanh())
            self.layers.append(torch.nn.Linear(hdims[ii-1], hdims[ii]))
        self.layers.append(torch.nn.Tanh())
        self.layers.append(torch.nn.Linear(hdims[-1], odim))

        # for layer in self.layers:
        #     if type(layer) == torch.nn.Linear:
        #         torch.nn.init.zeros_(layer.bias)
        #         torch.nn.init.ones_(layer.weight)

        self.model = torch.nn.Sequential(*self.layers)
        self.model.to(self.cuda)
        self.loss_fn = torch.nn.L1Loss(reduction='mean')
        self.optimizer = torch.optim.Adam(self.model.parameters(), weight_decay=regularization, lr=learning_rate)
        # self.optimizer = torch.optim.Adamax(self.model.parameters(), weight_decay=regularization, lr=learning_rate)

    def train(self, inputs, gt, num_iters=5000, ff=False):
        inputs = inputs.to(device=self.cuda)
        gt = gt.to(device=self.cuda)
        for ii in range(num_iters):
            # print(inputs)
            output = self.model(inputs)# + inputs[:, :8]
            if ff:
                output += inputs[:,ff:ff+1]
            # print(output)
            loss = self.loss_fn(output, gt)
            if ii % 100 == 1:
                print(ii, loss.item())
            self.model.zero_grad()
            loss.backward()
            self.optimizer.step()

    def validate(self, inputs):
        inputs = inputs.to(device=self.cuda)
        outputs = self.model(inputs)
        # outputs = outputs.cpu().detach().numpy()
        return outputs


def prepare_data():
    N0 = 200
    Nf = -200
    # data = pandas.read_csv('carsim_data/handling_course_hatchback_60kph.csv', skipinitialspace=True)
    # data.columns = data.keys().str.replace(" ", "")
    # ax = data['Ax'] * 9.81
    # alpha = (data['Alpha_L2'] + data['Alpha_R2']) / 2
    # alpha = alpha / 180 * pi
    # force = data['Fy_A2']
    mat = scipy.io.loadmat('mppi_data/mppi_ff_training1.mat')
    ax = mat['ff'][0, ::]
    alpha = mat['ff'][1, ::]
    force = mat['ff'][3, ::]
    measured_states1 = np.vstack((force)).reshape((1,-1))[:, N0:Nf]
    dx, N1 = measured_states1.shape
    controls1 = np.vstack((ax, alpha))[:, N0:Nf]
    du, N1 = controls1.shape
    # data = pandas.read_csv('carsim_data/handling_course_hatchback_120kph.csv', skipinitialspace=True)
    # data.columns = data.keys().str.replace(" ", "")
    # ax = data['Ax'] * 9.81
    # alpha = (data['Alpha_L2'] + data['Alpha_R2']) / 2
    # alpha = alpha / 180 * pi
    # force = data['Fy_A2']
    mat = scipy.io.loadmat('mppi_data/mppi_ff_training1.mat')
    ax = mat['ff'][0, ::]
    alpha = mat['ff'][1, ::]
    force = mat['ff'][3, ::]
    measured_states2 = np.vstack((force)).reshape((1,-1))[:, N0:Nf]
    dx, N2 = measured_states2.shape
    controls2 = np.vstack((ax, alpha))[:, N0:Nf]
    du, N2 = controls2.shape
    split = 0.9
    split = int(split * N1)
    training_inputs = np.zeros((du, 2*(split)))
    training_outputs = np.zeros((dx, 2 * (split)))
    # training_inputs[:dx, :] = predicted_states[:, :split]
    training_inputs[:, :split] = controls1[:du, :split]
    training_inputs[:, split:] = controls2[:du, :split]
    training_outputs[:, :split] = measured_states1[:, :split]
    training_outputs[:, split:] = measured_states2[:, :split]

    # data = pandas.read_csv('carsim_data/handling_course_hatchback_60kph.csv', skipinitialspace=True)
    # data.columns = data.keys().str.replace(" ", "")
    # ax = data['Ax'] * 9.81
    # alpha = (data['Alpha_L2'] + data['Alpha_R2']) / 2
    # alpha = alpha / 180 * pi
    # force = data['Fy_A2']
    mat = scipy.io.loadmat('mppi_data/mppi_ff_training1.mat')
    ax = mat['ff'][0, :]
    alpha = mat['ff'][1, :]
    force = mat['ff'][3, :]
    measured_states1 = np.vstack((force)).reshape((1,-1))[:, N0:Nf]
    dx, N1 = measured_states1.shape
    controls1 = np.vstack((ax, alpha))[:, N0:Nf]
    du, N1 = controls1.shape

    validation_inputs = np.zeros((du, N1))
    # validation_inputs[:dx, :] = predicted_states[:, split:]
    validation_inputs[:, :] = controls1[:du, :]
    validation_outputs = np.zeros((dx, N1))
    validation_outputs[:, :] = measured_states1[:, :]
    return training_inputs, training_outputs, validation_inputs, validation_outputs


def train_model():
    dims = (10,)
    dyn_model = LinearNetwork(2, dims, 1, learning_rate=1e-1, regularization=0)
    training_inputs, training_outputs, validation_inputs, validation_outputs = prepare_data()
    input_tensor = torch.from_numpy(training_inputs.T).float()
    output_tensor = torch.from_numpy(training_outputs.T).float()
    # print(training_inputs, input_tensor)
    dyn_model.train(input_tensor, output_tensor, num_iters=10000)
    torch.save(dyn_model.model.state_dict(), 'fric_net_ar1F.pth')

    dyn_model.model.load_state_dict(torch.load('fric_net_ar1F.pth'))
    forces = dyn_model.validate(input_tensor)
    # print(deltas)
    forces = forces.detach().numpy()
    plt.figure()
    N = len(forces)
    time = np.arange(N) / 100
    plt.plot(time, training_outputs[0, :], '.')
    plt.plot(time, forces[:], '.')
    plt.show()

    input_tensor = torch.from_numpy(validation_inputs.T).float()
    output_tensor = torch.from_numpy(validation_outputs.T).float()
    forces = dyn_model.validate(input_tensor)
    forces = forces.detach().numpy()
    plt.figure()
    N = len(forces)
    time = np.arange(N) / 100
    plt.plot(time, validation_outputs[0, :], '.')
    plt.plot(time, forces[:], '.')
    plt.show()
    # input_tensor = torch.from_numpy(validation_inputs.T).float()
    # forces = dyn_model.validate(input_tensor)
    # forces = forces.detach().numpy()
    # mat = {'forces': forces}
    # scipy.io.savemat('carsim_forces1.mat', mat)


def add_labels():
    plt.subplot(4, 2, 1)
    plt.gca().legend(('Measured vx', 'Pacejka vx', 'NN vx'))
    plt.xlabel('t (s)')
    plt.ylabel('m/s')
    plt.subplot(4, 2, 2)
    plt.gca().legend(('Measured vy', 'Pacejka vy', 'NN vy'))
    plt.xlabel('t (s)')
    plt.ylabel('m/s')
    plt.subplot(4, 2, 3)
    plt.gca().legend(('Measured wz', 'Pacejka wz', 'NN wz'))
    plt.xlabel('t (s)')
    plt.ylabel('rad/s')
    plt.subplot(4, 2, 4)
    plt.gca().legend(('Measured wF', 'Pacejka wF', 'NN wF'))
    plt.xlabel('t (s)')
    plt.ylabel('rad/s')
    plt.subplot(4, 2, 5)
    plt.gca().legend(('Measured wR', 'Pacejka wR', 'NN wF'))
    plt.xlabel('t (s)')
    plt.ylabel('rad/s')
    plt.subplot(4, 2, 6)
    plt.gca().legend(('Measured Yaw', 'Pacejka Yaw', 'NN Yaw'))
    plt.xlabel('t (s)')
    plt.ylabel('rad')
    plt.subplot(4, 2, 7)
    plt.gca().legend(('Measured X', 'Pacejka X', 'NN X'))
    plt.xlabel('t (s)')
    plt.ylabel('m')
    plt.subplot(4, 2, 8)
    plt.gca().legend(('Measured Y', 'Pacejka Y', 'NN Y'))
    plt.xlabel('t (s)')
    plt.ylabel('m')


def update_dynamics(state, input, nn=None, nnR=None):
    m_Vehicle_m = 21.7562#1270
    m_Vehicle_Iz = 1.124#2000
    m_Vehicle_lF = 0.34#1.015
    lFR = 0.57#3.02
    m_Vehicle_lR = lFR-m_Vehicle_lF
    m_Vehicle_IwF = 0.05#8
    m_Vehicle_IwR = .0373
    m_Vehicle_rF = 0.095#0.325
    m_Vehicle_rR = 0.09#0.325
    m_Vehicle_mu1 = 0.75
    m_Vehicle_mu2 = 0.90
    m_Vehicle_h = 0.12#.54
    m_g = 9.80665

    tire_B = 4#10
    tire_C = 2.0
    tire_D = 1.0
    tire_E = 1.0
    tire_Sh = 0.0
    tire_Sv = 0.0

    N, dx = state.shape
    m_nu = 1

    vx = state[:, 0]
    vy = state[:, 1]
    wz = state[:, 2]
    wF = state[:, 3]
    wR = state[:, 4]
    psi = state[:, 5]
    X = state[:, 6]
    Y = state[:, 7]

    m_Vehicle_kSteering = 18.7861
    m_Vehicle_cSteering = 0.0109
    # delta = input[:, 0]
    delta = m_Vehicle_kSteering * input[0, 0] + m_Vehicle_cSteering
    # T = m_Vehicle_kThrottle * input[:, 1]

    min_velo = 0.1
    deltaT = 0.01

    beta = atan2(vy, vx)

    V = sqrt(vx * vx + vy * vy)
    vFx = V * cos(beta - delta) + wz * m_Vehicle_lF * sin(delta)
    vFy = V * sin(beta - delta) + wz * m_Vehicle_lF * cos(delta)
    vRx = vx
    vRy = vy - wz * m_Vehicle_lR

    sEF = -(vFx - wF * m_Vehicle_rF) / (vFx) + tire_Sh
    muFx = tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv
    sEF = -(vRx - wR * m_Vehicle_rR) / (vRx) + tire_Sh
    muRx = tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv

    sEF = atan(vFy / abs(vFx)) + tire_Sh
    alpha = -sEF
    muFy = -tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv
    sEF = atan(vRy / abs(vRx)) + tire_Sh
    alphaR = -sEF
    muRy = -tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv

    fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * muRx) / (
            m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (muFx * cos(delta) - muFy * sin(delta) - muRx))
    # fFz = m_Vehicle_m * m_g * (m_Vehicle_lR / 0.57)
    fRz = m_Vehicle_m * m_g - fFz

    fFx = fFz * muFx
    fRx = fRz * muRx
    fFy = fFz * muFy
    fRy = fRz * muRy

    ax = ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz)

    dot_X =cos(psi)*vx - sin(psi)*vy
    dot_Y = sin(psi)*vx + cos(psi)*vy

    next_state = zeros_like(state)
    next_state[:, 0] = vx + deltaT * ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz)
    # next_state[:, 1] = vy + deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz)
    if nn and nnR:
        input_tensor = torch.from_numpy(np.vstack((ax, alpha)).T).float()
        fafy = nn(input_tensor).detach().numpy()
        input_tensor = torch.from_numpy(np.vstack((ax, alphaR)).T).float()
        fary = nnR(input_tensor).detach().numpy()

        next_state[:, 1] = vy + deltaT * ((fafy + fary) / m_Vehicle_m - vx * wz)
        next_state[:, 2] = wz + deltaT * ((fafy) * m_Vehicle_lF - fary * m_Vehicle_lR) / m_Vehicle_Iz
    else:
        next_state[:, 1] = vy + deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz)
        next_state[:, 2] = wz + deltaT * (
                    (fFy * cos(delta) + fFx * sin(delta)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz
    next_state[:, 3] = wF - deltaT * m_Vehicle_rF / m_Vehicle_IwF * fFx
    # next_state[:, 4] = wR + deltaT * (m_Vehicle_kTorque * (T-wR) - m_Vehicle_rR * fRx) / m_Vehicle_IwR
    next_state[:, 5] = psi + deltaT * wz
    next_state[:, 6] = X + deltaT * dot_X
    next_state[:, 7] = Y + deltaT * dot_Y

    # print(t1-t0, t2-t1, t3-t2, t4-t3, t5-t4)

    return next_state


def run_model():
    dyn_model = LinearNetwork(2, (10,), 1, learning_rate=1e-1, regularization=0)
    dyn_model.model.load_state_dict(torch.load('fric_net_ar1F.pth'))

    dyn_modelR = LinearNetwork(2, (10,), 1, learning_rate=1e-1, regularization=0)
    dyn_modelR.model.load_state_dict(torch.load('fric_net_ar1R.pth'))

    N0 = 2000
    Nf = -2000
    # # data = pandas.read_csv('carsim_data/highway_hatchback_120kph.csv', skipinitialspace=True)
    # # data.columns = data.keys().str.replace(" ", "")
    # vx = data['Vx']
    # vy = data['Vy']
    # wz = data['AVz']
    # wF = (data['AVy_L1'] + data['AVy_R1']) / 2
    # wR = (data['AVy_L2'] + data['AVy_R2']) / 2
    # yaw = data['Yaw']
    # X = data['Xcg_SM']
    # Y = data['Ycg_SM']
    # steering = (data['Steer_L1'] + data['Steer_R1']) / 2

    # vx = vx / 60 / 60 * 1000
    # vy = vy / 60 / 60 * 1000
    # wz = wz / 180 * pi
    # wF = wF / 60 * 2 * pi
    # wR = wR / 60 * 2 * pi
    # yaw = yaw / 180 * pi
    # steering = steering / 180 * pi
    #
    # states = np.vstack((vx, vy, wz, wF, wR, yaw, X, Y)).T
    # controls = np.asarray(steering).reshape((-1,1))

    mat = scipy.io.loadmat('mppi_data/mppi_states_controls1.mat')
    states = mat['states'][:, ::10].T
    controls = mat['inputs'][:, ::10].T

    states = states[N0:Nf, :]
    controls = controls[N0:Nf, :]
    time = np.arange(0, len(states)) * 0.01
    analytic_states = np.zeros_like(states)
    nn_states = np.zeros_like(states)
    state1 = states[0:1, :]
    state2 = states[0:1, :]
    for ii in range(len(time)):
        analytic_states[ii, :] = state1
        nn_states[ii, :] = state2
        state1 = update_dynamics(state1, controls[ii:ii+1, :])
        state2 = update_dynamics(state2, controls[ii:ii+1, :], dyn_model.validate, dyn_modelR.validate)
        state1[:, 3:5] = states[ii, 3:5]
        state2[:, 3:5] = states[ii, 3:5]
    plt.figure()
    for ii in range(states.shape[1]):
        plt.subplot(4,2,ii+1)
        plt.plot(time, states[:, ii])
        plt.plot(time, analytic_states[:, ii])
        plt.plot(time, nn_states[:, ii])
    add_labels()
    plt.show()


if __name__ == '__main__':
    # train_model()
    run_model()

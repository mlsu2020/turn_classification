import torch
from torch.nn import functional as F
import numpy as np
import scipy.io
import itertools
import matplotlib.pyplot as plt
import pandas
from numpy import sin, cos, tan, arctan as atan, sqrt, arctan2 as atan2, zeros, zeros_like, abs, pi


class Net(torch.nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        self.cuda = torch.device('cpu')

        self.si = torch.nn.Linear(1, 1)
        # self.so = torch.nn.Linear(10, 1)
        self.si.weight.requires_grad = False
        self.si.bias.requires_grad = False
        # self.so.weight.requires_grad = False
        # self.so.bias.requires_grad = False
        self.si.weight.data.fill_(18.7861)
        self.si.bias.data.fill_(0.0109)

        self.fi = torch.nn.Linear(2, 10)
        self.fo = torch.nn.Linear(10, 1)
        # self.fi.weight.requires_grad = False
        # self.fi.bias.requires_grad = False
        # self.fo.weight.requires_grad = False
        # self.fo.bias.requires_grad = False

        self.ri = torch.nn.Linear(2, 10)
        self.ro = torch.nn.Linear(10, 1)
        # self.ri.weight.requires_grad = False
        # self.ri.bias.requires_grad = False
        # self.ro.weight.requires_grad = False
        # self.ro.bias.requires_grad = False

    def forward(self, inputs):
        steering = inputs[:, 0:1]
        vx = inputs[:, 1:2]
        vy = inputs[:, 2:3]
        wz = inputs[:, 3:4]
        ax = inputs[:, 4:5]
        wF = inputs[:, 5:6]
        wR = inputs[:, 6:7]
        # delta = self.so(torch.tanh(self.si(steering)))
        delta = self.si(steering)
        # delta = 18.7862 * steering + 0.0109
        alphaF, alphaR, fFwx, fFwy, fRx, fRy = self.kinematics(delta, vx, vy, wz, wF, wR)
        delta_fFy = self.fo(torch.tanh(self.fi(torch.cat((ax, alphaF), dim=1))))
        delta_fRy = self.ro(torch.tanh(self.ri(torch.cat((ax, alphaR), dim=1))))
        fFy = fFwy * torch.cos(delta) + fFwx * torch.sin(delta)
        fFx = fFwx * torch.cos(delta) - fFwy * torch.sin(delta)
        output = torch.cat((fFy + delta_fFy, fRy + delta_fRy, fFx, fRx), dim=1)
        return output

    def kinematics(self, delta, vx, vy, wz, wF, wR):
        m_Vehicle_m = 21.7562
        lFR = 0.57
        m_Vehicle_lF = 0.34
        m_Vehicle_lR = lFR - m_Vehicle_lF
        m_Vehicle_rF = 0.095
        m_Vehicle_rR = 0.090
        m_Vehicle_h = 0.12
        m_g = 9.80665
        tire_B = 4
        tire_C = 1
        tire_D = 1.0

        beta = torch.atan2(vy, vx)
        V = torch.sqrt(vx * vx + vy * vy)
        vFx = V * torch.cos(beta - delta) + wz * m_Vehicle_lF * torch.sin(delta)
        vFy = V * torch.sin(beta - delta) + wz * m_Vehicle_lF * torch.cos(delta)
        vRx = vx
        vRy = vy - wz * m_Vehicle_lR

        sFx = (vFx - wF * m_Vehicle_rF) / (wF * m_Vehicle_rF)
        sRx = (vRx - wR * m_Vehicle_rR) / (wR * m_Vehicle_rR)
        sFy = vFy / (wF * m_Vehicle_rF)
        sRy = vRy / (wR * m_Vehicle_rR)
        sF = torch.sqrt(sFx ** 2 + sFy ** 2)
        sR = torch.sqrt(sRx ** 2 + sRy ** 2)
        muF = tire_D * torch.sin(tire_C * torch.atan(tire_B * sF))
        muR = tire_D * torch.sin(tire_C * torch.atan(tire_B * sR))
        muFx = -sFx / sF * muF
        muFy = -sFy / sF * muF
        muRx = -sRx / sR * muR
        muRy = -sRy / sR * muR
        fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * muRx) / (lFR + m_Vehicle_h * (muFx*torch.cos(delta) - muFy*torch.sin(delta) - muRx))
        fRz = m_Vehicle_m * m_g - fFz
        fFx = muFx * fFz
        fFy = muFy * fFz
        fRx = muRx * fRz
        fRy = muRy * fRz

        alphaF = -torch.atan(vFy / torch.abs(vFx))
        alphaR = -torch.atan(vRy / torch.abs(vRx))
        # vFx2 = vFx[:, 0]
        # vFx2[vFx2 < 0.2] = 0.2
        # vFx2 = vFx2.reshape((-1, 1))
        if((vFx == 0).any()):
            print(0)
        # slipF = -(vFx - m_Vehicle_rF*wF) / vFx
        return alphaF, alphaR, fFx, fFy, fRx, fRy


def prepare_data():
    N0 = 200
    Nf = -200
    mat = scipy.io.loadmat('mppi_data/mppi_ff_training1.mat')
    measured_states1 = mat['ff'][7:, N0:Nf]
    dx, N1 = measured_states1.shape
    controls1 = mat['ff'][:7, N0:Nf]
    du, N1 = controls1.shape

    mat = scipy.io.loadmat('mppi_data/mppi_ff_training1.mat')
    measured_states2 = mat['ff'][7:, N0:Nf]
    dx, N2 = measured_states2.shape
    controls2 = mat['ff'][:7, N0:Nf]
    du, N2 = controls2.shape
    split = 0.02
    split = int(split * N1)
    training_inputs = np.zeros((du, 2*(split)))
    training_outputs = np.zeros((dx, 2 * (split)))
    # training_inputs[:dx, :] = predicted_states[:, :split]
    training_inputs[:, :split] = controls1[:du, :split]
    training_inputs[:, split:] = controls2[:du, :split]
    training_outputs[:, :split] = measured_states1[:, :split]
    training_outputs[:, split:] = measured_states2[:, :split]

    mat = scipy.io.loadmat('mppi_data/mppi_ff_training1.mat')
    measured_states1 = mat['ff'][7:, N0:Nf]
    dx, N1 = measured_states1.shape
    controls1 = mat['ff'][:7, N0:Nf]
    du, N1 = controls1.shape

    validation_inputs = np.zeros((du, N1))
    # validation_inputs[:dx, :] = predicted_states[:, split:]
    validation_inputs[:, :] = controls1[:du, :]
    validation_outputs = np.zeros((dx, N1))
    validation_outputs[:, :] = measured_states1[:, :]
    return training_inputs, training_outputs, validation_inputs, validation_outputs


def train_model():
    dyn_model = Net()
    # dyn_model.load_state_dict(torch.load('hybrid_net_ar2.pth'))
    criterion = torch.nn.L1Loss()
    optimizer = torch.optim.Adamax(dyn_model.parameters(), lr=1e-2)
    training_inputs, training_outputs, validation_inputs, validation_outputs = prepare_data()
    input_tensor = torch.from_numpy(training_inputs.T).float()
    output_tensor = torch.from_numpy(training_outputs.T).float()
    for ii in range(5000):
        optimizer.zero_grad()
        output = dyn_model(input_tensor)
        wz_dot = (output[:,0] * 0.34 - output[:,1] * 0.23) / 1.124
        wz = torch.cumsum(wz_dot * 0.01, dim=0)
        vy_dot = ((output[:, 0] + output[:, 1]) / 21.7562 - input_tensor[:,1] * input_tensor[:,3])
        vy = torch.cumsum(vy_dot * 0.01, dim=0)
        output2 = torch.cat((output[:, 0:2], 100*wz.reshape(-1,1), 100*vy.reshape(-1,1)), dim=1)
        output_tensor2 = torch.cat((output_tensor[:,[0,1]], 100*input_tensor[:, 2:4]), dim=1)
        output2 = torch.cat((wz.reshape(-1, 1), vy.reshape(-1, 1)), dim=1)
        output_tensor2 = torch.cat((input_tensor[:, 3:4], input_tensor[:, 2:3]), dim=1)
        loss = criterion(output2, output_tensor2)
        loss.backward()
        optimizer.step()
        if ii % 100 == 0:
            print(ii, loss.item())
    torch.save(dyn_model.state_dict(), 'hybrid_net_ar2.pth')

    dyn_model.load_state_dict(torch.load('hybrid_net_ar2.pth'))
    forces = dyn_model(input_tensor)[:, :2]
    # print(deltas)
    forces = forces.detach().numpy()
    wz_dot = (forces[:, 0] * 0.34 - forces[:, 1] * 0.23) / 1.124
    wz = np.cumsum(wz_dot * 0.01)
    vy_dot = ((forces[:, 0] + forces[:, 1]) / 21.7562 - training_inputs[1,:] * wz)
    vy = np.cumsum(vy_dot * 0.01, axis=0)
    plt.figure()
    N = len(forces)
    time = np.arange(N) / 100
    plt.subplot(4,1,1)
    plt.plot(time, training_outputs[0, :], '.')
    plt.plot(time, forces[:, 0], '.')
    plt.subplot(4,1,2)
    plt.plot(time, training_outputs[1, :], '.')
    plt.plot(time, forces[:, 1], '.')
    plt.subplot(4, 1, 3)
    plt.plot(time, training_inputs[3, :], '.')
    plt.plot(time, wz[:], '.')
    plt.subplot(4, 1, 4)
    plt.plot(time, training_inputs[2, :], '.')
    plt.plot(time, vy[:], '.')
    plt.show()

    input_tensor = torch.from_numpy(validation_inputs.T).float()
    output_tensor = torch.from_numpy(validation_outputs.T).float()
    forces = dyn_model(input_tensor)[:, :2]
    forces = forces.detach().numpy()
    vy_dot = ((forces[:, 0] + forces[:, 1]) / 21.7562 - validation_inputs[1,:] * validation_inputs[3,:])
    wz_dot = (forces[:, 0] * 0.34 - forces[:, 1] * 0.23) / 1.124
    wz = np.cumsum(wz_dot * 0.01)
    plt.figure()
    N = len(forces)
    time = np.arange(N) / 100
    plt.subplot(3, 1, 1)
    plt.plot(time, validation_outputs[0, :], '.')
    plt.plot(time, forces[:, 0], '.')
    plt.subplot(3, 1, 2)
    plt.plot(time, validation_outputs[1, :], '.')
    plt.plot(time, forces[:, 1], '.')
    plt.subplot(3, 1, 3)
    plt.plot(time, validation_inputs[3, :], '.')
    plt.plot(time, wz[:], '.')
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


def update_dynamics(state, input, nn=None):
    m_Vehicle_m = 21.7562#1270
    m_Vehicle_Iz = 1.124#2000
    m_Vehicle_lF = 0.34#1.015
    lFR = 0.57#3.02
    m_Vehicle_lR = lFR-m_Vehicle_lF
    m_Vehicle_IwF = 0.1#8
    m_Vehicle_IwR = .0373
    m_Vehicle_rF = 0.095#0.325
    m_Vehicle_rR = 0.090#0.325
    m_Vehicle_h = 0.12#.54
    m_g = 9.80665

    tire_B = 4.0#10
    tire_C = 1.0
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
    steering = input[0, 0]
    delta = m_Vehicle_kSteering * steering + m_Vehicle_cSteering
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
    next_state[:, 1] = vy + deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz)
    vy_dot = ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz)
    if nn:
        input_tensor = torch.from_numpy(np.vstack((steering, vx, vy, wz, ax, wF, wR)).T).float()
        # input_tensor = torch.from_numpy(input).float()
        forces = nn(input_tensor).detach().numpy()
        fafy = forces[:, 0]
        fary = forces[:, 1]
        fafx= forces[0, 2]
        farx = forces[0, 3]

        next_state[:, 0] = vx + deltaT * ((fafx + farx) / m_Vehicle_m + vy * wz)
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
    dyn_model = Net()
    dyn_model.load_state_dict(torch.load('hybrid_net_ar2.pth'))

    N0 = 400
    Nf = -200
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

    # mat = scipy.io.loadmat('mppi_data/mppi_ff_training1.mat')
    # # measured_states1 = mat['ff'][6:, N0:Nf]
    # # dx, N1 = measured_states1.shape
    # controls1 = mat['ff'][:, N0:Nf].T
    # print(controls1.shape)
    # # du, N1 = controls1.shape
    # # forces = mat['ff'][6:, N0:Nf].T

    states = states[N0:Nf-1, :]
    controls = controls[N0:Nf-1, :]
    print(controls.shape)
    time = np.arange(0, len(states)) * 0.01
    analytic_states = np.zeros_like(states)
    nn_states = np.zeros_like(states)
    state1 = states[0:1, :]
    state2 = states[0:1, :]
    for ii in range(len(time)):
        analytic_states[ii, :] = state1
        nn_states[ii, :] = state2
        state1 = update_dynamics(state1, controls[ii:ii+1, :])
        state2 = update_dynamics(state2, controls[ii:ii+1, :], dyn_model)
        state1[:, 4:5] = states[ii, 4:5]
        state2[:, 4:5] = states[ii, 4:5]
        # state2[:, 0] = states[ii, 0]
        # state1[:, 0:2] = states[ii, 0:2]
        # state2[:, 0:2] = states[ii, 0:2]
    plt.figure()
    for ii in range(states.shape[1]):
        plt.subplot(4,2,ii+1)
        plt.plot(time, states[:, ii])
        plt.plot(time, analytic_states[:, ii])
        plt.plot(time, nn_states[:, ii])
    add_labels()
    plt.show()


if __name__ == '__main__':
    train_model()
    run_model()

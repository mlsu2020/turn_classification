import numpy as np
# from math import sin, cos
import scipy.io
import matplotlib.pyplot as plt
import torch
from numpy import sin, cos, arctan as atan, sqrt, arctan2 as arctan2, zeros, zeros_like
import time


6def update_dynamics(state, input):
    t0 = time.time()

    m_Vehicle_m = 22
    m_Vehicle_Iz = 1.9
    m_Vehicle_lF = .4
    m_Vehicle_lR = 0.57-m_Vehicle_lF
    m_Vehicle_IwF = .0416
    m_Vehicle_IwR = .0373
    m_Vehicle_rF = 0.095
    m_Vehicle_rR = 0.090
    m_Vehicle_mu1 = 0.75
    m_Vehicle_mu2 = 0.90
    m_Vehicle_h = .2
    m_g = 9.80665

    m_Vehicle_kSteering = 30.0811
    m_Vehicle_kThrottle = 204.0922
    m_Vehicle_kTorque = 0.1577

    tire_B = 1.5
    tire_C = 2.0
    tire_D = 1.0
    tire_E = 1.0
    tire_Sh = 0.0
    tire_Sv = 0.0

    N, dx = state.shape
    m_nu = 1

    t1 = time.time()

    vx = state[:, 0]
    vy = state[:, 1]
    wz = state[:, 2]
    wF = state[:, 3]
    wR = state[:, 4]
    psi = state[:, 5]
    X = state[:, 6]
    Y = state[:, 7]

    delta = input[:, 0]
    # T = m_Vehicle_kThrottle * input[:, 1]

    t2 = time.time()

    min_velo = 0.1
    deltaT = 0.01

    beta = arctan2(vy, vx)

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
    muFy = -tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv
    sEF = atan(vRy / abs(vRx)) + tire_Sh
    muRy = -tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv

    fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * muRx) / (
            m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (muFx * cos(delta) - muFy * sin(delta) - muRx))
    fFz = m_Vehicle_m * m_g * (m_Vehicle_lR / 0.57)
    fRz = m_Vehicle_m * m_g - fFz

    fFx = fFz * muFx
    fRx = fRz * muRx
    fFy = fFz * muFy
    fRy = fRz * muRy

    dot_X =cos(psi)*vx - sin(psi)*vy
    dot_Y = sin(psi)*vx + cos(psi)*vy

    t4 = time.time()

    next_state = zeros_like(state)
    next_state[:, 0] = vx + deltaT * ((fFx * cos(delta) - fFy * sin(delta) + fRx) / m_Vehicle_m + vy * wz)
    next_state[:, 1] = vy + deltaT * ((fFx * sin(delta) + fFy * cos(delta) + fRy) / m_Vehicle_m - vx * wz)
    next_state[:, 2] = wz + deltaT * ((fFy * cos(delta) + fFx * sin(delta)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz
    next_state[:, 3] = wF - deltaT * m_Vehicle_rF / m_Vehicle_IwF * fFx
    # next_state[:, 4] = wR + deltaT * (m_Vehicle_kTorque * (T-wR) - m_Vehicle_rR * fRx) / m_Vehicle_IwR
    next_state[:, 5] = psi + deltaT * wz
    next_state[:, 6] = X + deltaT * dot_X
    next_state[:, 7] = Y + deltaT * dot_Y

    t5 = time.time()

    # print(t1-t0, t2-t1, t3-t2, t4-t3, t5-t4)

    return next_state


def generate_predicted_states(file):
    mat = scipy.io.loadmat(file)
    measured_states = mat['states']
    dx, N = measured_states.shape
    controls = mat['inputs']
    du, N = controls.shape
    # for ii in range(N):
    state = update_dynamics(measured_states.T, controls.T)
    predicted_states = state.copy().T
    return predicted_states


class BicycleModel(torch.nn.Module):
    def __init__(self):
        super(BicycleModel, self).__init__()
        self.requires_grad_(False)

    def forward(self, state, control):
        t0 = time.time()
        state = state.cpu().detach().numpy()
        control = control.cpu().detach().numpy()
        next_state = update_dynamics(state, control)
        next_state = torch.from_numpy(next_state).to(torch.device('cuda'))
        # print(time.time()-t0)
        return next_state


if __name__ == '__main__':
    generate_predicted_states('single_track_cartesian_states_inputs.mat')
    # mat = scipy.io.loadmat('single_track_cartesian_states_inputs.mat')
    # measured_states = mat['states']
    # dx, N = measured_states.shape
    # controls = mat['inputs']
    # controls = torch.from_numpy(controls).float()
    # du, N = controls.shape
    # state = torch.from_numpy(measured_states[:, 0]).float()
    # predicted_states = np.zeros((dx, N))
    # for ii in range(N):
    #     state = update_dynamics(state, controls[:, ii])
    #     predicted_states[:, ii] = state.reshape((dx))
    # time = np.arange(N)
    # plt.figure()
    # for ii in range(dx):
    #     plt.subplot(4, 2, ii + 1)
    #     plt.plot(time, measured_states[ii, :])
    #     plt.plot(time, predicted_states[ii, :])
    # plt.show()

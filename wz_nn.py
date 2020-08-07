import torch
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
from torch import sin, cos, tan, atan, sqrt, atan2, zeros, zeros_like, abs


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
        # self.optimizer = torch.optim.Adam(self.model.parameters(), weight_decay=regularization, lr=learning_rate)
        self.optimizer = torch.optim.Adamax(self.model.parameters(), weight_decay=regularization, lr=learning_rate)

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


def prepare_data_steering():
    N0 = 0
    Nf = -1
    mat = scipy.io.loadmat('carsim_data/baseline_states_controls.mat')
    measured_states1 = mat['states'][:, N0:Nf:1]
    dx, N1 = measured_states1.shape
    controls1 = mat['inputs'][:, N0:Nf:1]
    du, N1 = controls1.shape
    mat = scipy.io.loadmat('carsim_data/baseline_states_controls.mat')
    measured_states2 = mat['states'][:, N0:Nf:1]
    dx, N2 = measured_states2.shape
    controls2 = mat['inputs'][:, N0:Nf:1]
    du, N2 = controls2.shape
    du = 3
    split = 0.5
    split = int(split * N2)
    training_inputs = np.zeros((du, 2*(split)))
    training_outputs = np.zeros((dx, 2 * (split)))
    # training_inputs[:dx, :] = predicted_states[:, :split]
    training_inputs[:, :split] = controls1[:du, :split]
    training_inputs[:, split:] = controls2[:du, :split]
    training_outputs[:, :split] = measured_states1[:, :split]
    training_outputs[:, split:] = measured_states2[:, :split]

    mat = scipy.io.loadmat('carsim_data/baseline_states_controls.mat')
    measured_states1 = mat['states'][:, :]
    dx, N1 = measured_states1.shape
    controls1 = mat['inputs'][:, :]
    _, N1 = controls1.shape

    validation_inputs = np.zeros((du, N1))
    # validation_inputs[:dx, :] = predicted_states[:, split:]
    validation_inputs[:, :] = controls1[:du, :]
    validation_outputs = np.zeros((dx, N1))
    validation_outputs[:, :] = measured_states1[:, :]
    return training_inputs, training_outputs, validation_inputs, validation_outputs


def steering_loss(delta, state, val=False):
    delta = delta[:,0]
    # deltar = delta[:,1]
    # print(torch.max(delta) - torch.min(delta))
    vx = state[:, 0]
    vy = state[:, 1]
    wz = state[:, 2]
    wF = state[:, 3]
    wR = state[:, 4]
    psi = state[:, 5]
    X = state[:, 6]
    Y = state[:, 7]

    dt = 0.05
    lFR = 2.910
    m_Vehicle_m = 1270#21.7562
    m_Vehicle_Iz = 1537#1.22
    m_Vehicle_lF = 1.015
    m_Vehicle_lR = lFR - m_Vehicle_lF
    m_Vehicle_IwF = 8
    m_Vehicle_rF = 0.325
    m_Vehicle_rR = 0.325
    m_Vehicle_h = .54
    m_g = 9.80665

    tire_B = 10
    tire_C = 2
    tire_D = 1
    tire_E = 1.0
    tire_Sh = -0.00
    tire_Sv = 0.00

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
    muFy = -tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv
    sEF = atan(vRy / abs(vRx)) + tire_Sh
    muRy = -tire_D * sin(tire_C * atan(tire_B * sEF - tire_E * (tire_B * sEF - atan(tire_B * sEF)))) + tire_Sv

    fFz = m_Vehicle_m * m_g * (m_Vehicle_lR - m_Vehicle_h * muRx) / (
                m_Vehicle_lF + m_Vehicle_lR + m_Vehicle_h * (muFx * cos(delta) - muFy * sin(delta) - muRx))
    fFz = m_Vehicle_m * m_g * (m_Vehicle_lR/0.57)
    fRz = m_Vehicle_m * m_g - fFz

    fFx = fFz * muFx
    fRx = fRz * muRx
    fFy = fFz * muFy
    fRy = fRz * muRy

    predicted = wz + dt * ((fFy * cos(delta) + fFx * sin(delta)) * m_Vehicle_lF - fRy * m_Vehicle_lR) / m_Vehicle_Iz
    errors = torch.abs(wz[5:] - predicted[:-5])
    # errors[torch.argmax(errors)] = 0
    sorty, _ = torch.sort(errors, descending=True)
    # print(sorty[:10])
    loss = torch.mean(errors)

    if val:
        return predicted
    else:
        return loss


def steering_loss2(delta, state, val=False):
    deltal = delta[:,0]
    deltar = delta[:,0]
    # print(torch.max(delta) - torch.min(delta))
    N = len(state)
    vx = state[:,0]
    vy = state[:,1]
    wz = state[:,2]
    w_lf = state[:,3]
    w_lr = state[:,4]
    psi = state[:,5]
    X = state[:,6]
    Y = state[:,7]
    vz = torch.zeros((N,1)).reshape(-1)#state[:,8]
    vtheta = torch.zeros((N,1)).reshape(-1)#state[:,9]
    vphi = torch.zeros((N,1)).reshape(-1)#state[:,10]
    z = 0#state[:,11]
    theta = 0#state[:,12]
    phi = 0#state[:,13]
    w_rf = w_lf
    w_rr = w_lr

    dt = 0.05
    m_Vehicle_m = 22.0262
    Iz = 1.2311
    lf = .3092
    lr = 0.57 - .3092
    h = .1159
    m_Vehicle_IwF = .0416
    m_Vehicle_IwR = .0373

    Cd = 0.03
    Ix = Iz / 4
    Iy = Iz
    Kf = 5500
    Kr = 5500
    Cf = 1000
    Cr = 1000
    L_axis = 0.57
    ms = 22.0262
    mu1 = .75
    mu2 = 0.9
    m = ms + mu1 + mu2
    Wa = 0.44
    Wb = 0.46
    mf = mu1 / 2
    mr = mu2 / 2
    g = 9.80665
    r_F = 0.095
    r_R = 0.095
    rho_A = 1.206
    Area = 1.8

    m_Vehicle_kSteering = 18.7861
    m_Vehicle_cSteering = 0.0109
    m_Vehicle_kThrottle = 204.0922
    m_Vehicle_kTorque = 0.1577

    tire_By = 1.9
    tire_Cy = 1.5
    tire_Dy = 1.1
    tire_E = 1.0
    tire_Sh = -0.00
    tire_Sv = 0.00
    tire_Bx = tire_By
    tire_Cx = tire_Cy
    tire_Dx = tire_Dy

    Vlf = torch.stack((vx,  vy, vz), dim=1).T + torch.cross(torch.stack((vphi, vtheta, wz), dim=1).T, torch.tensor((lf, Wa / 2, r_F - h)).reshape(3,1).repeat((1,N)), dim=0)
    Vrf = torch.stack((vx,  vy, vz), dim=1).T + torch.cross(torch.stack((vphi, vtheta, wz), dim=1).T, torch.tensor((lf, -Wa / 2, r_F - h)).reshape(3,1).repeat((1,N)), dim=0)
    Vlr = torch.stack((vx,  vy, vz), dim=1).T + torch.cross(torch.stack((vphi, vtheta, wz), dim=1).T, torch.tensor((-lr, Wb / 2, r_R - h)).reshape(3,1).repeat((1,N)), dim=0)
    Vrr = torch.stack((vx,  vy, vz), dim=1).T + torch.cross(torch.stack((vphi, vtheta, wz), dim=1).T, torch.tensor((-lr, -Wb / 2, r_R - h)).reshape(3,1).repeat((1,N)), dim=0)
    Vlf_x = cos(deltal) * Vlf[0, :] + sin(deltal) * Vlf[1,:]
    Vlf_y = -sin(deltal) * Vlf[0,:] + cos(deltal) * Vlf[1,:]
    Vrf_x = cos(deltar) * Vrf[0,:] + sin(deltar) * Vrf[1,:]
    Vrf_y = -sin(deltar) * Vrf[0,:] + cos(deltar) * Vrf[1,:]
    Vlr_x = Vlr[0]
    Vlr_y = Vlr[1]
    Vrr_x = Vrr[0]
    Vrr_y = Vrr[1]

    sEF = -(Vlf_x - w_lf * r_F) / (Vlf_x) + tire_Sh
    mu_lfx = tire_Dx * sin(tire_Cx * atan(tire_Bx * sEF - tire_E * (tire_Bx * sEF - atan(tire_Bx * sEF)))) + tire_Sv
    sEF = -(Vrf_x - w_rf * r_F) / (Vrf_x) + tire_Sh
    mu_rfx = tire_Dx * sin(tire_Cx * atan(tire_Bx * sEF - tire_E * (tire_Bx * sEF - atan(tire_Bx * sEF)))) + tire_Sv
    sEF = -(Vlr_x - w_lr * r_R) / (Vlr_x) + tire_Sh
    mu_lrx = tire_Dx * sin(tire_Cx * atan(tire_Bx * sEF - tire_E * (tire_Bx * sEF - atan(tire_Bx * sEF)))) + tire_Sv
    sEF = -(Vrr_x - w_rr * r_R) / (Vrr_x) + tire_Sh
    mu_rrx = tire_Dx * sin(tire_Cx * atan(tire_Bx * sEF - tire_E * (tire_Bx * sEF - atan(tire_Bx * sEF)))) + tire_Sv

    sEF = atan(Vlf_y / abs(Vlf_x)) + tire_Sh
    mu_lfy = -tire_Dy * sin(tire_Cy * atan(tire_By * sEF - tire_E * (tire_By * sEF - atan(tire_By * sEF)))) + tire_Sv
    sEF = atan(Vrf_y / abs(Vrf_x)) + tire_Sh
    mu_rfy = -tire_Dy * sin(tire_Cy * atan(tire_By * sEF - tire_E * (tire_By * sEF - atan(tire_By * sEF)))) + tire_Sv
    sEF = atan(Vlr_y / abs(Vlr_x)) + tire_Sh
    mu_lry = -tire_Dy * sin(tire_Cy * atan(tire_By * sEF - tire_E * (tire_By * sEF - atan(tire_By * sEF)))) + tire_Sv
    sEF = atan(Vrr_y / abs(Vrr_x)) + tire_Sh
    mu_rry = -tire_Dy * sin(tire_Cy * atan(tire_By * sEF - tire_E * (tire_By * sEF - atan(tire_By * sEF)))) + tire_Sv

    Flf0 = torch.tensor(m * g * lr / L_axis / 2).repeat(N)
    Frf0 = Flf0
    Flr0 = torch.tensor(m * g * lf / L_axis / 2).repeat(N)
    Frr0 = Flr0
    dFlf = Kf * (lf * theta - Wa / 2 * phi - z) + Cf * (lf * vtheta - Wa / 2 * vphi - vz)
    dFrf = Kf * (lf * theta + Wa / 2 * phi - z) + Cf * (lf * vtheta + Wa / 2 * vphi - vz)
    dFlr = Kr * (-lr * theta - Wb / 2 * phi - z) + Cr * (-lr * vtheta - Wb / 2 * vphi - vz)
    dFrr = Kr * (-lr * theta + Wb / 2 * phi - z) + Cr * (-lr * vtheta + Wb / 2 * vphi - vz)
    TireForce_z = torch.stack((Flf0 + dFlf, Frf0 + dFrf, Flr0 + dFlr, Frr0 + dFrr), dim=1)
    f_lfx = mu_lfx * TireForce_z[:, 0]
    f_lfy = mu_lfy * TireForce_z[:, 0]
    f_rfx = mu_rfx * TireForce_z[:, 1]
    f_rfy = mu_rfy * TireForce_z[:, 1]
    f_lrx = mu_lrx * TireForce_z[:, 2]
    f_lry = mu_lry * TireForce_z[:, 2]
    f_rrx = mu_rrx * TireForce_z[:, 3]
    f_rry = mu_rry * TireForce_z[:, 3]

    dot_r = ((f_lfy * cos(deltal) + f_rfy * cos(deltar) + f_lfx * sin(deltal) + f_rfx * sin(deltar)) * lf - (f_lry + f_rry) * lr) / Iz

    predicted = wz + dt * dot_r
    errors = torch.abs(wz[5:] - predicted[:-5])
    # errors[torch.argmax(errors)] = 0
    sorty, _ = torch.sort(errors, descending=True)
    # print(sorty[:10])
    loss = torch.mean(sorty[:1000]) + 0.22*torch.mean(torch.abs(deltal - deltar))

    if val:
        return predicted
    else:
        return loss


def steering_loss3(delta, state, val=False):
    delta = delta[:,0]
    # deltar = delta[:,1]
    # print(torch.max(delta) - torch.min(delta))
    vx = state[:, 0]
    vy = state[:, 1]
    wz = state[:, 2]

    dt = 0.05
    lFR = 2.910
    m_Vehicle_m = 1270#21.7562
    m_Vehicle_Iz = 1537#1.22
    m_Vehicle_lF = 1.015
    m_Vehicle_lR = lFR - m_Vehicle_lF
    m_Vehicle_IwF = 8
    m_Vehicle_rF = 0.325
    m_Vehicle_rR = 0.325
    m_Vehicle_h = .54
    m_g = 9.80665

    v = sqrt(vx**2 + vy**2)
    predicted = v / lFR * tan(delta)

    errors = torch.abs(wz[:] - predicted[:])
    # errors[torch.argmax(errors)] = 0
    sorty, _ = torch.sort(errors, descending=True)
    # print(sorty[:10])
    loss = torch.mean(errors)

    if val:
        return predicted
    else:
        return loss


def train_steering_model():
    dims = (10,)
    dyn_model = LinearNetwork(3, dims, 1, learning_rate=1e-3, regularization=0)
    dyn_model.loss_fn = steering_loss3
    training_inputs, training_outputs, validation_inputs, validation_outputs = prepare_data_steering()
    input_tensor = torch.from_numpy(training_inputs.T).float()
    output_tensor = torch.from_numpy(training_outputs.T).float()
    # print(training_inputs, input_tensor)
    # dyn_model.train(input_tensor, output_tensor, num_iters=5000)
    # torch.save(dyn_model.model.state_dict(), 'steering_net4.pth')

    dyn_model.model.load_state_dict(torch.load('steering_net4.pth'))
    deltas = dyn_model.validate(input_tensor)
    # print(deltas)
    wzs = steering_loss3(deltas, output_tensor, val=True)
    wzs = wzs.detach().numpy()
    deltas = deltas.detach().numpy()
    plt.figure()
    N = len(wzs)
    time = np.arange(N-5) / 100
    plt.plot(time, training_outputs[2, 5:], '.')
    plt.plot(time, wzs[:-5], '.')
    plt.show()

    input_tensor = torch.from_numpy(validation_inputs.T).float()
    output_tensor = torch.from_numpy(validation_outputs.T).float()
    deltas = dyn_model.validate(input_tensor)
    wzs = steering_loss3(deltas, output_tensor, val=True)
    wzs = wzs.detach().numpy()
    plt.figure()
    N = len(wzs)
    time = np.arange(N - 1) / 100
    plt.plot(time, validation_outputs[2, 1:], '.')
    plt.plot(time, wzs[:-1], '.')
    plt.show()
    input_tensor = torch.from_numpy(validation_inputs.T).float()
    deltas = dyn_model.validate(input_tensor)
    deltas = deltas.detach().numpy()
    mat = {'deltas': deltas}
    scipy.io.savemat('carsim_deltas2.mat', mat)


def add_labels():
    plt.subplot(4, 2, 1)
    plt.gca().legend(('Measured vx', 'Predicted vx', 'Corrected vx'))
    plt.xlabel('t (s)')
    plt.ylabel('m/s')
    plt.subplot(4, 2, 2)
    plt.gca().legend(('Measured vy', 'Predicted vy', 'Corrected vy'))
    plt.xlabel('t (s)')
    plt.ylabel('m/s')
    plt.subplot(4, 2, 3)
    plt.gca().legend(('Measured wz', 'Predicted wz', 'Corrected wz'))
    plt.xlabel('t (s)')
    plt.ylabel('rad/s')
    plt.subplot(4, 2, 4)
    plt.gca().legend(('Measured wF', 'Predicted wF', 'Corrected wF'))
    plt.xlabel('t (s)')
    plt.ylabel('rad/s')
    plt.subplot(4, 2, 5)
    plt.gca().legend(('Measured wR', 'Predicted wR', 'Corrected wF'))
    plt.xlabel('t (s)')
    plt.ylabel('rad/s')
    plt.subplot(4, 2, 6)
    plt.gca().legend(('Measured Yaw', 'Predicted Yaw', 'Corrected Yaw'))
    plt.xlabel('t (s)')
    plt.ylabel('rad')
    plt.subplot(4, 2, 7)
    plt.gca().legend(('Measured X', 'Predicted X', 'Corrected X'))
    plt.xlabel('t (s)')
    plt.ylabel('m')
    plt.subplot(4, 2, 8)
    plt.gca().legend(('Measured Y', 'Predicted Y', 'Corrected Y'))
    plt.xlabel('t (s)')
    plt.ylabel('m')


if __name__ == '__main__':
    train_steering_model()

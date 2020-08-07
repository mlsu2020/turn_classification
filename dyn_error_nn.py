import torch
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
# from bicycle_model import BicycleModel
import time
from python_nn import bicycle_model

from torch import sin, cos, atan, sqrt, atan2, zeros, zeros_like, abs


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
        #         torch.nn.init.zeros_(layer.weight)

        self.model = torch.nn.Sequential(*self.layers)
        self.model.to(self.cuda)
        self.loss_fn = torch.nn.L1Loss(reduction='mean')
        self.optimizer = torch.optim.Adam(self.model.parameters(), weight_decay=regularization, lr=learning_rate)

    def train(self, inputs, gt, num_iters=5000, ff=False):
        inputs = inputs.to(device=self.cuda)
        gt = gt.to(device=self.cuda)
        for ii in range(num_iters):
            # print(inputs)
            output = self.model(inputs[:,:])# + inputs[:, :8]
            if ff:
                output[:,0] += inputs[:,2]
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
        outputs = outputs.cpu().detach().numpy()
        return outputs


class RNN(torch.nn.Module):
    def __init__(self, idim, hdim, odim, learning_rate=1e-3, regularization=0):
        super(RNN, self).__init__()

        self.gpu = torch.device('cuda')
        bicycle = BicycleModel()
        rnn = torch.nn.RNNCell(2, 8)
        # rnn.weight_ih.data.fill_(0.01)
        # rnn.weight_hh.data.fill_(1.1)
        # rnn.bias_ih.data.fill_(0.01)
        # rnn.bias_hh.data.fill_(0.01)
        #initialize weights
        self.add_module('bicycle', bicycle)
        self.add_module('rnn', rnn)
        self.cuda(self.gpu)
        self.loss_fn = torch.nn.MSELoss(reduction='mean').to(device=self.gpu)
        # for param in self.parameters():
        #     print(type(param.data), param.size())
        self.optimizer = torch.optim.Adam(self.parameters(), weight_decay=regularization, lr=learning_rate)

    def forward(self, state, control):
        t0 = time.time()
        predicted_next_state = self.bicycle(state, control)
        t1 = time.time()
        error = self.rnn(control, predicted_next_state)
        t2 = time.time()
        corrected_next_state = predicted_next_state + error
        t3 = time.time()
        print(t1-t0, t2-t1, t3-t2)
        # print(corrected_next_state)
        return corrected_next_state

    def optimize(self, measured_states, controls, num_iters=5000):
        controls = controls.to(device=self.gpu)
        measured_states = measured_states.to(device=self.gpu)

        dx, N = measured_states.shape
        du, N = controls.shape
        controls = controls.reshape((1, du, N))
        for ii in range(num_iters):
            state = measured_states[:, 0]
            state = state.reshape((1, dx))
            predicted_states = torch.zeros_like(measured_states)
            # predicted_states = predicted_states.detach()
            # measured_states = measured_states.detach()
            # state = state.detach()
            # train over all data as batch
            for jj in range(N):
                state = self(state, controls[:, :, jj])
                predicted_states[:, jj] = state
            print((predicted_states - measured_states).sum())
            self.zero_grad()
            loss = self.loss_fn(predicted_states, measured_states)
            # if ii % 100 == 9:
            print(ii, loss.item())
            loss.backward()
            self.optimizer.step()


def wz_loss(corrected, measured):
    loss1 = torch.mean(torch.abs(corrected[:,0] - measured[:, 2]))

    wzs = corrected[:,0]
    wzs = wzs.reshape((4,-1))
    _, N = wzs.size()
    yaws = torch.zeros((4, 5))
    true_yaws = measured[:, 3].reshape((4, -1))
    jj=0
    indices = np.arange(start=int(N/5), stop=N+1, step=int(N/5))
    # print(indices)
    for ii in indices:
        yaws[:, jj] = torch.sum(wzs[:, 0:ii] * 0.01) + true_yaws[:,0]
        jj+=1
    # print(yaws)

    # yaw = torch.sum(wzs * 0.01) + true_yaws[0]
    loss2 = torch.sum(0.001 * torch.abs(yaws - true_yaws[:, indices-1]))
    # print(loss1, loss2)

    return loss1 #+ loss2


def prepare_training_data_for_prediction():
    N0 = 1000
    mat = scipy.io.loadmat('python_nn/single_track_cartesian_states_inputs.mat')
    states = mat['states'][:, N0:]
    dx, N = states.shape
    controls = mat['inputs'][:, N0:]
    du, N = controls.shape
    split = 0.667
    split = int(split*N)
    training_data = np.zeros((dx+du, split))
    training_data[:dx, :] = states[:, :split]
    training_data[dx:, :] = controls[:, :split]
    validation_data = np.zeros((dx+du, N-split))
    validation_data[:dx, :] = states[:, split:]
    validation_data[dx:, :] = controls[:, split:]
    return training_data, validation_data


def prepare_training_data_for_error():
    N0 = 2000
    Nf = -1000
    runs = [1,2,3,4]
    measured_states = []
    controls = []
    predicted_states = []
    measured_states2 = []
    controls2 = []
    predicted_states2 = []
    for ii, run in enumerate(runs):
        mat = scipy.io.loadmat('mppi_data/mppi_states_controls{}.mat'.format(run))
        measured_states.append(mat['states'][:, N0:Nf:10])
        # mat = scipy.io.loadmat('nn_deltas{}.mat'.format(run))
        controls.append(mat['inputs'][:1, N0:Nf:10] * 1000)
        # mat = scipy.io.loadmat('mppi_data/mppi_states_controls9.mat')
        measured_states2.append(mat['states'][:, :])
        # mat = scipy.io.loadmat('nn_deltas9.mat')
        controls2.append(mat['inputs'][:1, :] * 1000)
        dx, N1 = measured_states[0].shape
        dx, N2 = measured_states2[0].shape
        du, N1 = controls[0].shape
        mat = scipy.io.loadmat('mppi_data/predicted_states{}.mat'.format(run))
        predicted_states.append(mat['analytic_states'][:, N0:Nf:10])
        # mat = scipy.io.loadmat('mppi_data/full_predicted_states9.mat')
        predicted_states2.append(mat['analytic_states'][:, :])
    split = 0.75
    split = int(split * (N1))
    dx = 13
    # rand_indices = np.arange(0, N-1, 100)
    training_inputs = np.zeros((dx+du, len(runs) * split))
    training_outputs = np.zeros((4, len(runs) * split))
    for ii in range(len(runs)):
        training_inputs[:5, ii*split:split*(ii+1)] = predicted_states[ii][:5, :split]*1
        training_inputs[5:dx, ii * split:split * (ii + 1)] = predicted_states[ii][8:, :split]
        training_inputs[dx:, ii*split:split*(ii+1)] = controls[ii][:, :split]
        # training_inputs[:dx, split:] = predicted_states2[:dx, :split]
        # training_inputs[dx:, split:] = controls2[:, :split]
        training_outputs[:3, ii*split:split*(ii+1)] = measured_states[ii][:3, :split]
        training_outputs[3, ii*split:split*(ii+1)] = measured_states[ii][5, :split]

    # training_outputs[:, split:] = measured_states2[2, :split]
    validation_inputs_list = []
    validation_outputs_list = []
    for ii in range(len(measured_states2)):
        _, N2 = measured_states2[ii].shape
        validation_inputs = np.zeros((dx+du, N2))
        validation_inputs[:5, :] = predicted_states2[ii][:5, :]
        validation_inputs[5:dx, :] = predicted_states2[ii][8:, :]
        validation_inputs[dx:, :] = controls2[ii][:, :]
        validation_outputs = np.zeros((3, N2))
        validation_outputs[:3, :] = measured_states2[ii][:3, :]
        # validation_outputs[3, :] = measured_states1[5, :]
        validation_inputs_list.append(validation_inputs)
        validation_outputs_list.append(validation_outputs)
    return training_inputs, training_outputs, validation_inputs_list, validation_outputs_list


def train_and_validate_one_step_prediction():
    dims = (16, 16)
    dyn_model = LinearNetwork(10, dims, 8, learning_rate=5e-4)
    training_data, validation_data = prepare_training_data_for_prediction()
    input_tensor = torch.from_numpy(training_data[:, :-1].T).float()
    output_tensor = torch.from_numpy(training_data[:8, 1:].T).float()
    dyn_model.train(input_tensor, output_tensor, num_iters=10000)
    torch.save(dyn_model.model.state_dict(), 'net.pth')

    dyn_model.model.load_state_dict(torch.load('net.pth'))
    cuda = torch.device('cuda')
    predicted_states = dyn_model.model(input_tensor.to(device=cuda))
    plt.figure()
    predicted_states = predicted_states.T.cpu().detach().numpy()
    n, N = predicted_states.shape
    time = range(N)
    for ii in range(n):
        plt.subplot(4, 2, ii + 1)
        plt.plot(time, training_data[ii, 1:])
        plt.plot(time, predicted_states[ii, :])

    input_tensor = torch.from_numpy(validation_data[:, :-1].T).float()
    predicted_states = dyn_model.model(input_tensor.to(device=cuda))
    predicted_states = predicted_states.T.cpu().detach().numpy()
    n, N = predicted_states.shape
    time = np.arange(N) + time[-1]
    for ii in range(n):
        plt.subplot(4, 2, ii + 1)
        plt.plot(time, validation_data[ii, 1:])
        plt.plot(time, predicted_states[ii, :])
    plt.show()


def train_and_validate_error_correction():
    dims = (9, 4)
    dyn_model = LinearNetwork(14, dims, 1, learning_rate=1e-3, regularization=0)
    dyn_model.loss_fn = wz_loss
    training_inputs, training_outputs, validation_inputs_list, validation_outputs_list = prepare_training_data_for_error()
    input_tensor = torch.from_numpy(training_inputs.T).float()
    output_tensor = torch.from_numpy(training_outputs.T).float()
    dyn_model.train(input_tensor, output_tensor, num_iters=5000, ff=True)
    torch.save(dyn_model.model.state_dict(), 'error_net6.pth')

    dyn_model.model.load_state_dict(torch.load('error_net6.pth'))
    corrected_states = dyn_model.validate(input_tensor[:,:])
    # corrected_states = corrected_states.detach().numpy()
    corrected_states = corrected_states + training_inputs[[2], :].T
    # print(corrected_states.shape)
    plt.figure()
    N = len(corrected_states)
    time = np.arange(N) / 100
    plt.plot(time, training_outputs.T, '.')
    # plt.plot(time, training_inputs[2, :], '.')
    plt.plot(time, corrected_states, '.')
    corrected_states1 = corrected_states

    # add_labels()
    # plt.suptitle('Training')
    plt.show()
    wzs = {}
    for ii in range(len(validation_inputs_list)):
        plt.figure()
        validation_inputs = validation_inputs_list[ii]
        validation_outputs = validation_outputs_list[ii]
        input_tensor = torch.from_numpy(validation_inputs.T).float()
        corrected_states = dyn_model.validate(input_tensor[:,:])
        # corrected_states = corrected_states.detach().numpy()
        corrected_states = corrected_states + validation_inputs[[2],:].T
        N = len(corrected_states)
        # print(np.mean(np.abs(validation_inputs[:8, :] - validation_outputs)))
        # print(np.mean(np.abs(validation_inputs[:8, :] + corrected_states - validation_outputs)))
        time = np.arange(N)/100 + time[-1]
        plt.plot(time, validation_outputs.T, '.')
        plt.plot(time, validation_inputs[2, :], '.')
        plt.plot(time, corrected_states, '.')
        # plt.suptitle('Validation')
        # plt.show()
        wzs['wzs{}'.format(ii+1)] = corrected_states
    scipy.io.savemat('python_nn/nn_wzs.mat', wzs)

    # mat = scipy.io.loadmat('single_track_cartesian_states_inputs2.mat')
    # N0=1000
    # measured_states = mat['states'][:, N0:]
    # dx, N = measured_states.shape
    # controls = mat['inputs'][:, N0:]
    # du, N = controls.shape
    # controls = controls.reshape((1,du,N))
    # controls_tensor = torch.from_numpy(controls).float()
    # state = measured_states[:, 0].reshape((1,dx))
    # # state = torch.from_numpy(state).float()
    # predicted_states = np.zeros((dx, N))
    # for ii in range(N):
    #     state = bicycle_model.update_dynamics(state, controls[:,:, ii])
    #     if ii % 100 == 0:
    #         input_tensor = np.hstack((state, controls[:,:, ii]))
    #         input_tensor = torch.from_numpy(input_tensor).float()
    #         error = dyn_model.validate(input_tensor)
    #         state += error
    #     predicted_states[:, ii] = state.reshape((dx))
    # time = np.arange(N)
    # plt.figure()
    # for ii in range(dx):
    #     plt.subplot(4, 2, ii + 1)
    #     plt.plot(time, measured_states[ii, :])
    #     plt.plot(time, predicted_states[ii, :])
    # plt.show()


def train_and_validate_rnn():
    dyn_model = RNN(2, 8, 8, learning_rate=1e-4)
    training_data, validation_data = prepare_training_data_for_prediction()
    N0, Nf = (1000, 2000)
    input_tensor = torch.from_numpy(training_data[8:, N0:Nf]).float()
    output_tensor = torch.from_numpy(training_data[:8, N0:Nf]).float()
    dyn_model.optimize(output_tensor, input_tensor, num_iters=5000)
    torch.save(dyn_model.model.state_dict(), 'rnn_net.pth')

    # dyn_model.model.load_state_dict(torch.load('rnn_net.pth'))
    # corrected_states = dyn_model.validate(input_tensor)
    # corrected_states = corrected_states.T
    # plt.figure()
    # n, N = corrected_states.shape
    # time = range(N)
    # for ii in range(n):
    #     plt.subplot(4, 2, ii + 1)
    #     plt.plot(time, training_outputs[ii, :])
    #     plt.plot(time, training_inputs[ii, :])
    #     plt.plot(time, corrected_states[ii, :] + training_inputs[ii, :])
    #
    # input_tensor = torch.from_numpy(validation_inputs.T).float()
    # corrected_states = dyn_model.validate(input_tensor)
    # corrected_states = corrected_states.T
    # n, N = corrected_states.shape
    # time = np.arange(N) + time[-1]
    # for ii in range(n):
    #     plt.subplot(4, 2, ii + 1)
    #     plt.plot(time, validation_outputs[ii, :])
    #     plt.plot(time, validation_inputs[ii, :])
    #     plt.plot(time, corrected_states[ii, :] + validation_inputs[ii, :])
    # plt.show()


def corrected_prediction():
    dims = (6,6,)
    dyn_model = LinearNetwork(6, dims, 3, learning_rate=1e-3, regularization=0)
    dyn_model.model.load_state_dict(torch.load('error_net5a.pth'))
    N0 = 2200
    Nf = -1000
    # mat = scipy.io.loadmat('python_nn/single_track_cartesian_predicted_states.mat')
    # predicted_states = mat['analytic_states'][:, N0:10:Nf]
    mat = scipy.io.loadmat('mppi_data/mppi_states_controls1.mat')
    measured_states = mat['states'][:, N0:Nf:10]
    dx, N = measured_states.shape
    print(N)
    mat = scipy.io.loadmat('nn_deltas1.mat')
    controls = mat['deltas'].T[:, 20:]
    du, N = controls.shape
    print(N)
    state = measured_states[:, 0].reshape((1, dx))
    corrected_states = np.zeros_like(measured_states)
    error = np.zeros_like(state)
    for ii in np.arange(N):
        control = controls[:, ii].reshape((1, du))
        state = bicycle_model.update_dynamics(state, control)
        state[0,4] = measured_states[4, ii]
        # print(state)
        if ii % 10 == 9:
            tensor = np.concatenate((state[:,:5], control), axis=1)
            # print(tensor.shape)
            tensor = torch.from_numpy(tensor).float()
            error = dyn_model.validate(tensor)
            print(error)
            state[:, 0:3] = state[:, 0:3] + error
        corrected_states[:,ii] = state

    plt.figure()
    time = range(N)
    for ii in range(dx):
        plt.subplot(4, 2, ii + 1)
        plt.plot(time, measured_states[ii, :])
        # plt.plot(time, predicted_states[ii, :])
        plt.plot(time, corrected_states[ii, :])
    plt.show()


if __name__ == '__main__':
    train_and_validate_error_correction()
    # corrected_prediction()

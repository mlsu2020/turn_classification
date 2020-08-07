import os
import torch
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils
import pandas as pd
from PIL import Image
import numpy as np
import time


class ImageDataset(Dataset):
    """Face Landmarks dataset."""

    def __init__(self, csv_file, root_dir, transform=None):
        """
        Args:
            csv_file (string): Path to the csv file with annotations.
            root_dir (string): Directory with all the images.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.labels = pd.read_csv(csv_file)
        self.root_dir = root_dir
        self.transform = transform
        self.images = np.zeros((len(self.labels), 256, 320))
        for ii in range(len(self.labels)):
            img_name = os.path.join(self.root_dir,
                                    'img{}_m.png'.format(ii++1))
            image = Image.open(img_name)
            self.images[ii,:,:] = np.asarray(image)
        self.images_tensor = torch.from_numpy(self.images).float()
        self.labels_array = np.asarray(self.labels)
        self.labels_array += 1

    def __len__(self):
        return len(self.labels)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        image = self.images_tensor[idx,:,:]
        label = self.labels_array[idx, 0]
        sample = {'image': image, 'label': label}

        if self.transform:
            sample = self.transform(sample)

        return sample


class Net(torch.nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        self.mp1 = torch.nn.MaxPool2d(5)
        self.fc1 = torch.nn.Linear(3264, 300)
        self.fc2 = torch.nn.Linear(300, 30)
        self.fc3 = torch.nn.Linear(30, 10)
        self.fc4 = torch.nn.Linear(10, 3)
        self.relu = torch.nn.ReLU()

    def forward(self, x):
        n, h, w = x.shape
        x1 = self.mp1(x.reshape(n,1,h,w))
        h1 = self.relu(self.fc1(x1.flatten(1,-1)))
        h2 = self.relu(self.fc2(h1))
        h3 = self.relu(self.fc3(h2))
        output = self.fc4(h3)

        return output


def main():
    dataset = ImageDataset('turn_labels2.csv', 'road_unsupervised/map/')
    n = len(dataset)
    split = int(0.33 * n)
    indices = list(range(n))
    np.random.shuffle(indices)
    # print(indices)
    train_indices, val_indices = indices[split:], indices[:split]
    train_sampler = torch.utils.data.sampler.SubsetRandomSampler(train_indices)
    val_sampler = torch.utils.data.sampler.SubsetRandomSampler(val_indices)
    batch_size = 100
    num_epochs = 20
    train_dataloader = DataLoader(dataset, batch_size=batch_size, sampler=train_sampler)
    val_dataloader = DataLoader(dataset, batch_size=batch_size, sampler=val_sampler)
    net = Net()
    criterion = torch.nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(net.parameters(), lr=1e-4)
    t0 = time.time()
    for ii in range(num_epochs):
        losses = 0
        for bb, sample in enumerate(train_dataloader):
            images = sample['image']
            labels = sample['label']
            optimizer.zero_grad()
            outputs = net(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            losses += loss.item()
        print(losses / (bb+1))
    print('time:', time.time() - t0)
    torch.save(net.state_dict(), 'turn_classifier1.pt')

    net = Net()
    net.load_state_dict(torch.load('turn_classifier1.pt'))
    correct = 0
    with torch.no_grad():
        for sample in train_dataloader:
            images = sample['image']
            labels = sample['label']
            output = net(images)
            _, predicted = torch.max(output, 1)
            correct += (predicted == labels).sum().item()
    print('training accuracy', 100 * correct / (len(dataset) - split))
    correct = 0
    with torch.no_grad():
        for sample in val_dataloader:
            images = sample['image']
            labels = sample['label']
            output = net(images)
            _, predicted = torch.max(output, 1)
            correct += (predicted == labels).sum().item()
    print('validation accuracy', 100 * correct / split)


if __name__ == '__main__':
    main()


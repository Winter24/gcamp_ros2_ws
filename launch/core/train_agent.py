from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
import torch
import os
import time
from tqdm import tqdm

from core.datasets.dataset import Dataset
from core.models.model import CustomModel
from core.losses.loss_fn import LossFunction

class TrainAgent():
    def __init__(self, config):
        self.config = config
        self.device = config["device"]
        self.prev_val_loss = 1e6
        self.multi_gpu = config["multi_gpu"]


    def prepare_loaders(self):
        config = self.config["data"]
        aug_config = self.config["augmentation"]
        model_config = self.config["model"]

        train_dataset = Dataset( self.config["train"]["data"], config, aug_config, model_config["cls_encoding"], "train")
        self.train_loader = DataLoader(train_dataset, shuffle=True, batch_size=self.config["train"]["batch_size"])

        val_dataset = Dataset(self.config["val"]["data"], config, aug_config, model_config["cls_encoding"], "sth")
        self.val_loader = DataLoader(val_dataset, shuffle=True, batch_size=self.config["val"]["batch_size"])

    def build_model(self):
        learning_rate = self.config["train"]["learning_rate"]
        weight_decay = self.config["train"]["weight_decay"]
        lr_decay_at = self.config["train"]["lr_decay_at"]


        self.model = CustomModel(self.config["model"], self.config["data"]["num_classes"])
        if self.multi_gpu:
            self.model = torch.nn.DataParallel(self.model)
        self.model.to(self.device)
        self.loss = LossFunction(self.config["model"]["cls_encoding"])
        if self.config["train"]["use_differential_learning"]:
            dif_learning_rate = self.config["train"]["differential_learning_rate"]
            self.optimizer = torch.optim.Adam([{'params': self.model.backbone.parameters(), 'lr': dif_learning_rate[1]},
                                             {'params': self.model.header.parameters(), 'lr': dif_learning_rate[0]}], 
                                             lr=learning_rate, weight_decay=weight_decay)
        else:
            self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate, weight_decay=weight_decay)
  
        self.scheduler = torch.optim.lr_scheduler.MultiStepLR(self.optimizer, milestones=lr_decay_at, gamma=0.1)


    def train_one_epoch(self, epoch):
        total_loss = {}
        start_time = time.time()
        self.model.train()
        for data in tqdm(self.train_loader):
            for key in data:
                data[key] = data[key].to(self.device)

            self.optimizer.zero_grad()

            pred = self.model(data["voxel"])
            loss_dict = self.loss(pred, data)

            loss_dict["loss"].backward()
            self.optimizer.step()

            loss_dict["loss"] = loss_dict["loss"].item()
            for key in loss_dict:
                if key in total_loss:
                    total_loss[key] += loss_dict[key]
                else:
                    total_loss[key] = loss_dict[key]


            
        for key in total_loss:
            self.writer.add_scalar(key, total_loss[key] / len(self.train_loader), epoch)

        
        print("Epoch {}|Time {}|Training Loss: {:.5f}".format(
            epoch, time.time() - start_time, total_loss["loss"] / len(self.train_loader)))
            


    def train(self):
        self.prepare_loaders()
        self.build_model()
        self.make_experiments_dirs()
        self.writer = SummaryWriter(log_dir = self.runs_dir)

        start_epoch = 0
        if self.config["resume_training"]:
            model_path = os.path.join(self.checkpoints_dir, str(self.config["resume_from"]) + "epoch")
            if self.multi_gpu:
                self.model.module.load_state_dict(torch.load(model_path, map_location=self.config["device"]))
            else:
                self.model.load_state_dict(torch.load(model_path, map_location=self.config["device"]))
            start_epoch = self.config["resume_from"]
            print("successfully loaded model starting from " + str(self.config["resume_from"]) + " epoch") 
        
        for epoch in range(start_epoch + 1, self.config["train"]["epochs"]):
            self.train_one_epoch(epoch)

            if epoch % self.config["train"]["save_every"] == 0:
                path = os.path.join(self.checkpoints_dir, str(epoch) + "epoch")
                if self.multi_gpu:
                    torch.save(self.model.module.state_dict(), path)
                else:
                    torch.save(self.model.state_dict(), path)

            if (epoch + 1) % self.config["val"]["val_every"] == 0:
                self.validate(epoch)

            self.scheduler.step()


    def validate(self, epoch):
        total_loss = {}
        start_time = time.time()
        self.model.eval()
        with torch.no_grad():
            for data in self.val_loader:
                for key in data:
                    data[key] = data[key].to(self.device)
                

                pred = self.model(data["voxel"])
                loss_dict = self.loss(pred, data)

                loss_dict["loss"] = loss_dict["loss"].item()

                for key in loss_dict.keys():
                    if key in total_loss:
                        total_loss[key] += loss_dict[key]
                    else:
                        total_loss[key] = loss_dict[key]

        self.model.train()

        for key in total_loss.keys():
            self.writer.add_scalar(key, total_loss[key] / len(self.val_loader), epoch)

        print("Epoch {}|Time {}|Validation Loss: {:.5f}".format(
            epoch, time.time() - start_time, total_loss["loss"] / len(self.val_loader)))

        if total_loss["loss"] / len(self.val_loader) < self.prev_val_loss:
            self.prev_val_loss = total_loss["loss"] / len(self.val_loader)
            path = os.path.join(self.best_checkpoints_dir, str(epoch) + "epoch")
            if self.multi_gpu:
                torch.save(self.model.module.state_dict(), path)
            else:
                torch.save(self.model.state_dict(), path)
            


    def make_experiments_dirs(self):
        base = self.config["model"]["backbone"] + "_" + self.config["model"]["cls_encoding"] + "_" + self.config["note"] + "_" + self.config["date"] + "_" + str(self.config["ver"])
        path = os.path.join(self.config["experiments"], base)
        if not os.path.exists(path):
            os.mkdir(path)
        self.checkpoints_dir = os.path.join(path, "checkpoints")
        self.best_checkpoints_dir = os.path.join(path, "best_checkpoints")
        self.runs_dir = os.path.join(path, "runs")

        if not os.path.exists(self.checkpoints_dir):
            os.mkdir(self.checkpoints_dir)

        if not os.path.exists(self.best_checkpoints_dir):
            os.mkdir(self.best_checkpoints_dir)

        if not os.path.exists(self.runs_dir):
            os.mkdir(self.runs_dir)

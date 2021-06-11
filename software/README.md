## Setting up the environments

### install Ubuntu 18.04 on a computer with Nvidia GPU
[install Ubuntu 18.04 on a computer with Nvidia GPU](https://github.com/deepakkrishnamurthy/gravitymachine-research/blob/master/Setting%20up%20a%20new%20computer%20with%20Ubuntu%2018.04%20%2B%20GPU%20driver.md)

### set up the software environment
[set up the software environment](https://github.com/deepakkrishnamurthy/gravitymachine-research/blob/master/Setting%20up%20the%20software%20environment.md)

### install camera drivers
If you're using The Imaging Source cameras, follow instructions on https://github.com/TheImagingSource/tiscamera (if not already installed) 

If you're using Daheng cameras, follow instructions in the `drivers and libraries/daheng camera` folder

### enable access to serial ports without sudo

```
sudo usermod -aG dialout $USER
```
 
### (optional) install pytorch and torchvision on Jetson Nano
Follow instructions on https://forums.developer.nvidia.com/t/pytorch-for-jetson-nano-version-1-5-0-now-available/72048

```
sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install python3-pip libopenblas-base libopenmpi-dev 
pip3 install -U pip testresources setuptools
pip3 install Cython
wget https://nvidia.box.com/shared/static/3ibazbiwtkl181n95n9em3wtrca7tdzp.whl -o torch-1.5.0-cp36-cp36m-linux_aarch64.whl
pip3 install torch-1.5.0-cp36-cp36m-linux_aarch64.whl
```
```
sudo apt-get install libjpeg-dev zlib1g-dev
git clone --branch torchvision v0.6.0 https://github.com/pytorch/vision torchvision   # see below for version of torchvision to download
cd torchvision
sudo python3 setup.py install
```
## Using the software
Use one of the following to start the program
```
python3 main.py
python3 main_camera_only.py
```
To start the program when no cameras are connected, use
```
python3 main.py --simulation
```

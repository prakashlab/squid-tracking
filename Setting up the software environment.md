## Setting Up the Software Environments

### install software dependencies
```
sudo rm /var/lib/apt/lists/lock
sudo apt-get update
sudo apt install git
sudo apt-get install python3-pip
sudo apt-get install python3-pyqtgraph
sudo apt-get install python3-pyqt5
pip3 install --upgrade setuptools pip
pip3 install opencv-python==4.4.0.46 opencv-contrib-python==4.4.0.46
pip3 install qtpy pyserial lxml imutils scipy==1.1.0 pandas 
pip3 install torch==1.8.1+cu111 torchvision==0.9.1+cu111 torchaudio==0.8.1 -f https://download.pytorch.org/whl/torch_stable.html
```
Note that desipte CUDA 11.2 is installed with nvidia-driver-460, the above pytorch installation still works. For up-to-date pytorch release, check https://pytorch.org/get-started/locally/.

### install camera drivers
If you're using The Imaging Source cameras, follow instructions on https://github.com/TheImagingSource/tiscamera 
```
git clone https://github.com/TheImagingSource/tiscamera.git
cd tiscamera
# only works on Debian based systems like Ubuntu
sudo ./scripts/install-dependencies.sh --compilation --runtime
mkdir build
cd build

# With ARAVIS:
cmake -DBUILD_ARAVIS=ON ..
# Without ARAVIS
cmake -DBUILD_ARAVIS=OFF ..

make
sudo make install
```
If you're using Daheng cameras, follow instructions in the `drivers and libraries/daheng camera` folder. If the repo is cloned to the home directory, you can use the following.
```
cd '/home/prakashlab/gravitymachine-research/software/drivers and libraries/daheng camera/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.1911.9122'
./Galaxy_camera.run
```
```
cd '/home/prakashlab/gravitymachine-research/software/drivers and libraries/daheng camera/Galaxy_Linux_Python_1.0.1905.9081'
cd api
python3 setup.py build
sudo python3 setup.py install
```

### enable access to serial ports without sudo
```
sudo usermod -aG dialout $USER
```
Reboot the computer for the change to take effect.

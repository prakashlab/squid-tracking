# Installation of Scientific Python, CUDA, OpenCV-CUDA and TIS camera modules for gravity machine on Ubuntu 18.04 LTS. 

## These installations and dependencies correspond to the legacy version (1.0.0) of the code-base. 
---
## 1. Preliminaries

Install pip

	sudo apt-get install python3-pip

Install numpy

	pip3 install numpy

Install Scipy

	pip3 install scipy

Install pandas

	pip3 install pandas

Install Python Image Library

	pip3 install pillow

Install imutils
	
	pip3 install imutils

Install Serial

	pip3 install pyserial

Import cmocean
	
	sudo pip3 install cmocean 

Install matplotlib

	sudo pip3 install matplotlib

---
## 2. CUDA install: 
Important note: Need to ensure that UEFI secure boot is configured correctly. In particular, this needs to be enabled during Ubuntu installation and the key should be 'Enrolled' during the first reboot. Not doing this can cause lots of downstream issues installing NVIDIA drivers.


	sudo add-apt-repository ppa:graphics-drivers/ppa
	sudo apt update
	sudo ubuntu-drivers autoinstall

Reboot.

	sudo apt install nvidia-cuda-toolkit gcc-6

Once this completes

	nvcc --version

This should show the version of the drivers installed.

##3. Install dependencies for TIS camera (Need to install before OpenCV)
Build Dependency
	sudo apt-get install git g++ cmake pkg-config libudev-dev libudev1 libtinyxml-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libglib2.0-dev libgirepository1.0-dev libusb-1.0-0-dev libzip-dev uvcdynctrl python-setuptools libxml2-dev libpcap-dev libaudit-dev libnotify-dev autoconf intltool gtk-doc-tools python3-setuptools

Run time dependency

sudo apt-get install gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly libxml2 libpcap0.8 libaudit1 libnotify4 python3-pyqt5 python3-gi


### Build tiscamera

	git clone --recursive https://github.com/TheImagingSource/tiscamera.git
	cd tiscamera
	mkdir build
	cd build

	# With ARAVIS:
	cmake -DBUILD_ARAVIS=ON -DBUILD_GST_1_0=ON -DBUILD_TOOLS=ON -DBUILD_V4L2=ON -DCMAKE_INSTALL_PREFIX=/usr ..
	# Without ARAVIS
	cmake -DBUILD_ARAVIS=OFF -DBUILD_GST_1_0=ON -DBUILD_TOOLS=ON -DBUILD_V4L2=ON -DCMAKE_INSTALL_PREFIX=/usr ..

	make
	sudo make install


---------------------------------------------------------------------------------------------------
## 4. OpenCV installation (from source)
sources: 1. https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/
2. https://gist.github.com/raulqf/a3caa97db3f8760af33266a1475d0e5e
Against Pyimagesearch's advice, this installation will not use virtual environments since that causes issues while using the TIS camera module.

	
	sudo apt-get update

	sudo apt-get install build-essential cmake unzip pkg-config

	sudo apt-get install build-essential cmake unzip pkg-config

	sudo apt-get install libjpeg-dev libpng-dev libtiff-dev

	sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
	
	sudo apt-get install libxvidcore-dev libx264-dev

	sudo apt-get install libgtk-3-dev

	sudo apt-get install libatlas-base-dev gfortran

Get the official OpenCV release
	
	wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.4.zip

Get the OpenCV-contrib
	
	wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.4.zip

Unzip the two files
	
	unzip opencv.zip
	unzip opencv_contrib.zip

	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules -D WITH_CUDA=ON -D WITH_TBB=ON -D ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D WITH_CUBLAS=1 -D BUILD_EXAMPLES=ON -D WITH_GSTREAMER=ON ..


Build fails since the compiler used is greater than gcc-6. Trying to recompile using a specified compiler.
	
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules -D WITH_CUDA=ON -D WITH_TBB=ON -D ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D WITH_CUBLAS=1 -D BUILD_EXAMPLES=ON -D WITH_GSTREAMER=ON -D CMAKE_C_COMPILER=/usr/bin/gcc-6 -D CMAKE_CXX_COMPILER=/usr/bin/g++-6 ..

The above command with the compiler specified works!


## 5. Install OpenCV (Direct). This alternative to step 4 above.
	
	pip3 install opencv-python
	pip3 install opencv-contrib-python


Install pyqtgraph

	pip3 install pyqtgraph


Install gi

	sudo apt-get install python3-gi


Install Tk (for python3)

	sudo apt-get install python3-tk




If you got this far, you are now ready to run some really kickass experiments with gravity machine!



## 6. GUI dependencies (There are specific dependencies to make sure the Qt GUI works)

Make sure that only PyQt5 is installed and not alongside earlier versions

Install QtPy as a wrapper over PyQt5
	
	sudo pip3 install qtpy

Install opengl for 3D graphics

	sudo apt-get install python3-pyqt5.qtopengl


(OPTIONAL) Install the Qtsvg library for exporting vector graphics from Qt

	sudo apt-get install python3-pyqt5.qtsvg












# install pytorch
[Instructions](https://medium.com/@balaprasannav2009/install-tensorflow-pytorch-in-ubuntu-18-04-lts-with-cuda-9-0-for-nvidia-1080-ti-9e45eca99573)

	sudo pip3 install http://download.py)torch.org/whl/cu91/torch-0.4.0-cp36-cp36m-linux_x86_64.whl 

	sudo pip3 install torchvision


# Install v4l for help with debugging camera issues
$ sudo apt-get update
$ sudo apt-get install v4l-utils

Install pyserial

	sudo pip3 install pyserial

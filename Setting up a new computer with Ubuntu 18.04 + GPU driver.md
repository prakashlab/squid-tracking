## Instructions from 03/12/2022, on MSI Katana GF66 (i5-11400H, RTX3050)

**Step 1**: Download the image for Ubuntu 20.04.4 ([ubuntu-20.04.4-desktop-amd64.iso](https://releases.ubuntu.com/20.04/ubuntu-20.04.4-desktop-amd64.iso)) and create a bootable USB following the official instruction ([for Mac](https://ubuntu.com/tutorials/create-a-usb-stick-on-macos#1-overview), [for Windows](https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview), [for Ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)). (Note, with Ubuntu 20.04.3 and kernerl version 5.13.0-30, once the GPU driver is installed, the computer freezes all the time. The issue is resolved by release 20.04.4). 

**Step 1.5**: (For dual boot only). Boot into windows, then use disk management -> shrink volumes (you may run into the error "The Volume You Have Selected to Shrink May Be Corrupted", to resolve it, right click on the disk -> properties -> Tools -> Check -> Scan Drive -> Repair Drive -> Repair now).

**Step 2**: Insert the bootable USB drive to the new computer. Power on the computer. If you're using a MSI computer, press and hold the Del key to enter the BIOS manu. Set boot priority such that the computer boots from the inserted USB drive. Disable the Secure Boot support.

**Step 3**: Follow the instructions to finish installing Ubuntu. Make sure you check "Install third-party software ...". If the computer has both a SSD and a mechanical drive, make sure install to the SSD. 

**Step 4**: After Ubuntu is successfully installed, you will still boot into windows by default. To fix this, press the Delete key on booting to enter the BIOS menu, then Boot -> UEFI Hard Disk Drive BBS Priorities and make Ubuntu the first option.

**Step 5**: The GPU driver should already be installed. To check, run
```
nvidia-smi
```

## Instructions from 11/19/2021, on MSI Aegis RS 10TD-213US (i7-10700K, RTX2070)

**Step 1**: Download the image for Ubuntu 18.04.5 ([ubuntu-18.04.5-desktop-amd64.iso](https://old-releases.ubuntu.com/releases/18.04.5/ubuntu-18.04.5-desktop-amd64.iso)) and create a bootable USB following the official instruction ([for Mac](https://ubuntu.com/tutorials/create-a-usb-stick-on-macos#1-overview), [for Windows](https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview), [for Ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)).

**Step 2**: Insert the bootable USB drive to the new computer. Power on the computer. If you're using a MSI computer, press and hold the Del key to enter the BIOS manu. Set boot priority such that the computer boots from the inserted USB drive.

**Step 3**: Follow the instructions to finish installing Ubuntu. Make sure you check "Install third-party software ...". You can keep the windows installation for dual boot. If the computer has both a SSD and a mechanical drive, make sure install to the SSD. If you are having issue adjusting the windows partition size, you'll need to stop the installation, boot into windows, then use disk management -> shrink volumes (you may run into the error "The Volume You Have Selected to Shrink May Be Corrupted", to resolve it, right click on the disk -> properties -> Tools -> Check -> Scan Drive -> Repair Drive -> Repair now).

**Step 4**: After Ubuntu is successfully installed, you will still boot into windows by default. To fix this, press the Delete key on booting to enter the BIOS menu, then Boot -> UEFI Hard Disk Drive BBS Priorities and make Ubuntu the first option.

**Step 5**: Install the GPU driver
```
sudo apt-get update
sudo apt install nvidia-driver-470
```
Reboot, and check if the driver is successfully installed
```
nvidia-smi
```

## Instructions from 6/10/2021, on MSI computers from 2019 (i7-9700K, RTX2070 and RTX2080)

**Step 1**: Download the image for Ubuntu 18.04.5 ([ubuntu-18.04.5-desktop-amd64.iso](https://old-releases.ubuntu.com/releases/18.04.5/ubuntu-18.04.5-desktop-amd64.iso)) and create a bootable USB following the official instruction ([for Mac](https://ubuntu.com/tutorials/create-a-usb-stick-on-macos#1-overview), [for Windows](https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview), [for Ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)).

**Step 2**: Insert the bootable USB drive to the new computer. Power on the computer. If you're using a MSI computer, press and hold the Del key to enter the BIOS manu. Set boot priority such that the computer boots from the inserted USB drive.

**Step 3**: Before selecting "install Ubuntu", press E to enter the grub menu, use the array keys on the keyboard to navigate the cursor before `quiet splash`, add `nomodeset`, Ctrl + X to save.

**Step 4**: Follow the instructions to finish installing Ubuntu. Make sure you check "Install third-party software ..." and "Configure secure boot"

**Step 5**: When the computer restart, make sure you choose "Enroll MOK".

**Step 6**: Then keep the right SHIFT key pressed so that you enter grub. Then press E -> add `nomodeset` before `quiet splash` -> CTRL+X (like Step 3). ([reference](https://askubuntu.com/questions/38780/how-do-i-set-nomodeset-after-ive-already-installed-ubuntu))

**Step 7**: After logging in, open terminal and
```
sudo gedit /etc/default/grub
```
Add `nomodeset` before `quiet splash`. Save the file. Then
```
sudo update-grub
```
This will fix the issue permanently

**Step 8**: Install the GPU driver
```
sudo apt-get update
sudo ubuntu-drivers autoinstall
```

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

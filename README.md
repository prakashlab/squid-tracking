# Squid-tracking

Squid (Simplifying Quantitive Imaging Development and Deployment) [1] provides a full suite of hardware and software components for rapidly configuring high-performance microscopes tailored to users' applications with reduced cost, effort and turnaround time. Besides increasing accessibility of research microscopes and available microscope hours to labs, it is also designed to simplify development and dissemination of new or otherwise advanced microscopy techniques. For more information, visit our website: https://www.squid-imaging.org

This repo hosts the current tracking software, which was first developed for scale-free vertical tracking microscopy (a.k.a Gravity Machine) [2], refactored with the new squid code base, and now applied back to squid for it to support tracking microscopy.

## References
[1] Hongquan Li, Deepak Krishnamurthy, Ethan Li, Pranav Vyas, Nibha Akireddy, Chew Chai, Manu Prakash, "**Squid: Simplifying Quantitative Imaging Platform Development and Deployment**." BiorXiv [ link | [website](https://www.squid-imaging.org)]

[2] Deepak Krishnamurthy, Hongquan Li, François Benoit du Rey, Pierre Cambournac, Adam G. Larson, Ethan Li, and Manu Prakash. "**Scale-free vertical tracking microscopy.**" Nature Methods 17, no. 10 (2020): 1040-1051. [ [link](https://www.nature.com/articles/s41592-020-0924-7) | [website](https://gravitymachine.org) ]

## Software Instructions
The microscope is controled by an Arduino Due and a computer running Ubuntu. The computer can be one of the Nvidia Jetson platforms (e.g. Jetson Nano, Jetson Xavier NX) or a regular laptop/workstation. For tracking fast moving objects, a laptop or workstation is recommended. Using a 2019 MSI Gaming Laptop (MSI GE65 9SF Raider-051 with Intel i7-9750H CPU, Nvidia RTX 2070 GPU, 2 x 8 GB 2666 MHz DDR4 RAM and a Samsung 970 EVO Plus NVMe M.2 SSD) we were able to track at > 100 fps. Instructions for using the firmware and software can be found in the respective folders.

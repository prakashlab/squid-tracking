# Scale-free Vertical Tracking Microscopy (aka Gravity Machine)

Gravity Machine is a new paradigm in tracking microscopy that uses a "hydrodynamic treadmill for single cells" invented by [Krishnamurthy at al.](https://www.nature.com/articles/s41592-020-0924-7) to track microscale organisms like marine plankton over theoretically infinite scales in the vertical direction. This repo contains the software (Python) and firmware (C++) for running Gravity Machine. This work is jointly developed with other open-microscopy platforms specificallty [Octopi](https://github.com/hongquanli/octopi-research) and [SQUID](https://github.com/prakashlab/squid-tracking) from the [Prakash lab, Stanford](https://github.com/prakashlab). The repo documents both legacy and current versions of the code, as we move towards an initial software release. Watch this space for updates! 

Below we briefly discuss the scientific challenges and puzzles that motivated this tool!

## The challenge

> "How to track a single cell at microscale resolution while allowing free vertical movement over ecological-scales?" 

For the last 300 years of history of microscopy, majority of subjects of the microscopic world have been stuck under a cover slip in the X-Y plane (horizontal plane). But many phenomena occur far from such confinements, for instance, plankton swimming and sinking particles suspended in the ocean where gravity is the one constant. So how can we observe microscopic life and physical processes that canntraverse ecological length scales?

## The Idea
	
To address this challenge we invented the nearest thing we could imagine to an "endless" water-column using a simple insight "A circle has no beginning (or end)": The result is a microscope stage which functions as a "hydrodynamic treadmill" for single-cells. We describe the tool in the following paper [Nature Methods](https://www.nature.com/articles/s41592-020-0924-7), [Biorxiv](https://www.biorxiv.org/content/10.1101/610246v1).

## What can you do with Gravity Machine?

It allows you to capture never-before seen dynamics of processes that span multiple scales, for instance, marine microscale plankton swimming many meters along the vertical axis, allowing measurement of their free-swimming behavior.

To explore more visit the [Gravity Machine website](https://gravitymachine.org/) and our associated [data-gallery](https://gravitymachine.org/gallery) of the first-ever multi-scale plankton tracks.

## Getting Started

The tracking microscope is run using software (written in Python) that runs on standard PC, and handles the real-time image-processing and GUI interaction with the user, along with firmware that runs on a microcontroller (Arduin Due). The code for each of those can be found within the *software* and *firmware* folders, respectively. 

## News
**5 Dec 2020** We are currently working towards a release of the latest software and firmware versions. Please watch this space for updates!

# Selected Publications
1. Krishnamurthy, Deepak, Hongquan Li, François Benoit du Rey, Pierre Cambournac, Adam G. Larson, Ethan Li, and Manu Prakash. "Scale-free vertical tracking microscopy." Nature Methods (2020): 1-12. [Weblink](https://www.nature.com/articles/s41592-020-0924-7)










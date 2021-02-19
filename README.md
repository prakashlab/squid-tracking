# Scale-free Vertical Tracking Microscopy (aka Gravity Machine)

Gravity Machine is a new paradigm in tracking microscopy invented by [Krishnamurthy et al.](https://www.nature.com/articles/s41592-020-0924-7) that uses a "hydrodynamic treadmill for single cells"  to track microscale organisms like marine plankton over theoretically infinite scales in the vertical direction while maintaining microscale resolution. In this repo we share the software (Python) and firmware (C++) for running Gravity Machine. This is jointly developed with other open-microscopy platforms, specifically [Octopi](https://github.com/hongquanli/octopi-research) and [SQUID](https://github.com/prakashlab/squid-tracking) being developed at the [Prakash lab, Stanford](https://github.com/prakashlab). The repo documents both legacy and current versions of the code-base. As we move towards an initial release, kindly watch this space for updates! 

Below we briefly discuss the scientific challenges and puzzles that motivated this tool!

## The challenge

> "How to track a single cell at microscale resolution while allowing free vertical movement over ecological-scales?" 

For the last 300 years of history of microscopy, majority of subjects of the microscopic world have been stuck under a cover slip in the X-Y plane (horizontal plane). But many phenomena occur far from such confinements, for instance, plankton swimming and sinking particles suspended in the ocean where gravity is the one constant. So how can we observe microscopic life and physical processes that can traverse ecological length scales?

## The Idea
	
To address this challenge we invented the nearest thing we could imagine to an "endless" water-column using a simple insight "A circle has no beginning (or end)": The result is a microscope stage which functions as a "hydrodynamic treadmill for single-cells". This stage becomes the basis for a 3D tracking microscope which allows theoretically infinite movement along the vertical direction, and also free movement (compared to the organism size) in the two horizontal directions. We describe the tool, and results based on the new measurement paradigm it opens up, in the following papers [Nature Methods](https://www.nature.com/articles/s41592-020-0924-7), [Biorxiv](https://www.biorxiv.org/content/10.1101/610246v1).

## What can you do with Gravity Machine?

Gravity Machine allows you to capture dynamics of processes that can span multiple length and time-scales, for instance, marine microscale plankton swimming many meters along the vertical axis, allowing direct measurement of their free-swimming behavior. Using Gravity Machine we have, for the first time, observed the multi-scale behavior of marine inventrebrate larve, observed [diel vertical migrations](https://en.wikipedia.org/wiki/Diel_vertical_migration) at the scale of single cells and organisms, measured flow-fields around freely swimming plankton, as well as observed multiscale processes associated with sedimenting particles and ["marine snow"](https://en.wikipedia.org/wiki/Marine_snow). In short it opens up a new measurement paradigm in fields such as ocean biophysics, marine ecology, marine biology and fluid mechanics, among others. 

To explore more visit the [Gravity Machine website](https://gravitymachine.org/) and our associated [data-gallery](https://gravitymachine.org/gallery) of the first-ever multi-scale plankton tracks.

## Getting Started

The tracking microscope is run using software (Python) that runs on standard PC, and handles the real-time image-processing and GUI interaction with the user, along with firmware (C++) that runs on a microcontroller ([Arduino Due](https://store.arduino.cc/usa/due)). The code for each of those can be found within the *software* and *firmware* folders, respectively. 

## News
**5 Dec 2020** We are currently working towards a release of the latest software and firmware versions. Please watch this space for updates!

## Selected Publications
1. Krishnamurthy, Deepak, Hongquan Li, François Benoit du Rey, Pierre Cambournac, Adam G. Larson, Ethan Li, and Manu Prakash. "Scale-free vertical tracking microscopy." Nature Methods (2020): 1-12. [Weblink](https://www.nature.com/articles/s41592-020-0924-7)
2. Krishnamurthy, Deepak, Hongquan Li, François Benoit du Rey, Pierre Cambournac, Adam Larson, and Manu Prakash. "Scale-free Vertical Tracking Microscopy: Towards Bridging Scales in Biological Oceanography." bioRxiv (2019): 610246.

## To cite this tool
	@article{krishnamurthy2020scale,
	  title={Scale-free vertical tracking microscopy},
	  author={Krishnamurthy, Deepak and Li, Hongquan and du Rey, Fran{\c{c}}ois Benoit and Cambournac, Pierre and Larson, Adam G and Li, Ethan and Prakash, Manu},
	  journal={Nature Methods},
	  volume={17},
	  number={10},
	  pages={1040--1051},
	  year={2020},
	  publisher={Nature Publishing Group}
	}








# Stereo Vision App (Steviapp)

Steviapp is a basic, research oriented bundle adjustement application

## Dependencies

To install the dependencies on ubuntu (tested for 24.04, but should work also for 22.04) run

    apt install -y build-essential cmake git pkg-config libeigen3-dev libproj-dev cimg-dev libceres-dev libgoogle-glog-dev libexiv2-dev qtbase5-dev qt5-qmake pybind11-dev

If you are using a different linux distribution, or a different operating system, 
you need to install the corresponding packages from the package manager of your system.

The application will also pull some external modules, either using git submodules or cmake fetchcontent.
As devellopement progress, efforts are being made to migrate more and more of these external modules to cmake fetchcontent.

## Get the code

You can get the code using git, pay attention to init and sync the submodules:

```
mkdir src
cd src
git clone https://github.com/french-paragon/steviapp.git
cd steviapp
git submodule init --recursive
git submodule update --recursive
```

## Build

The building process is straighforward with cmake:

```
mkdir build
cd ./build
cmake ../path/to/src/steviapp
make
```

The buildTests and buildExamples options are OFF by default.
        
## Citation

If you are using this tool during your research, please cite my PhD 
dissertation for which the tool has been originally develloped:

```
@phdthesis{f2affe5fb56d4bff947751468315d31e,
title = "Advances in Stereo Reconstruction towards Improved Holographic Communication",
abstract = "Recent advances in holographic displays and depth cameras promise to turn long-distance 3D holographic communication into reality. However, despite the advent of RGBD cameras, reconstructing a dynamic scene in real-time from every possible viewpoint is still a challenge. This thesis addresses the challenges associated to real-time stereo reconstruction, which constitutes the foundation of the holographic communication pipeline.",
keywords = "Stereo Vision, Sub-pixel accuracy, Bayesian Statistics",
author = "Jospin, \{Laurent Valentin\}",
year = "2023",
doi = "10.26182/86gs-h749",
language = "English",
school = "The University of Western Australia",
}
```

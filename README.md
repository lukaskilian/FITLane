# FITLane
FITLane is a **F**ramework for **I**mplementing and **T**esting of **Lane**-keeping systems.
It was developed in 2020 as a prototype for a framework for evaluating artificial intelligence methods for application in lane-keeping systems by Lukas Kilian as part of a bachelor's thesis at the University of Oldenburg.

The framework helps to perform automated tests of lane-keeping systems in a software simulation.
The framework uses external simulation software. For this purpose, a generic interface was defined and implemented for the [CARLA Simulator](https://github.com/carla-simulator/carla).

### Getting started

- Download CARLA Simulator. FITLane was tested with [CARLA Simulator Version 0.9.9.4](https://github.com/carla-simulator/carla/releases/tag/0.9.9).
- Download source code.
- Install dependencies. You can use the package manager conda and the command `conda env create -f conda_environment.yml`
- Copy and adapt `main_template.py`
- Start CARLA simulator
- Run `main` Python script

### Further notes
The framework is currently not being actively updated.  
The source code contains annotations in German.

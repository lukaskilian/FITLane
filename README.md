# FITLane
FITLane is a **F**ramework for **I**mplementing and **T**esting of **Lane**-keeping systems.
It was developed in 2020 as a prototype for a framework for evaluating artificial intelligence methods for application in lane-keeping systems by Lukas Kilian as part of a bachelor's thesis at the University of Oldenburg.

The framework was developed in Python and helps to perform automated tests of lane-keeping systems in a software simulation.
The framework uses external simulation software. For this purpose, a generic interface was defined and implemented for the [CARLA Simulator](https://github.com/carla-simulator/carla).

FITLane can also be used to generate training data for Deep Learning. In addition to the framework, the repository also contains an end-to-end learning lane-keeping agent.

### Getting started

- Download CARLA Simulator. FITLane was tested with [CARLA Simulator Version 0.9.9.4](https://github.com/carla-simulator/carla/releases/tag/0.9.9).
- Install dependencies. You can use the package manager conda and the command `conda env create -f conda_environment.yml`
- Copy CARLA Python package (`.egg` file) to `src/framework/carla_simulator/`
- Copy and adapt `main_template.py`
- Start CARLA simulator
- Run `main` Python script

### Further notes
The framework is currently not being actively updated.  
The source code contains comments and logging messages in German.

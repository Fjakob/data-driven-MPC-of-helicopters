# Data-driven MPC of helicopters

Data-driven Model Predictive Control of 3-DoF Helicopters including mathematical modelling, data generation and control algorithm implementation. This repo is the result of a part-time student employement at the [Institute of Systems Theory and Automatic Control](https://www.ist.uni-stuttgart.de/) at the University of Stuttgart under the supervision of Prof. Frank Allgöwer.

### Requirements

* [Matlab 2020b](https://de.mathworks.com/company/newsroom/mathworks-introduces-release-2020b-of-matlab-and-simulink.html)
* [qpOASES](https://github.com/coin-or/qpOASES) (will be pulled as submodule upon cloning)


### Running the Code

The [scripts](/src/scripts/) directory contains the main scripts to run all relevant Simulink models with the controllers. The controllers are initialized in each respective script and implemented in the respective model. The main scripts are

    quadprog_fixed_data.m
    quadprog_updated_data.m
    quadprog_scheduled_data.m
    qpOases_uncondensed.m
    qpOases_condensed.m

The qpOASES library has to be compiled first to run the related scripts/models. It is recommended to read the manual first. After cloning/pulling the repo, in the [core](/src/core/) folder, move the adjusted file `qpOASES_SQProblem` into the `interfaces\simulink` subdirectory in qpOASES to allow for computation time restriction.


### Repo Structure

```
data-driven-MPC-of-helicopters
│   README.md  
│
└───data
│   │   datatraj_elevation.mat
│   │   datatraj_travel.mat
│   |   qpMat_elevation.mat
|   └───qpMat_travel.mat
│
└───doc
│   └───Data-driven-MPC-of-3-DoF-Helicopters.pdf
│
└───src
│   └───core
|   |   |   @qpOASES
|   |   └───qpOASES_SQProblem.cpp
|   |
│   └───functions
|   |   └───hankel_c.m
|   |
│   └───models
|   |   |   sim_generate_data.slx
|   |   |   sim_qpOases_condensed.slx
|   |   |   sim_qpOases_uncondensed.slx
|   |   |   sim_quadprog_fixed_data.slx
|   |   |   sim_quadprog_scheduled_data.slx
|   |   └───sim_quadprog_updated_data.slx
|
│   └───scripts
|   |   |   generate_data.m
|   |   |   plots.m
|   |   |   qpOases_condensed.m
|   |   |   qpOases_uncondensed.m
|   |   |   quadprog_fixed_data.m
|   |   |   quadprog_schedule_data.m
|   |   |   quadprog_update_data.m
|   |   └───setup.m
```

### Documentation

A detailed [documentation](/doc/Data-driven-MPC-of-3-DoF-Helicopters.pdf) is given in the `doc` directory.


# Data-driven MPC of helicopters

Data-driven Model Predictive Control of 3-DoF Helicopters including mathematical modelling, data generation and control algorithm implementation.

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


### Documentation

A detailed [documentation](/doc/Data-driven-MPC-of-3-DoF-Helicopters) is given in the `doc` directory.

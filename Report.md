## Rubric Points

### The Model
The model used is a Kinematic model which is a  simplification of dynamic model that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics.

The states for the model are:
* `x, y`: vehicle position
* `psi`: orientation angle
* `v`: velocity of the vehicle
* `cte`: cross-track error
* `epsi`: orientation error

The actuators are:
* `delta`: steering angle
* `a`:  acceleration

The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below:

![equations](./eqns.png)

The objective is to find the acceleration (`a`) and the steering angle (`delta`) in the way it will minimize an objective function that is the combination of different factors:

* Square sum of `cte` and `epsi`
* Square sum of the difference actuators to penalize a lot of actuator's actions
* Square sum of the difference between two consecutive actuator values to penalize sharp changes

### Timestep Length and Elapsed Duration (N & dt)

Given the state and update functions described above, the MPC calculates a number of timesteps ahead of it's current state and measures if the predicted outcome of the current trajectory is adequate or not.

#### Number of Timesteps (N)

`N` determines how many timesteps will be calculated at each point in time. However, large number of timestep will incur more computational cost as for each timestep, the number of variable to be optimized is also increased.

#### Timestep duration

`dt` is how much time elapses between actuations.

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

#### Time Horizon (N*dt)
The prediction horizon (T) is the duration over which future predictions are made.

In the case of driving a car, `T` should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.

This project uses N=20 and dt=0.05. These values mean that the optimizer is considering a one-second duration in which to determine a corrective trajectory. The other values tried was N=10 and dt=0.1.

### Polynomial Fitting and MPC Preprocessing

A 3rd order polynomial is used to fit the waypoints. The fitting process is in the car coordinates, so that the waypoints are transformed from the map coordinates to the car coordinates (see `main.cpp` lines #103-113). These polynomial coefficients are used to calculate the `cte` and `epsi` later on. They are used by the solver as well to create a reference trajectory.

Weights for each cost item were tuned manually to find a working solution.
* 60, 150, 1 for the reference states
* 60, 60, for the actuators
* 30000, 9000 for sequential actuation

Large weight on the cost related to the change of delta is used to ensure a smooth change of steering angle. The target speed is set for 120mph.

### Model Predictive Control with Latency

In order to account the latency, we use the solution from the solver at a t=0.1s delay (`Main.cpp` line #130-139, `MPC.cpp` line #238-241).

---
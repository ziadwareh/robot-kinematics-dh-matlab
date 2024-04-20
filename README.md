# robot-kinematics-dh-matlab
This MATLAB class provides functionalities for robot kinematics modeling using the Denavit-Hartenberg (DH) convention. It supports both forward and inverse kinematics computations, including position, velocity, and acceleration kinematics.

## Features

The class provides the following functionalities:

- Forward and inverse kinematics modeling using the DH convention.
- Computation of forward position, velocity, and acceleration kinematics.
- Computation of inverse position, velocity, and acceleration kinematics.
- Calculation of Jacobian matrices (Jacobian, its inverse, and its derivative) and display as symbolic matrices.

<!-- 
### Forward Kinematics

- `forward_position_kinematics(q)`: Computes the forward position kinematics given joint angles `q`.
- `forward_velocity_kinematics(q, q_dot)`: Computes the forward velocity kinematics given joint angles `q` and joint velocities `q_dot`.
- `forward_acceleration_kinematics(q, q_dot, q_double_dot)`: Computes the forward acceleration kinematics given joint angles `q`, joint velocities `q_dot`, and joint accelerations `q_double_dot`.

### Inverse Kinematics

- `inverse_kinematics_func(q0, X, TOL, MaxNoOfIterations)`: Performs inverse position kinematics numerically.
- `inverse_velocity_kinematics(q, x_dot)`: Performs inverse velocity kinematics given joint angles `q` and desired end effector velocities `x_dot`.
- `inverse_acceleration_kinematics(q, q_dot, x_double_dot)`: Performs inverse acceleration kinematics given joint angles `q`, joint velocities `q_dot`, and desired end effector accelerations `x_double_dot`. -->

## Public Attributes

- `DH_table`: Matrix representing the DH table for the robot.
- `task_space_size`: Size of the task space.
- `EE_pose`: Current position of the end effector.
- `FPK`: Forward position kinematics output.
- `FVK`: Forward velocity kinematics output.
- `FAK`: Forward acceleration kinematics output.
- `q`: Joint angles from inverse position kinematics.
- `q_dot`: Joint velocities from inverse velocity kinematics.
- `q_double_dot`: Joint accelerations from inverse acceleration kinematics.

## Usage

### Constructor

#### `robot_kinematics(DH_table, task_space_size, rigid_transform, symbolic_params)`

Creates an instance of the `robot_kinematics` class.

- `DH_table`: A matrix representing the DH parameters of the robot.
- `task_space_size`: The size of the task space.
- `rigid_transform`: Optional rigid transformation matrix.
- `symbolic_params`: Boolean flag indicating whether symbolic analysis is desired.

### Display Functions

#### `display_EE_constants()`

Prints the symbolic parameters in the end effector position equations.

#### `display_EE_pose()`

Displays the position of the end effector.

#### `display_symbolic_jacobian()`

Displays the symbolic Jacobian matrix.

#### `display_jacobian()`

Displays the numerical Jacobian matrix.

#### `display_symbolic_inverse_jacobian()`

Displays the symbolic inverse Jacobian matrix.

#### `display_inverse_jacobian()`

Displays the numerical inverse Jacobian matrix.

#### `display_symbolic_jacobian_dot()`

Displays the symbolic derivative of the Jacobian matrix.

#### `display_jacobian_dot()`

Displays the numerical derivative of the Jacobian matrix.

#### `display_q()`

Displays the joint angles from the inverse position kinematics.

#### `display_q_dot()`

Displays the joint velocities from the inverse velocity kinematics.

#### `display_q_double_dot()`

Displays the joint accelerations from the inverse acceleration kinematics.

#### `display_FPK()`

Displays the forward position kinematics output.

#### `display_FVK()`

Displays the forward velocity kinematics output.

#### `display_FAK()`

Displays the forward acceleration kinematics output.

### Manipulation Functions

#### `insert_EE_constants_values(symbols, values)`

Substitutes the symbolic parameters with their actual values.

- `symbols`: Array containing the symbols to be substituted.
- `values`: Array containing the corresponding values.

#### `jacobian_matrix()`

Computes the numerical Jacobian matrix.

#### `inverse_jacobian_matrix()`

Computes the numerical inverse Jacobian matrix.

#### `jacobian_derivative()`

Computes the numerical derivative of the Jacobian matrix.

#### `forward_position_kinematics(q)`

Computes the end effector position for given joint angles.

- `q`: Joint angles.

#### `inverse_kinematics_func(q0, X, TOL, MaxNoOfIterations)`

Computes the inverse position kinematics numerically using Newton Raphson.

- `q0`: Initial guess for joint angles.
- `X`: Desired end effector position.
- `TOL`: Tolerance in final output.
- `MaxNoOfIterations`: Maximum number of iterations.

#### `forward_velocity_kinematics(q, q_dot)`

Performs forward velocity kinematics.

- `q`: Current joint angles.
- `q_dot`: Current joint velocities.

#### `inverse_velocity_kinematics(q, x_dot)`

Performs inverse velocity kinematics.

- `q`: Current joint angles.
- `x_dot`: Desired end effector velocities.

#### `forward_acceleration_kinematics(q, q_dot, q_double_dot)`

Performs forward acceleration kinematics.

- `q`: Current joint angles.
- `q_dot`: Current joint velocities.
- `q_double_dot`: Current joint accelerations.

#### `inverse_acceleration_kinematics(q, q_dot, x_double_dot)`

Performs inverse acceleration kinematics.

- `q`: Current joint angles.
- `q_dot`: Current joint velocities.
- `x_double_dot`: Desired end effector accelerations.

## Example

`example_robot.m` demonstrates how to use the `robot_kinematics` class to perform various kinematic computations for a robot with two degrees of freedom (2-DOF). Please check the included `example_robot.png` to see the modeled robot in this code. The example goes through the following:

1. **Initialization**: 
    - Creates a `robot_kinematics` object with a DH table, task space size, and optional symbolic parameter flag.

2. **End Effector Position Equations**:
    - Displays symbolic and numeric end effector position equations.
    - Shows symbolic parameters present in the end effector position equation.

3. **Jacobian Matrix**:
    - Computes and displays both symbolic and numeric Jacobian matrices.

4. **Inverse Jacobian Matrix**:
    - Computes and displays both symbolic and numeric inverse Jacobian matrices.

5. **Derivative of Jacobian Matrix**:
    - Computes and displays both symbolic and numeric derivative of the Jacobian matrices.

6. **Inverse Position Kinematics**:
    - Computes joint angles using inverse kinematics to achieve a desired end effector position.
    - Validates the result by performing forward position kinematics.

7. **Inverse Velocity Kinematics**:
    - Computes joint velocities using inverse kinematics to achieve desired end effector velocities.
    - Validates the result by performing forward velocity kinematics.

8. **Inverse Acceleration Kinematics**:
    - Computes joint accelerations using inverse kinematics to achieve desired end effector accelerations.
    - Validates the result by performing forward acceleration kinematics.

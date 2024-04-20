    clc;
    syms q1 q2 l1 l2;
    DH_table = [
        q1,0,l1,0;
        q2,0,l2,0];
    robot = robot_kinematics(DH_table,2,[], true);
    
    disp("Position equations of end effector")
    disp("Symbolic")
    robot.display_EE_pose();
    disp("Symbols in end effector position equation")
    robot.display_EE_constants();
    % Substituting with values
    robot = robot.insert_EE_constants_values([l1,l2],[0.026,0.026]);
    disp("Numeric")
    robot.display_EE_pose();
    
    disp("Jacobian Matrix")
    disp("symbolic")
    robot.display_symbolic_jacobian();
    robot = robot.jacobian_matrix();
    disp("Numeric")
    robot.display_jacobian();
  
    disp("Inverse of Jacobian Matrix")
    disp("symbolic")
    robot.display_symbolic_inverse_jacobian();
    robot = robot.inverse_jacobian_matrix();
    disp("Numeric")
    robot.display_inverse_jacobian();
    
    disp("Derivative of Jacobian Matrix")
    disp("symbolic")
    robot.display_symbolic_jacobian_dot();
    robot = robot.jacobian_derivative();
    disp("Numeric")
    robot.display_jacobian_dot();

    disp("Inverse Position Kinematics to get [x,y] = [0.0162,0.0363]")
    robot = robot.inverse_kinematics_func([0.1,0.1],[0.0162,0.0363],0.0001,800);
    disp("Computed Joint Angles")
    robot.display_q();
    disp("Performing Forward Position Kinematics with computed joint angles to validate")
    robot = robot.forward_position_kinematics(transpose(robot.q));
    robot.display_FPK()
    
    disp("Inverse Velocity Kinematics to get [x_dot,y_dot] = [-1.0113,1.0151]")
    robot = robot.inverse_velocity_kinematics(transpose(robot.q),[-1.0113,1.0151]);
    disp("Computed Joint velocities")
    robot.display_q_dot();
    disp("Performing Forward Velocity Kinematics with computed joint velocities to validate")
    robot = robot.forward_velocity_kinematics(transpose(robot.q),transpose(robot.q_dot));
    robot.display_FVK();
    
    disp("Inverse Acceleration Kinematics to get [x_double_dot,y_double_dot] = [-53.7908,-35.5217]")
    robot = robot.inverse_acceleration_kinematics(transpose(robot.q),transpose(robot.q_dot),[-53.7908,-35.5217]);
    disp("Computed Joint accelerations")
    robot.display_q_double_dot();
    disp("Performing Forward Acceleration Kinematics with computed joint accelerations to validate")
    robot = robot.forward_acceleration_kinematics(transpose(robot.q),transpose(robot.q_dot),transpose(robot.q_double_dot));
    robot.display_FAK();


classdef robot_kinematics
    properties
        DH_table
        task_space_size
        EE_pose % This contains the current position of the end effector
        FPK % The forward position kinematics output
        FVK  % The forward velocity kinematics output
        FAK % The forward acceleration kinematics output

        q % Contains the joint angles from the invrese position kinematics
        q_dot % contains the joint velocities from the inverse velocity kinematics
        q_double_dot % Contains the joint accelerations from the invese acceleration kinematics

    end

    properties (Access = private)
        
        jacobian_mat % Contains the numerical jacobian matrix
        jacobian_flag % Flag to make sure the numerical jacobian has been computed
        jacobian_symbolic % Contains the symbolic jacobian matrix

        inverse_jacobian % Contains the numerical inverse jacobian matrix
        inverse_jacobian_flag % Flag to make sure the numerical inverse jacobian has been computed
        inverse_jacobian_symbolic % Contains the symbolic inverse jacobian matrix

        jacobian_dot % Contains the numerical jacobian_dot matrix
        jacobian_dot_flag % Flag to make sure the numberical jacobian dot has been computed
        jacobian_dot_symbolic % Contains the symbolic jacobian_dot matrix
        
        f_dot_inv_func % A function stored in order to improve the computational time required for computing the inverse position kinematics

    end

    methods
%         DH_table: A matrix representing the DH table for the robot
%         task_space_size: The size of the task space
%         rigid_transform: This is a matrix incase a rigid transformation is needed
%         for the end effector to move it to another place. This is
%         beneficial when the end effector needs to be moved rigidly along
%         an axis not depicted in the DH table. Incase this is not needed,
%         pass []
%         symbolic_params: This is a boolean static whether or not the user
%         is interesed in the symbolic analysis. If set as false, the
%         symbolic matrices are not gonna be computed
        function obj = robot_kinematics(DH_table,task_space_size,rigid_transform, symbolic_params)
            obj.DH_table = DH_table;
            obj.task_space_size = task_space_size;

            obj.jacobian_flag = 0;
            obj.jacobian_dot_flag = 0;
            obj.inverse_jacobian_flag = 0;

            obj.EE_pose =obj.forward_kinematics_func(rigid_transform);
            if symbolic_params
                obj.jacobian_symbolic = obj.jacobian_matrix_symbolic();
                obj.inverse_jacobian_symbolic = obj.inverse_jacobian_matrix_symbolic();
                obj.jacobian_dot_symbolic = obj.jacobian_derivative_symbolic();
            end

        end
        
%        This is a function that prints out the symbolic parameters in the
%        end effector position equations. This comes in handy incase it is
%        desired to substitute with the values for the parameters and
%        needed to display what symbolic parameters are there. This is
%        helpful to know what symbolic parameters need to be substituted in
%        with their actual values in order to perform inverse kinematics
       function display_EE_constants(obj)
           disp(symvar(obj.EE_pose))
       end
       
%        This is a mathod that displays the position of the end effector
%        incase parameter values have been substituted in. Incase
%        everything was left symbolic, it will display the equation for the
%        position of the end effector in the order of x, y, then z ( incase
%        Z was in the task space)
        function display_EE_pose(obj)
            disp(obj.EE_pose)
        end

        function display_symbolic_jacobian(obj)
            disp(obj.jacobian_symbolic)
        end

        function display_jacobian(obj)
            disp(obj.jacobian_mat);
        end

        function display_symbolic_inverse_jacobian(obj)
            disp(obj.inverse_jacobian_symbolic);
        end

        function display_inverse_jacobian(obj)
            disp(obj.inverse_jacobian);
        end

        function display_symbolic_jacobian_dot(obj)
            disp(obj.jacobian_dot_symbolic);
        end

        function display_jacobian_dot(obj)
            disp(obj.jacobian_dot);
        end

%         Displays the needed joint angles from the inverse position
%         kinematics
        function display_q(obj)
            disp(obj.q);
        end

%         Displays the needed joint_dot angles from the inverse velocity
%         kinematics
        function display_q_dot(obj)
            disp(obj.q_dot);
        end

%         Displays the needed joint_double_dot angles from the inverse
%         acceleration kinematics
        function display_q_double_dot(obj)
            disp(obj.q_double_dot);
        end

        function display_FPK(obj)
            disp(obj.FPK);
        end

        function display_FVK(obj)
            disp(obj.FVK);
        end

        function display_FAK(obj)
            disp(obj.FAK);
        end

%         This is a function that substitues the symbolic parameters with
%         their acutal values. symbols is an array containing the symbols
%         that it is desired to substitute them with their actual value
%         while values is an array containing the values for said symbols.
%         Please note that the first entry of the values list corrsponds to
%         the value of the first entry in the symbols list.
       function obj = insert_EE_constants_values(obj,symbols, values)
           obj.EE_pose = subs(obj.EE_pose,symbols,values);
           obj.jacobian_symbolic = obj.jacobian_matrix_symbolic();

       end

%         Computes the jacobian matrix. your forward position kinematics
%         must not contain any symbols other than the joint angles.
        function obj = jacobian_matrix(obj)
            if obj.jacobian_flag == 0
                disp("Make sure the only symbols in the EE_pose are joint symbols");
                obj.jacobian_mat = jacobian(obj.EE_pose,symvar(obj.EE_pose));
                obj.jacobian_flag = 1;
            end
        end

%         Computes the invrese jacobian matrix. your forward position kinematics
%         must not contain any symbols other than the joint angles.
        function obj = inverse_jacobian_matrix(obj)
            if obj.inverse_jacobian_flag == 0
    
                if obj.jacobian_flag ==0
                    obj = obj.jacobian_matrix();
                else
%                     This else was to make sure we dont print the same
%                     warning message twice. once here and another while
%                     computing the jacobian_matrix
                    disp("Make sure the only symbols in the EE_pose are joint symbols");
                end
                obj.inverse_jacobian_flag = 1;

                obj.inverse_jacobian = pinv(obj.jacobian_mat);

                q_vars = symvar(obj.inverse_jacobian);
                % This is needed to make the newton rhapson calculations
                % for inverse positon kinematics more efficient
                obj.f_dot_inv_func = matlabFunction(obj.inverse_jacobian,'Vars',{q_vars});
            end
        end

%         Computes the derivative jacobian matrix. your forward position kinematics
%         must not contain any symbols other than the joint angles.
        function obj = jacobian_derivative(obj)

            if obj.jacobian_dot_flag == 0
    
                if obj.jacobian_flag ==0
                    obj = obj.jacobian_matrix();
                else
                    disp("Make sure the only symbols in the EE_pose are joint symbols");
                end
                obj.jacobian_dot_flag = 1;
                
                % Now we are ready to compute the derivative of the Jacobian. What happens
                % is that we get the jacobian, and for each element, we get the jacobian of
                % that element and then multiply it by the q_dot matrix. The result of that
                % is a single entry in our J' matrix.
                q = symvar(obj.EE_pose);
                q_dot = sym([]);
                for i = 1:length(q)
                    q_dot = [q_dot; sym(sprintf('%s_dot', q(i)))];
                end
                obj.jacobian_dot = sym(zeros(size(obj.jacobian_mat)));
                    for i = 1: length(obj.jacobian_mat(:,1))
                        for j = 1: length(obj.jacobian_mat(1,:))
                            obj.jacobian_dot(i,j) = jacobian(obj.jacobian_mat(i,j),q)* reshape(q_dot,[],1);
                        end
                    end
            end
        end
       

%        Computes the end effector position for a certain joint angles
        function obj = forward_position_kinematics(obj,q)
           obj.FPK = double(subs(obj.EE_pose,symvar(obj.EE_pose),q));
        end

%         Computes the inverse position kinematics numerically. q0: initial
%         guess. X: desired end effector position. TOL: allowable tolerance
%         in final output. MaxNoOfIterations: Maximum iteration before
%         termination
        function obj = inverse_kinematics_func(obj,q0,X,TOL,MaxNoOfIterations)

            %This is a function that should compute the inverse position kinematics of
            %a 3 DOF over actuated system neumerically. q_n+1 = q_n - inv(t'(n))(t(n)), where
            % t is f - x = 0. Please not that when you use this function, make sure
            % that f contains only the joints as symbols, everything else must have a
            % numerical value.
            
            %%%%% Please note the following: f, q0 and X should be a column vectors
            %%%%% (nx1)

%             disp("Make sure the only symbols in the EE_pose are joint symbols and make sure to manually initialize the inverse jacobian matrix");
            
            % computing t
            t = obj.EE_pose-reshape(X,[],1);
            
            q_raw = obj.newton_raphson(t,reshape(q0,[],1),TOL,MaxNoOfIterations);
            
            %we now need to normalize the output of the neumerical solution so that it
            %is between [0,2*pi]
            obj.q = wrapTo2Pi(q_raw);

            disp("To see result, please use 'name_of_obj'.display_q(); or access the q parameter directly");
        end

%         Performs the foward velocity kinematics given the current joint
%         angles (q) and joint velocities (q_dot)
        function obj = forward_velocity_kinematics(obj,q,q_dot)

            %this is a function that will compute the forward velocity kinematics of
            %the robot
            % V = J*q'
            

            if obj.jacobian_flag ==0
                obj = obj.jacobian_matrix();
            else
                disp("Make sure the only symbols in the EE_pose are joint symbols");
            end
            temp = obj.jacobian_mat*reshape(q_dot,[],1);
            obj.FVK = double(subs(temp,symvar(temp),q));
        end
        
%         Performs inverse velocity kinematics given current joint angles
%         (q) and desired end effector velocities (x_dot)
        function obj = inverse_velocity_kinematics(obj,q,x_dot)

            %This is a function to compute the inverse velocity kinematics of a robot
            %using q' = inv(J) * X'.

            if obj.inverse_jacobian_flag ==0
                obj = obj.inverse_jacobian_matrix();
            else
                disp("Make sure the only symbols in the EE_pose are joint symbols");
            end
            
            temp = obj.inverse_jacobian * reshape(x_dot,[],1);
            obj.q_dot = double(subs(temp,symvar(temp),q));
            disp("To see result, please use 'name_of_obj'.display_q_dot(); or access the q_dot parameter directly");
        end

%         Performs forward acceleration kinematics given the current joint
%         angles (q), joint velocities (q_dot), and joint accelerations
%         (q_double_dot).
        function obj = forward_acceleration_kinematics(obj,q,q_dot,q_double_dot)

            if obj.jacobian_dot_flag ==0
                obj = obj.jacobian_derivative();
            else
                disp("Make sure the only symbols in the EE_pose are joint symbols");
            end
            
            % A_F = J*q'' + J'*q'
            temp = obj.jacobian_mat*reshape(q_double_dot,[],1) + obj.jacobian_dot*reshape(q_dot,[],1);
            mixed_vars = [];
            for i = 1:length(q)
                mixed_vars = [mixed_vars,q(i),q_dot(i)];
            end
            obj.FAK = double(subs(temp,symvar(temp),mixed_vars));
        end

%         Performs the inverse acceleration kinematics given the current
%         joint angles (q), joint velocities (q_dot_, and desired end
%         effector velocity (x_double_dot)
        function obj = inverse_acceleration_kinematics(obj,q,q_dot,x_double_dot)
            %this is a function that will compute the inverse acceleration kinematics

            % q'' = inv(J)*(X'' - J'*q')

            if obj.inverse_jacobian_flag == 0
                obj = obj.inverse_jacobian_matrix();
            end
            if obj.jacobian_dot_flag == 0
                obj = obj.jacobian_derivative();
            else
                disp("Make sure the only symbols in the EE_pose are joint symbols");
            end
           
            temp = obj.inverse_jacobian*(reshape(x_double_dot,[],1)-(obj.jacobian_dot*reshape(q_dot,[],1)));
            mixed_vars = [];
            symvar(temp)
            for i = 1:length(q)
                mixed_vars = [mixed_vars,q(i),q_dot(i)];
            end
%             mixed_vars = [mixed_vars, x_double_dot]
            obj.q_double_dot = double(subs(temp,symvar(temp),mixed_vars));
            disp("To see result, please use 'name_of_obj'.display_q_double_dot(); or access the q_double_dot parameter directly");

        end
    
    end

    methods (Access = private)

%         A method to evaluate the jacobian matrix incase the symbolic
%         parameters were substituted with their actual values.
%         function obj = revaluate(obj)
%            obj.jacobian_symbolic = obj.jacobian_matrix_symbolic();
%            
%         end

%         This is a function that computes the transformation matrix for
%         each row of the DH table
        function T = transformation_func(obj,theta,d,a,alpha)

            %defines the transformation matrix (i-1)T_(i)
    
            T = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
                     sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
                     0, sin(alpha), cos(alpha), d;
                     0, 0, 0, 1];
        end

%         This is a function that computes the forward position kinematics
%         of the robot. It computes the transformation matrices from the DH
%         table for each link then multiplies them together to get the
%         total transformation matrix relating the end effector to the
%         first frame (the frame depicted in the first row of the DH
%         table). Additionally, the function can then perform a
%         multiplication with a pre-defined rigid transformation incase
%         then end effector modeled in the DH actually needs to be moved to
%         another position using a rigid transformation. This is useful
%         incase the end effector needed to be moved along a third axes not
%         modeled by the DH table. Finally, the fourth column is extracted
%         containing the position information of the end effector.
        function X = forward_kinematics_func(obj,rigid_transform)

            transformation_matrices = {};
            
            for i= 1:size(obj.DH_table,1)
                transformation_matrices{end+1} = obj.transformation_func(obj.DH_table(i,1),obj.DH_table(i,2),obj.DH_table(i,3),obj.DH_table(i,4));
            end

            final_transformation_matrix = eye(4);
            for i=1:1:numel(transformation_matrices)
                final_transformation_matrix = final_transformation_matrix * transformation_matrices{i};
            end

            if ~isempty(rigid_transform)
                final_transformation_matrix= final_transformation_matrix * rigid_transform;
            end
            
            fourth_column = final_transformation_matrix(:,4);
            X = fourth_column(1:end-(1+(3-obj.task_space_size)));
        
        end
        
        function J = jacobian_matrix_symbolic(obj)

            %This will compute the jacobian while keeping everything symbolic,
            %including the lengths of the robot. The only constraint is that no other
            %symbol can start with q other than the robot joints and all the robot
            %joints must start with q.
            
            all_variables=symvar(obj.EE_pose);
            joints_variables = [];
                for i = 1:length(all_variables)
                    if startsWith(char(all_variables(i)), 'q')
                        joints_variables = [joints_variables, all_variables(i)];
                    end
                end
            J = jacobian(obj.EE_pose,joints_variables);

        end

%         Computes the inverse jacobian symbolic matrix
        function J_inv = inverse_jacobian_matrix_symbolic(obj)
            J_inv = pinv(obj.jacobian_symbolic);

        end

%         Computes the derivative jacobian symbolic matrix
        function J_dot = jacobian_derivative_symbolic(obj)

            %This will compute the jacobian while keeping everything symbolic,
            %including the lengths of the robot. The only constraint is that no other
            %symbol can start with q other than the robot joints and all the robot
            %joints must start with q.
            
            % I am extracting the symbols from the input J matrix excluding the
            % lengths. This means I am extracting the joint symbols (the qs)
            
            all_variables=symvar(obj.jacobian_symbolic);

            joints_variables = [];
                for i = 1:length(all_variables)
                    if startsWith(char(all_variables(i)), 'q')
                        joints_variables = [joints_variables, all_variables(i)];
                    end
                end
            
            % Now I am creating an Nx1 column matrix containing q_dots. This is needed
            % for the chain rule
            joints_dots = sym([]);
                for i = 1:length(joints_variables)
                    joints_dots = [joints_dots; sym(sprintf('%s_dot', joints_variables(i)))];
                end
             joints_dots = reshape(joints_dots,[],1);
             
            
            % Now we are ready to compute the derivative of the Jacobian. What happens
            % is that we get the jacobian, and for each element, we get the jacobian of
            % that element and then multiply it by the q_dot matrix. The result of that
            % is a single entry in our J' matrix.
            
            J_dot = sym(zeros(size(obj.jacobian_symbolic)));
                for i = 1: length(obj.jacobian_symbolic(:,1))
                    for j = 1: length(obj.jacobian_symbolic(1,:))
                        J_dot(i,j) = jacobian(obj.jacobian_symbolic(i,j),joints_variables)* joints_dots;
                    end
                end
        end

%         Function that performs the newton_raphson function needed for
%         computing the inverse position kinematics

        function q = newton_raphson(obj,f,q0,TOL,MaxNoOfIterations)

                %This is a function that implements newton raphson method
                % q_n+1 = q_n - inv(f'(n))(f(n))
                i = 1;
                
                q_vars = symvar(f);
                f_func = matlabFunction(f, 'Vars', {q_vars});
                while i <= MaxNoOfIterations
                    q = double(q0 - obj.f_dot_inv_func(transpose(q0)) * f_func(transpose(q0)));
                    %doing the tolerance check bit
                    tolerance_check = double(f_func(transpose(q)));
                    tolerance_check = abs(tolerance_check);
                    if all(tolerance_check < TOL)
                        return;
                    end
                    i = i + 1;
                    q0 = q;
                    
                end
                disp("I have failed you master. This is the best I could do");
        end
        
    end
end
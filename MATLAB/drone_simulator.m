figure;
hold on;
grid on;
view(3);
axis equal;
xlim([-2 2]); ylim([-2 2]); zlim([0 4]);
xlabel('X'); ylabel('Y'); zlabel('Z');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CONSTANTS AND INITIAL SETPOINTS/ATTITUDE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G = -9.81; % GRAVITATIONAL ACCELERATION

% set the initial position and rotation variables
% positions in the world
pos_x = 0;
pos_y = 0;
pos_z = 1;
% attitude
roll = 0;
pitch = 0;
yaw = 0;
% relative velocity
vel_x = 0;
vel_y = 0;
vel_z = 0;
% angular velocity
omega_roll = 0;
omega_pitch = 0;
omega_yaw = 0;

% throttle values
throttle_FR = 0;
throttle_FL = 0;
throttle_RL = 0;
throttle_RR = 0; % throttle vals are 0-255

THROTTLE_TO_THRUST_COEFFICIENT = 0.001; % multiply by this value to convert RPM to motor thrust force (N)
DRONE_MASS = 1.0; % (in kg)

drone_x_direction_initial = [1, 0, 0]; % pointing along the x axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIALIZE DRONE BODY IN THE SIMULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define the motor positions relative to the center of mass
motor_distance = 0.4; % distance from center to each motor
motor_positions_initial = [
    motor_distance/sqrt(2), motor_distance/sqrt(2), 0;    % FR motor
    -motor_distance/sqrt(2), motor_distance/sqrt(2), 0;   % FL motor
    -motor_distance/sqrt(2), -motor_distance/sqrt(2), 0;  % RL motor
    motor_distance/sqrt(2), -motor_distance/sqrt(2), 0    % RR motor
];

% create initial plot objects
drone_center = plot3(pos_x, pos_y, pos_z, 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');

% create the motor plot objects (blue circles)
motor1 = plot3(motor_positions_initial(1,1), motor_positions_initial(1,2), pos_z, 'go', 'MarkerSize', 3);
motor2 = plot3(motor_positions_initial(2,1), motor_positions_initial(2,2), pos_z, 'go', 'MarkerSize', 3);
motor3 = plot3(motor_positions_initial(3,1), motor_positions_initial(3,2), pos_z, 'go', 'MarkerSize', 3);
motor4 = plot3(motor_positions_initial(4,1), motor_positions_initial(4,2), pos_z, 'go', 'MarkerSize', 3);

arrow_length = 0.4;
front_arrow = quiver3(pos_x, pos_y, pos_z, arrow_length, 0, 0, 0, 'LineWidth', 2, 'Color', 'r', 'MaxHeadSize', 0.5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SIMULATION LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
iteration_counter = 0;
sim_running = true;
previous_time = 0;
current_time = 0;

while sim_running == true
    if iteration_counter == 0
    tic;
    end
    delta_time = toc;
    tic;

    % calculate quaternion from current euler angles
    q_roll = quaternion(cos(roll / 2), sin(roll / 2), 0, 0);
    q_pitch = quaternion(cos(pitch / 2), 0, sin(pitch / 2), 0);
    q_yaw = quaternion(cos(yaw / 2), 0, 0, sin(yaw / 2));
    q_total = q_yaw * q_pitch * q_roll;
    
    % apply the rotation to direction vectors
    drone_front_direction = rotatepoint(q_total, drone_x_direction_initial);
    
    % apply the rotation to motor positions
    motor_positions_rotated = rotatepoint(q_total, motor_positions_initial);
    
    % update the center of mass
    set(drone_center, 'XData', pos_x, 'YData', pos_y, 'ZData', pos_z);
    
    % update motors with rotated positions
    set(motor1, 'XData', pos_x + motor_positions_rotated(1,1), 'YData', pos_y + motor_positions_rotated(1,2), 'ZData', pos_z + motor_positions_rotated(1,3));
    set(motor2, 'XData', pos_x + motor_positions_rotated(2,1), 'YData', pos_y + motor_positions_rotated(2,2), 'ZData', pos_z + motor_positions_rotated(2,3));
    set(motor3, 'XData', pos_x + motor_positions_rotated(3,1), 'YData', pos_y + motor_positions_rotated(3,2), 'ZData', pos_z + motor_positions_rotated(3,3));
    set(motor4, 'XData', pos_x + motor_positions_rotated(4,1), 'YData', pos_y + motor_positions_rotated(4,2), 'ZData', pos_z + motor_positions_rotated(4,3));
    
    % update arrows
    set(front_arrow, 'XData', pos_x, 'YData', pos_y, 'ZData', pos_z, 'UData', drone_front_direction(1)*arrow_length, 'VData', drone_front_direction(2)*arrow_length, 'WData', drone_front_direction(3)*arrow_length);

    % update drone velocity based on gravitational acceleration
    vel_z = vel_z + G * delta_time;

    pos_x = pos_x + vel_x * delta_time;
    pos_y = pos_y + vel_y * delta_time;
    if pos_z > 0
        pos_z = pos_z + vel_z * delta_time;
    else
        pos_z = 0; % keeps it from going below the datum
    end


    previous_time = current_time;
    
    drawnow;
    
    iteration_counter = iteration_counter + 1
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
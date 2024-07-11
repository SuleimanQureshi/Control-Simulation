%% 
% 
% 
% Defining without convention (Tractor length,width , trailer length,width) 
% 
% Following Convention Lw0, Lh0, Lw1, Lh1, Lw2, Lh2
clear
% These values are yet to be measured/ corrected
global tractor_length;
global tractor_width;
global trailer_length;
global trailer_width;
global Lw0;
global Lh0;
global Lw1;
global Lh1;
global Lw2;
global Lh2;
tractor_length = 0.5;
tractor_width = 0.3;
trailer_length = 0.5;
trailer_width = 0.3;
Lw0 = 0.42;
Lh0 = 0.4;
Lw1 = 0.4;
Lh1 = 0.4;
Lw2 = 0.4;
Lh2 = 0.4;
%% 
% Defining Initial Conditions (position and vel of carts)

x_0_initial = 0;
y_0_initial = 0;
theta_0_initial = 0;
theta_1_initial = 0;
theta_2_initial = 0;
v_1_initial = 0;
v_2_initial = 0;



%% 
% Defining the Trajectory (with control inputs, linear velocity and steering 
% angle of front wheels) 
% 
% So, if we input these as the linear vel and steering angle, we will see how 
% the forward kinematics take us

global dt;
dt = 0.05;
t_start = 0;
t_stop = 15;
T = t_start:dt:t_stop;



%Reference Trajectory
global x_ref;
global y_ref;
radius = 6;
% x_ref = radius*cos(2*pi*T/t_stop-pi/2) ;
% y_ref = radius*sin(2*pi*T/t_stop-pi/2) + 6;

waypoints = [            
            5 1;
            15 5;
            5 1;
            12 10;
            8 11;
            1 10;
            ];

% waypoints = [randi(14) randi(14); randi(14) randi(14); randi(14) randi(14); randi(14) randi(14); randi(14) randi(14);];

X = [
    zeros(1,length(T)); % x_0 
    zeros(1,length(T)); % y_0
    zeros(1,length(T)); % theta_0
    zeros(1,length(T)); % theta_1
    zeros(1,length(T)); % theta_2
    zeros(1,length(T)); % v_1
    zeros(1,length(T)); % v_2
    ];
X(1,1) = x_0_initial;
X(2,1) = y_0_initial;
X(3,1) = theta_0_initial;
X(4,1) = theta_1_initial;
X(5,1) = theta_2_initial;
X(6,1) = v_1_initial;
X(7,1) = v_2_initial;


function [X_Dot, V_1_2] = transf_func(X_prev,u)
    global tractor_length;
    global tractor_width;
    global trailer_length;
    global trailer_width;
    global Lw0;
    global Lh0;
    global Lw1;
    global Lh1;
    global Lw2;
    global Lh2;
    global dt;

    lineVel = u(1);
    steerAngle = u(2);
    x_0_prev = X_prev(1);
    y_0_prev = X_prev(2);
    theta_0_prev = X_prev(3);
    theta_1_prev = X_prev(4);
    theta_2_prev = X_prev(5);
    v_1_prev = X_prev(6);
    v_2_prev = X_prev(7);

    theta_0_dot = lineVel*tan(steerAngle)/Lw0;
    theta_0 = theta_0_prev + dt*theta_0_dot;
    x_0_dot = lineVel*cos(theta_0);
    y_0_dot = lineVel*sin(theta_0);

    theta_1_dot = (lineVel*sin(theta_0_prev-theta_1_prev)/Lw1 - Lh0*cos(theta_0_prev-theta_1_prev)*((theta_0-theta_0_prev))/dt/Lw1);
    theta_1 = theta_1_prev + dt*theta_1_dot;
    v_1 = lineVel*cos(theta_0-theta_1)+Lh0*sin(theta_0-theta_1)*(theta_0-theta_0_prev)/dt;
    theta_2_dot = (v_1*sin(theta_1_prev-theta_2_prev)/Lw2 - Lh1*cos(theta_1_prev-theta_2_prev)*((theta_1-theta_1_prev))/dt/Lw2);
    theta_2 = theta_2_prev + dt*theta_2_dot;
    v_2 = v_1*cos(theta_1-theta_2)+Lh0*sin(theta_1-theta_2)*(theta_1-theta_1_prev)/dt;


    X_Dot = [
      x_0_dot;
      y_0_dot;
      theta_0_dot;
      theta_1_dot;
      theta_2_dot;
    ];

    V_1_2 = [
    v_1;
    v_2;
    ];
end

global test_int;
test_int = zeros(2,length(T));
global testing_desired_angle;

global angle_error;
angle_error = zeros(1,length(T));

global distance_error;
angle_error = zeros(1,length(T));

global another_test;
another_test = zeros(1,length(T));

global another_test2;
another_test2 = zeros(1,length(T));


testing_desired_angle = zeros(2,length(T));

function [x] = manual_mod(a,b)
    x = a-fix(a/b)*b;
end

function  [v_path, phi_path, e_prev, int_val,current_waypoint] = PID(X_prev,t,int_val,k_gains,epsilon,e_prev,waypoints,current_waypoint,prev_vel,prev_phi)
    global x_ref
    global y_ref
    global dt
    global Lw0
    global testing_desired_angle;
    global test_int;   
    global distance_error;
    global angle_error;
    global another_test2;


    % NOTE: these aren't real values like m/s^2 etc, these are just
    % temporary values for PoC, these will be experimentally gathered and
    % implemented then and we will also have to take care of sampleTime dt
    v_limit = 5; %velocity limit
    phi_limit = pi/3; %steering angle limit
    a_limit = 0.4; %acceleration limit 
    deceleration_limit = 1; %deacceleration limit
    ang_vel_limit = 0.4; %angular velocity limit
    
    e_x = waypoints(current_waypoint,1)-X_prev(1);
    e_y = waypoints(current_waypoint,2)-X_prev(2);

    psi = (atan2(e_y,e_x)) - X_prev(3);
    % psi = manual_mod(psi,2*pi);
    psi = wrapToPi(psi);
    testing_desired_angle(1,t) =  psi;
    row = sqrt(e_x^2 + e_y^2);
    distance_error(t) = row;

    %changing waypoints
    wp_dist = 0.5; % distance to change waypoints
    if (row <= wp_dist)
        if (current_waypoint ~= length(waypoints))
            current_waypoint=current_waypoint + 1;
        else
            v_path = 0;
            phi_path = 0;
            return
        end
        % x_ref_dot = (waypoints(current_waypoint-1,1)-waypoints(current_waypoint,1))/dt;
        % y_ref_dot = (waypoints(current_waypoint-1,2)-waypoints(current_waypoint,2))/dt;
        x_ref_dot = 0;
        y_ref_dot = 0;
        %recalculate everything
        e_x = waypoints(current_waypoint,1)-X_prev(1);
        e_y = waypoints(current_waypoint,2)-X_prev(2);
    
        psi = (atan2(e_y,e_x)) - X_prev(3);
        % psi = manual_mod(psi,2*pi);
        psi = wrapToPi(psi);

        testing_desired_angle(1,t) =  psi;

        row = sqrt(e_x^2 + e_y^2);
    else
        x_ref_dot = 0;
        y_ref_dot = 0;
    end
    
    angle_error(t) = psi;

    psi_int = int_val(1) + dt*psi;
    row_int = int_val(2) + dt*row;
    int_val = [psi_int row_int];
    
    if (abs(psi) < epsilon)
        v_path = (k_gains(3)*row+k_gains(4)*row_int)/cos(psi) + (e_x*x_ref_dot+e_y*y_ref_dot)/(row*cos(psi));
        % v_path = k_gains(3)*row+k_gains(4)*row_int;
    else
        disp("here")
        another_test2(t) = 5;
        v_path = 2;
        phi_path = (psi)/abs(psi)*phi_limit;
        % x = (psi);
        % phi_path = (wrapToPi(x))/abs(wrapToPi(x))*phi_limit;

        if (abs(phi_path) > phi_limit)
        phi_path = phi_limit*abs(phi_path)/phi_path;
        end
        
        if ((v_path-prev_vel) > a_limit)
            v_path = prev_vel + a_limit;
            
        end
    
        if ((v_path-prev_vel) < -deceleration_limit)
            v_path = prev_vel - deceleration_limit;
        end
    
        if ((phi_path-prev_phi) > ang_vel_limit)
            phi_path = prev_phi + ang_vel_limit;
            disp("123")
        end
    
        if ((phi_path-prev_phi) < -ang_vel_limit)
            phi_path = prev_phi - ang_vel_limit;
                        disp("456")

        end
    
        if (abs(phi_path) > phi_limit)
            phi_path = phi_limit*abs(phi_path)/phi_path;
                        disp("789")

        end
    
        if (abs(v_path) > v_limit)
            v_path = abs(v_path)/v_path*v_limit;
                        disp("abc")

        end
        e_prev = [e_x, e_y];
        return

    end
    tan_dev = ((atan2(e_y,e_x)) - (atan2(e_prev(2),e_prev(1))))/dt;
    % phi_path = k_gains(1)*psi+k_gains(2)*psi_int + tan_dev;
    % test_int(:,t) = [(k_gains(3)*row+k_gains(4)*row_int)/cos(psi), (e_x*x_ref_dot+e_y*y_ref_dot)/(row*cos(psi)); ];


    theta_dot_PID = k_gains(1)*psi+k_gains(2)*psi_int + tan_dev;
    % theta_dot_PID = k_gains(1)*psi+k_gains(2)*psi_int 

    phi_path = (atan2(theta_dot_PID*Lw0,v_path));
    testing_desired_angle(1,t) =  phi_path ;

    
    if (abs(phi_path) > phi_limit)
        phi_path = phi_limit*abs(phi_path)/phi_path;
    end
    
    if ((v_path-prev_vel) > a_limit)
        v_path = prev_vel + a_limit;
    end

    if ((v_path-prev_vel) < -deceleration_limit)
        v_path = prev_vel - deceleration_limit;
    end

    if ((phi_path-prev_phi) > ang_vel_limit)
        phi_path = prev_phi + ang_vel_limit;
    end

    if ((phi_path-prev_phi) < -ang_vel_limit)
        phi_path = prev_phi - ang_vel_limit;
    end

    if (abs(phi_path) > phi_limit)
        phi_path = phi_limit*abs(phi_path)/phi_path;
    end

    if (abs(v_path) > v_limit)
        v_path = abs(v_path)/v_path*v_limit;
    end

    if ((v_path) < 0)
        v_path = 0.4;
    end
    e_prev = [e_x, e_y];



    test_int(1,t) = v_path;
    testing_desired_angle(2,t) =  phi_path ;
    
end

theta_0_temp_test = zeros(1,length(T));
% Initialization of PID values
k_gains = [ 32 0.25 2.55 0.3]; % sorta tuned
epsilon = pi/2-0.01;
int_val = [0 0]; %integration value
e_prev = [0 0];
current_waypoint = 1;
v_path = 0;
phi_path = 0;
for t = 2:length(T)
    % v_path = 1;
    % phi_path = pi/10*cos(T(t-1));
    [v_path, phi_path, int_val, e_prev, current_waypoint] = PID(X(1:7,t-1), t , int_val,k_gains,epsilon,e_prev,waypoints,current_waypoint, v_path,phi_path);
    another_test(t) = phi_path;
    [X_Dot, V_1_2] = transf_func(X(1:7,t-1), [v_path,phi_path]);
    X(1:5,t) = X(1:5,t-1) + dt*X_Dot;
    X(6:7,t) = V_1_2;
end
x_0 = X(1,:);
y_0 = X(2,:);
theta_0 = X(3,:);
theta_1 = X(4,:);
theta_2 = X(5,:);
v_1 = X(6,:);
v_2 = X(7,:);


%% 
% 
% 
% Midpoint of trailers and hitching points

x_1 = x_0 -Lw1*cos(theta_1)-Lh0*cos(theta_0);
y_1 = y_0 -Lw1*sin(theta_1)-Lh0*sin(theta_0);
x_2 = x_1 -Lw2*cos(theta_2)-Lh1*cos(theta_1);
y_2 = y_1 -Lw2*sin(theta_2)-Lh1*sin(theta_1);

x_h_0 = x_0 - Lh0*cos(theta_0);
y_h_0 = y_0 - Lh0*sin(theta_0);
x_h_1 = x_1 - Lh1*cos(theta_1);
y_h_1 = y_1 - Lh1*sin(theta_1);
x_h_2 = x_2 - Lh2*cos(theta_2);
y_h_2 = y_2 - Lh2*sin(theta_2);
%% Simulation

function drawRobotsystem(x, y, theta, phi1, phi2, trailer1_x, trailer1_y, trailer2_x,trailer2_y)
    global tractor_length;
    global tractor_width;
    global trailer_length;
    global trailer_width;
    global Lw0;
    global Lh0;
    global Lw1;
    global Lh1;
    global Lw2;
    global Lh2;
    global dt;
    % 
    % trailer_length = 0.5;
    % trailer_width = 0.3;
    % tractor_length = 0.5;
    % tractor_width = 0.3;

    % Get half dimensions for robot/tractor and trailers
    half_length = tractor_length / 2;
    half_width = tractor_width / 2;
    trailer_half_length = trailer_length / 2;
    trailer_half_width = trailer_width / 2;
    
    % Define robot/tractor corners
    robot_corners = [
        -half_length, -half_width;
        half_length, -half_width;
        half_length, half_width;
        -half_length, half_width;
        -half_length, -half_width;
    ];

    % Define trailer corners
    trailer_corners = [
        -trailer_half_length, -trailer_half_width;
        trailer_half_length, -trailer_half_width;
        trailer_half_length, trailer_half_width;
        -trailer_half_length, trailer_half_width;
        -trailer_half_length, -trailer_half_width;
    ];

    % Define arrow points
    arrow_length = tractor_length / 2;
    arrow = [
        0, 0;
        arrow_length, 0;
        arrow_length * 0.8, 0.1 * tractor_width;
        arrow_length, 0;
        arrow_length * 0.8, -0.1 * tractor_width;
    ];

    % Rotation/Transform matrices for theta, phi1, and phi2
    rot_trans_robot = [cosd(theta), -sind(theta), x; sind(theta), cosd(theta), y; 0 0 1];
    rot_trans_trailer1 = [cosd(phi1), -sind(phi1), trailer1_x; sind(phi1), cosd(phi1), trailer1_y; 0 0 1];
    rot_trans_trailer2 = [cosd(phi2), -sind(phi2), trailer2_x; sind(phi2), cosd(phi2), trailer2_y; 0 0 1];

    % Apply rotation and translation to points
    robot_corners_h = [robot_corners, ones(size(robot_corners, 1), 1)]';
    arrow_h = [arrow, ones(size(arrow, 1), 1)]';
    trailer_corners_h = [trailer_corners, ones(size(trailer_corners, 1), 1)]';

    robot_corners_t = rot_trans_robot * robot_corners_h;
    arrow_t = rot_trans_robot * arrow_h;
    trailer1_corners_t = rot_trans_trailer1 * trailer_corners_h;
    trailer2_corners_t = rot_trans_trailer2 * trailer_corners_h;

    robot_corners_t = robot_corners_t(1:2, :)';
    arrow_t = arrow_t(1:2, :)';
    trailer1_corners_t = trailer1_corners_t(1:2, :)';
    trailer2_corners_t = trailer2_corners_t(1:2, :)';

    plot(robot_corners_t(:, 1), robot_corners_t(:, 2), 'blue'); hold on;
    plot(arrow_t(:, 1), arrow_t(:, 2), 'red');
    plot(trailer1_corners_t(:, 1), trailer1_corners_t(:, 2), 'blue');
    plot(trailer2_corners_t(:, 1), trailer2_corners_t(:, 2), 'blue');

    
    axis equal;
end
for i = 150:length(T)
    clf;
    hold on;
    xlim([-15 20]); ylim([-10 12]);
    grid on;
    xlabel('meters'); ylabel('meters');
    title(sprintf('Waypoint tracking Truck-trailer system using PID at t = %.2f s, (k_1 = %.2f, k_2 = %.2f, k_3 = %.2f, k_4 = %.2f)',T(i),k_gains(1),k_gains(2),k_gains(3),k_gains(4)))
    drawRobotsystem(x_0(i), y_0(i), rad2deg(theta_0(i)), rad2deg(theta_1(i)), rad2deg(theta_2(i)), x_1(i),y_1(i),x_2(i),y_2(i));
    % Trace out the path that has been followed
    plot(x_0(1,1:i),y_0(1,1:i));
    plot(waypoints(:,1),waypoints(:,2),'rx','LineWidth',2)
    L = legend('','','','','Path Followed','Waypoints');
    L.AutoUpdate = 'off';
    % Plotting out the connnections between the tractor and trailers
    plot([x_h_0(i), x_0(i)],[y_h_0(i), y_0(i)],  'g-');
    plot( [x_h_0(i), x_1(i)],[y_h_0(i), y_1(i)], 'r-');
    plot([x_h_1(i), x_1(i)],[y_h_1(i), y_1(i)],  'g-');
    plot( [x_h_1(i), x_2(i)],[y_h_1(i), y_2(i)], 'r-');
    plot([x_h_2(i), x_2(i)], [y_h_2(i), y_2(i)],  'g-');
    pause(0.0001);

end
% close all
% plot(testing_desired_angle)
% hold on
% plot(theta_0)
% xlim([0 150])
figure;
plot(T,distance_error)
hold on
plot(T,angle_error)
% title("Errors")
% legend('Distance Error', 'Angle Error');
% xlabel("Time (s)")
% ylabel("Amplitude")


plot(T,testing_desired_angle(2,:))
hold on
plot(T,testing_desired_angle(1,:))
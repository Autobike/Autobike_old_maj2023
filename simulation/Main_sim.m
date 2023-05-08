%% clear the possible remnant on previous running

set(0,'defaulttextinterpreter','none');
dbclear all;
clear;
close all;
clc;

%% Simulation Settings and Bike Parameters
% General Parameters

    % Gravitational Acceleration
        g = 9.81;
    % Name of the model
        model = 'Main_bikesim';
    % Simulation time
        sim_time = 100;

    % Sampling Time
        Ts = 0.01; 
    % First closest point selection in reference 
    % Starts at 2 because the one before closest is in the local reference as well
        ref_start_idx = 2;
    % Horizon distance [m]
        hor_dis = 10;
    % Constant Speed [m/s]
        v = 2;    

% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');
% Choose The Bike - Options: 'red' or 'black'
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
% bike model
    bike_model = 1; % 1 = non-linear model || 2 = linear model
% Run all test cases
    Run_tests = 0; % 0 = Don't run test cases & save measurementdata in CSV || 1 = run test cases || 2 = generate ref yourself

% Initial states

% Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
% Initial Steering
        initial_state.steering = deg2rad(0);
% Initial Pose (X,Y,\theta)
        initial_state.x = 0;
        initial_state.y = 0;
        initial_state.heading = deg2rad(0);
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];

%% Reference trajectory generation

% SHAPE options: sharp_turn, line, infinite, circle, ascent_sin, smooth_curve
type = 'circle';
% Distance between points
ref_dis = 1;
% Number# of reference points
N = 100; 
% Scale (only for infinite and circle)
scale = 40; 

[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%% OWN TRAJECTORY
% if Run_tests == 2
% test_trajectory();
% data = fileread('trajectory.txt');
% test_curve=[Xref,Yref,Psiref];
% Nn = size(test_curve,1); % needed for simulink
% end

%% Reference test (warnings and initialization update)
if Run_tests == 0 || Run_tests == 2

Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v, ref_dis);

%update initial states if offset is detected
initial_state.x = Output_reference_test(1);
initial_state.y = Output_reference_test(2);
initial_state.heading = Output_reference_test(3);
initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
initial_state_estimate = initial_state;
end

%% Unpacked bike_params
h = bike_params.h_mod;
lr = bike_params.lr_mod;
lf = bike_params.lf_mod; 
lambda = bike_params.lambda_mod;
c = bike_params.c_mod;
m = bike_params.m_mod;
h_imu = bike_params.IMU_height_mod;

% Correction IMU
IMU_x = bike_params.IMU_x_mod;
IMU_roll = bike_params.IMU_roll_mod;
IMU_pitch = bike_params.IMU_pitch_mod;            
IMU_yaw = bike_params.IMU_yaw_mod;

% Convert orientation offsets to radians
roll = IMU_roll * pi / 180;
pitch = IMU_pitch * pi / 180;
yaw = IMU_yaw * pi / 180;

% Calculate transformation matrix
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];   % Rotation matrix for roll
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)]; % Rotation matrix for pitch
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];        % Rotation matrix for yaw

T = Rz*Ry*Rx;                                                      

%% Disturbance Model
% 
% % Roll Reference  
% roll_ref_generation;%long time ago left by other students, it's helpless now but keep it
% 
% % Steering Rate State Perturbation
% pert_deltadot_state = 0; % Switch ON (1) / OFF (0) the perturbation
% pert_deltadot_state_fun = @(time)  -0.5*(time>10) && (ceil(mod(time/3,2)) == 1) &&(time<30);
% 
% % Roll Rate State Perturbation
% pert_phidot_state = 0; % Switch ON (1) / OFF (0) the perturbation
% pert_phidot_state_fun = @(time) cos(time)*(time>10 && time < 10.4);

%% Bike State-Space Model

% Continuous-Time Model

% % Controllable Canonical Form
%     A = [0 g/bike_params.h ; 1 0];
%     B = [1 ; 0];
%     C = [bike_params.a*v/(bike_params.b*bike_params.h) g*bike_params.inertia_front/(bike_params.h^3*bike_params.m)+v^2/(bike_params.b*bike_params.h)];
%     D = [bike_params.inertia_front/(bike_params.h^2*bike_params.m)];

% % Observable Canonical Form
%     A = [0 g/bike_params.h ; 1 0];
%     B = [g*bike_params.inertia_front/(bike_params.h^3*bike_params.m)+(v^2./(bike_params.b*bike_params.h)-bike_params.a*bike_params.c*g/(bike_params.b*bike_params.h^2)).*sin(bike_params.lambda) ;
%         bike_params.a*v/(bike_params.b*bike_params.h).*sin(bike_params.lambda)];
%     C = [0 1];
%     D = [bike_params.inertia_front/(bike_params.h^2*bike_params.m)];
% 
% % Linearized System
%     linearized_sys = ss(A,B,C,D);
% % Augmented System
%     fullstate_sys = ss(linearized_sys.A,linearized_sys.B,eye(size(linearized_sys.A)),0);
% % Discretized System
%     discretized_sys = c2d(linearized_sys,Ts);

%% Balancing Controller

% Outer loop -- Roll Tracking
P_balancing_outer = 1.3;
I_balancing_outer = 0.0;
D_balancing_outer = 0.0;

% Inner loop -- Balancing
P_balancing_inner = 3;
I_balancing_inner = 0;
D_balancing_inner = 0;  

%% The LQR controller
% error model for LQR controller calculation
A_con=[0 v;0 0];
B_con=[lr*v/(lr+lf);v/(lr+lf)];

 kk=0.000000010;
 Q=kk*[1000 0;0 100];
 R=0.5;
 [K,S,e] = lqr(A_con,B_con,Q,R);
 k1=K(1);
 k2=K(2);

e2_max=deg2rad(30);%Here is the e2_max we used to calculate e1_max
e1_max=abs(-k2*e2_max/k1);% k1,k2 has been known, so we can calculate e1_max

% e2_max=deg2rad(30);
% e1_max=abs(1000);

%% Transfer function for heading in wrap traj

% Transform tf into ss
num = 1;
den = [lr/(lr+lf), v/(lr+lf)];
[A_t, B_t, C_t, D_t] = tf2ss(num,den);

% Discretize the ss
Ad_t = (eye(size(A_t))+Ts*A_t);
Bd_t = B_t*Ts;

%% Kalman Filter

% % A matrix (linear bicycle model with constant velocity)
% % Est_States := [X Y psi phi phi_dot delta v]
A = [0 0 0 0 0 0 1;
     0 0 v 0 0 v*(lr/(lf+lr))*sin(lambda) 0;
     0 0 0 0 0 (v/(lr+lf))*sin(lambda) 0;
     0 0 0 0 1 0 0;
     0 0 0 (g/h) 0 ((v^2*h-lr*g*c)/(h^2*(lr+lf)))*sin(lambda) 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0];

% B matrix (linear bicycle model with constant velocity)
B = [0 0 0 0 ((lr*v)/(h*(lr+lf))) 1 0]';

% Including GPS
C1 = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 0 -g+((-h_imu*g)/h) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2) + (v^2)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 (v)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];

D1 = [0 0 (-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';

% Excluding GPS
C2 = [(-g+((-h_imu*g)/h)) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2)+(v^2)*sin(lambda)/(lr+lf) 0;
      0 1 0 0;
      0 0 (v)*sin(lambda)/(lr+lf) 0;
      0 0 1 0;
      0 0 0 1];

D2 = [(-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';

% Discretize the model
A_d = (eye(size(A))+Ts*A);
B_d = Ts*B;

% Q and R matrix
% Parameters of Q
Q_GPS = 0.2^2;
Q_Psi = 1^2;
Q_roll = deg2rad(0.01)^2;
Q_rollrate = deg2rad(0.05)^2;
Q_delta = deg2rad(0.05)^2;
Q_v = 0.1^2;
Qscale = 1;
Q =Qscale* [Q_GPS 0 0 0 0 0 0;
              0 Q_GPS 0 0 0 0 0;
              0 0 Q_Psi 0 0 0 0;
              0 0 0 Q_roll 0 0 0;
              0 0 0 0 Q_rollrate 0 0;
              0 0 0 0 0 Q_delta 0;
              0 0 0 0 0 0 Q_v];

% Parameters of R
R_GPS = 0.2^2;
R_ay =deg2rad(0.1)^2;
R_wx = deg2rad(0.1)^2;
R_wz = deg2rad(0.1)^2;
R_delta = 0.001^2;
R_v = 10^2;
Rscale = 1;
R =Rscale* [R_GPS 0 0 0 0 0 0;
              0 R_GPS 0 0 0 0 0;
              0 0 R_ay 0 0 0 0;
              0 0 0 R_wx 0 0 0;
              0 0 0 0 R_wz 0 0;
              0 0 0 0 0 R_delta 0;
              0 0 0 0 0 0 R_v];

% Compute Kalman Gain
    % including GPS
    [P1,Kalman_gain1,eig] = idare(A_d',C1',Q,R,[],[]);
    eig1 = abs(eig);
    Kalman_gain1 = Kalman_gain1';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag

% Polish the kalman gain (values <10-5 are set to zero)
for i = 1:size(Kalman_gain1,1)
    for j = 1:size(Kalman_gain1,2)
        if abs(Kalman_gain1(i,j)) < 10^-5
            Kalman_gain1(i,j) = 0;
        end
    end
end 

% Kalman_gain excluding GPS
Kalman_gain2 = Kalman_gain1(4:7,3:7);

%% Save matrix in XML/CSV

matrixmat = [A_d; B_d'; C1; D1';Kalman_gain1];

filename_matrix = 'matrixmat.csv'; % Specify the filename
csvwrite(filename_matrix, matrixmat); % Write the matrix to the CSV file

filename_trajectory = 'trajectorymat.csv'; % Specify the filename
csvwrite(filename_trajectory, test_curve); % Write the matrix to the CSV file

 %% Start the Simulation
if Run_tests == 0 || Run_tests == 2

tic
try Results = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

% Simulation Messages and Warnings
% if Results.stop.Data(end) == 1
%     disp('Message: End of the trajectory has been reached');
% end

% save the states for offline kalman
bikedata_simulation_bikestates = array2table([Results.bike_states.Time Results.bike_states.Data]);
filename_simulation_bikestates = 'bikedata_simulation_bikestates.csv'; % Specify the filename
bikedata_simulation_bikestates.Properties.VariableNames(1:8) = {'Time', 'X', 'Y', 'Psi', 'Roll', 'Rollrate', 'Delta', 'Velocity'};
writetable(bikedata_simulation_bikestates ,filename_simulation_bikestates);

% save measurement data and estimations for offline kalman
bikedata_simulation = array2table([Results.estimated_states.Time Results.estimated_states.Data Results.measurements.Data Results.steerrate_input.data]);
filename_simulation= 'bikedata_simulink_simulation.csv'; % Specify the filename
bikedata_simulation.Properties.VariableNames(1:16) = {'Time', 'X_estimated', 'Y_estimated', 'Psi_estimated', 'Roll_estimated', 'Rollrate_estimated', 'Delta_estimated', 'Velocity_estimated','GPS_X','GPS_Y','a_y','w_x','w_z','Delta_enc','Velocity','Steerrate'};
writetable(bikedata_simulation,filename_simulation);

%% Plotting
%name of the plot
Tnumber = 'No test case: General simulation run';
        Plot_bikesimulation_results(Tnumber, Ts, test_curve, Results);

end

%% Test cases for validation

if Run_tests == 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TEST CASE 1 Sparse infinite
disp('Test case 1: Sparse infinite')

type = 'infinite';
ref_dis = 0.5;
N = 40; 
scale = 100; 
[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%test reference
Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v,ref_dis);
initial_state.x = Output_reference_test(1);
initial_state.y = Output_reference_test(2);
initial_state.heading = Output_reference_test(3);
initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
initial_state_estimate = initial_state;

try Results1 = sim(model);
    catch error_details 
end
% Simulation Messages and Warnings
if Results1.stop.Data(end) == 1
    disp('Message: End of the trajectory has been reached');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TEST CASE 2 Large offset X:-5 Y:0 PSI: 0
disp('Test case 2: Large offset');

type = 'line';
ref_dis = 0.1;
N = 1000; 
scale = 100; 
[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%test reference
Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v,ref_dis);
initial_state.x = Output_reference_test(1);
initial_state.y = Output_reference_test(2)-5;
initial_state.heading = Output_reference_test(3)-pi/4;
initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
initial_state_estimate = initial_state;

try Results2 = sim(model);
    catch error_details 
end
% Simulation Messages and Warnings
if Results2.stop.Data(end) == 1
    disp('Message: End of the trajectory has been reached');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%TEST CASE 3 Small circle
disp('Test case 3: Small circle')

type = 'circle';
ref_dis = 0.5;
N = 100; 
scale = 5; 
[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%test reference
Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v,ref_dis);
initial_state.x = Output_reference_test(1);
initial_state.y = Output_reference_test(2);
initial_state.heading = Output_reference_test(3);
initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
initial_state_estimate = initial_state;

try Results3 = sim(model);
    catch error_details 
end
% Simulation Messages and Warnings
if Results3.stop.Data(end) == 1
    disp('Message: End of the trajectory has been reached');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%TEST CASE 4 Sharp turn
disp('Test case 4: Sharp turn')

type = 'sharp_turn';
ref_dis = 0.01;
N = 2100; 
scale = 10; 
[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%test reference
Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v,ref_dis);
initial_state.x = Output_reference_test(1);
initial_state.y = Output_reference_test(2);
initial_state.heading = Output_reference_test(3);
initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
initial_state_estimate = initial_state;

try Results4 = sim(model);
    catch error_details 
end

% Simulation Messages and Warnings
if Results4.stop.Data(end) == 1
    disp('Message: End of the trajectory has been reached');
end

end

%% Plotting Testcases
if Run_tests == 1
    close all
    %Test case 1
    Tnumber = 'Test case 1: Sparse infinite';

        Plot_bikesimulation_results(Tnumber, Ts, test_curve, Results1, bike_params);
    %Test case 2 
    Tnumber = 'Test case 2: Large offset';
 
        Plot_bikesimulation_results(Tnumber, Ts, test_curve, Results2, bike_params);
    %Test case 3    
    Tnumber = 'Test case 3: Small circle';
     
        Plot_bikesimulation_results(Tnumber, Ts, test_curve, Results3, bike_params);
    %Test case 4 
    Tnumber = 'Test case 4: Sharp turn';

        Plot_bikesimulation_results(Tnumber, Ts, test_curve, Results4, bike_params);

end

%% Utility Functions

function Parameters = LoadBikeParameters(bike)

    if strcmp(bike,'red')
        % Red bike
 % real parameters and positions on bike
        Parameters.inertia_front = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.r_wheel = 0.311;        % radius of the wheel
        Parameters.h = 0.2085 + Parameters.r_wheel;   % height of center of mass [m]
        Parameters.lr = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m = 45;                 % Bike mas [kg]
        Parameters.lambda = deg2rad(70);   % angle of the fork axis [deg]
        Parameters.IMU_height = 0.615;      % IMU height [m]
        Parameters.IMU_x = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
        
        % Parameters in the model
        Parameters.inertia_front_mod = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.r_wheel_mod = 0.311;        % radius of the wheel
        Parameters.h_mod = 0.2085 + Parameters.r_wheel_mod;   % height of center of mass [m]
        Parameters.lr_mod = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf_mod = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c_mod = 0.06;                % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m_mod = 45;                  % Bike mas [kg]
        Parameters.lambda_mod = deg2rad(70);    % angle of the fork axis [deg]
        Parameters.IMU_height_mod = 0.615;      % IMU height [m]
        Parameters.IMU_x_mod = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll_mod = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch_mod = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw_mod = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps_mod = 0.0;              % GPS position X accoarding to the model [m] 
        Parameters.Ygps_mod = 0.0;              % GPS position Y accoarding to the model [m]
        Parameters.Hgps_mod = 0.0;              % GPS position height accoarding to the model   [m]
        

        %
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use

    elseif strcmp(bike,'black')
        % Black bike

        % Parameters on bike (actual measured)
        Parameters.inertia_front = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.h = 0.534 ;             % height of center of mass [m]
        Parameters.lr = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m = 31.3;               % Bike mass [kg]
        Parameters.lambda = deg2rad(66);   % angle of the fork axis [deg]  !!! TO BE MEASURED
        Parameters.IMU_height = 0.215;     % IMU height [m]
        Parameters.IMU_x = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
        
        % Parameters in model 
        Parameters.inertia_front_mod = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.h_mod = 0.534 ;             % height of center of mass [m]
        Parameters.lr_mod = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf_mod = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c_mod = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m_mod = 31.3;               % Bike mass [kg]
        Parameters.lambda_mod = deg2rad(66);   % angle of the fork axis [deg]  !!! TO BE MEASURED
        Parameters.IMU_height_mod = 0.215;     % IMU height [m]
        Parameters.IMU_x_mod = 0.0;            % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll_mod = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch_mod = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw_mod = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps_mod = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps_mod = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps_mod = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
       
        %
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use
         
    end
end





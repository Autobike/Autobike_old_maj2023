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
% Sampling Time
        Ts = 0.01; 
% Constant Speed [m/s]
        v = 2;    
% Choose The Bike - Options: 'red' or 'black'
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
% Trajectory Record : 1=using measurment data GPS as trajectory
TrajectoryRecord = 0;

% Trajectory offset
offset_X = 0;
offset_Y = 0;
offset_Psi = deg2rad(0); 

% Calculate starting point in case of creating your own trajectory
long_start = deg2rad(57.68755656300765);
lat_start = deg2rad(11.978256853033995);

%% Trajectory creator
% recorded trajectory

if TrajectoryRecord == 1
data_lab = readtable('data.csv');
longitude0 = deg2rad(0);
latitude0 = deg2rad(0);
Earth_rad = 6371000.0;
X = Earth_rad * (data_lab.LongGPS_deg_ - longitude0) * cos(latitude0);
Y = Earth_rad * (data_lab.LatGPS_deg_ - latitude0);

% XY = unique([X Y],'rows');
XY = unique([X Y],'rows','stable');
n = length(XY);

psiref = atan2(XY(2:n,2)-XY(1:n-1,2),XY(2:n,1)-XY(1:n-1,1));
traj_or_rec = [XY [psiref(1);psiref]];
traj_or_rec(1:3,:) = []; 

filename_trajectory = 'trajectorymat.csv';
dlmwrite( filename_trajectory, traj_or_rec, 'delimiter', ',', 'precision', 10);

end

if TrajectoryRecord == 2
longitude0 = deg2rad(0);
latitude0 = deg2rad(0);
Earth_rad = 6371000.0;
X = Earth_rad * (long_start - longitude0) * cos(latitude0);
Y = Earth_rad * (lat_start - latitude0);
TrajectoryCreator()
disp('X start = ') 
disp(X)
disp('Y start = ')
disp(Y)
disp('Create your trajectory and press a key to continue !')
pause
end

% Apply offset trajectory
trajectory = readtable('trajectorymat.csv');
traj_or = table2array(trajectory);
traj_or = traj_or';

n = length(traj_or(1,:));

psiref = atan2(traj_or(2,2:n)-traj_or(2,1:n-1),traj_or(1,2:n)-traj_or(1,1:n-1));
traj_or = [traj_or(1,:); traj_or(2,:); psiref(1) psiref]';

traj(:,1) = traj_or(:,1) + offset_X;
traj(:,2) = traj_or(:,2) + offset_Y;
traj(:,3) = traj_or(:,3) + offset_Psi;
GPSXY = traj(:,1:2)-traj(2,1:2);
RotGPS = [cos(offset_Psi) sin(offset_Psi); -sin(offset_Psi) cos(offset_Psi)];
traj(:,1:2) = (GPSXY*RotGPS)+traj(2,1:2);

filename_trajectory = 'trajectorymat.csv'; % Specify the filename
dlmwrite( filename_trajectory, traj, 'delimiter', ',', 'precision', 10);

figure()
plot(traj_or(2:end,1),traj_or(2:end,2))
hold on
plot(traj(2:end,1),traj(2:end,2))
xlabel('X position (m)')
ylabel('Y position (m)')
axis equal
grid on
legend('Original trajectory', 'adjusted trajectory')
title('Trajectory')

%% Initial states
% Initial states
initial_X = traj(2,1);
initial_Y = traj(2,2);
initial_Psi = traj(2,3);
initial_roll = deg2rad(0);
initial_roll_rate = deg2rad(0);
initial_delta = deg2rad(0);
initial_v = 0;

initial_states = [initial_X,initial_Y,initial_Psi, initial_roll, initial_roll_rate, initial_delta, initial_v];

%% Unpacked bike_params
h = bike_params.h;
lr = bike_params.lr;
lf = bike_params.lf; 
lambda = bike_params.lambda;
c = bike_params.c;
m = bike_params.m;
h_imu = bike_params.IMU_height; 
r_wheel = bike_params.r_wheel; 



%% Balancing Controller

% Outer loop -- Roll Tracking
P_balancing_outer = 1.75;
I_balancing_outer = 0.0;
D_balancing_outer = 0.0;

% Inner loop -- Balancing
P_balancing_inner = 1;
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

%% Transfer function for heading in wrap traj

% Transform tf into ss
num = 1;
den = [lr/(lr+lf), v/(lr+lf)];
[A_t, B_t, C_t, D_t] = tf2ss(num,den);

% Discretize the ss
Ad_t = (eye(size(A_t))+Ts*A_t);
Bd_t = B_t*Ts;

%% Kalman Filter

% A matrix (linear bicycle model with constant velocity)
% Est_States := [X Y psi phi phi_dot delta v]
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
Q_GPS = 0.1;
Q_Psi = 0.1;
Q_roll = 1e-9;
Q_rollrate = 5;
Q_delta = 10;
Q_v = 0.5;
Qscale = 1;
Q =Qscale* [Q_GPS 0 0 0 0 0 0;
              0 Q_GPS 0 0 0 0 0;
              0 0 Q_Psi 0 0 0 0;
              0 0 0 Q_roll 0 0 0;
              0 0 0 0 Q_rollrate 0 0;
              0 0 0 0 0 Q_delta 0;
              0 0 0 0 0 0 Q_v];

% Parameters of R
R_GPS = 1.567682871320335;
R_ay = 0.256431376435930;
R_wx = 3.941639024088922e-12;
R_wz = 0.023363599865703;
R_delta = 4.175280633723090e-05;
R_v = 0.1;
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

matrixmat = [A_d; B_d'; C1; D1';Kalman_gain1;initial_states];

filename_matrix = 'matrixmat.csv'; % Specify the filename
dlmwrite(filename_matrix, matrixmat, 'delimiter', ',', 'precision', 10);

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
        
        %
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use
         
    end
end
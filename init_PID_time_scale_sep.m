Theta_i = [0,0,0].'; % deg
Omega_i= [15    , -15, 15].'*pi/180; %deg/s 
X_i = [Theta_i; Omega_i]; % deg -> rad
Theta_des=[30,50,70].'*pi/180;   
Omega_des=[0.2,0.2,0.2].'*pi/180;
X_des = [Theta_des; Omega_des];
I_c = 0.00042; % kg m^2
I= [2.10, 0, 0; 0, 2.3, -0.03; 0.01, -0.03, 1.72]; % kg m^2
%I= [2.10, 0, 0; 0, 2.3, 0; 0, 0, 1.72]; % kg m^2
I_inv = inv(I);
T_d = 1e-6; % Nm
p1 = pi; p2 = pi/2; p3=pi/4;


T_c_max = 0.015; % Nm
H_c_max = 0.035; % Nms
beta = 40*pi/180; % RW titlt angle
R_rw = [cos(beta), 0, -cos(beta), 0;
    0, cos(beta), 0, -cos(beta);
    sin(beta), sin(beta), sin(beta), sin(beta)];
R_rw_pinv = pinv(R_rw);

% for dynamics
orbit_alt = 600000; %m
R = earthRadius+orbit_alt;
mu=3.986e14;
n = sqrt(mu/R^3);

Kp_t=[1;1;1]*1e-2;
Kd_t= 1e-4;
Ki_t=1e-4*0;
Kp_w=1.1*[1;1 ;1]*1e-1;
Kd_w=1e-1;
Ki_w=1e-4;



K_unload=1.5*[1;1;1]*1e2;  
K_redist = [1;1;5]*1e1;
    
% test
% Theta_test = [0;0;0];
% Omega_test = [0;0;0];
% out=sim('Dynamics.slx')
% out.yout{1}.Values.Data
% 
% Theta_test = [pi;0;pi];
% Omega_test = [0;0;0];
% out=sim('Dynamics.slx')
% out.yout{1}.Values.Data
out=sim('PID_time_scale_sep.slx')
% plot(out.X.Time,out.X.Data(1,:,:),out.X.Time,out.X.Data(2,:,:),out.X.time,out.X.Data(3,:,:))
% legend('theta1','theta2','theta3')
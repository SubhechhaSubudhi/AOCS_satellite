Theta_i = [0,0,0].'; % deg
Omega_i= [15    , -15, 15].'*pi/180; %deg/s 
X_i = [Theta_i; Omega_i]; % deg -> rad
Theta_des=[30,50,70].'*pi/180;   
Omega_des=[0.2,0.2,0.2].'*pi/180*0;
X_des = [Theta_des; Omega_des];
I_c = 0.00042; % kg m^2
I= [2.10, 0, 0; 0, 2.3, -0.03; 0.01, -0.03, 1.72]; % kg m^2
I_inv = inv(I);
T_d = 1e-6; % Nm
p1 = pi; p2 = pi/2; p3=pi/4;

T_c_max = 0.015; % Nm
H_c_max = 0.035; % Nms
beta = 40*pi/180; % RW titlt angle

% R_rw = [cos(beta), 0, -cos(beta), 0;
%     0, cos(beta), 0, -cos(beta);
%     sin(beta), sin(beta), sin(beta), sin(beta)];
% R_rw_pinv = pinv(R_rw);
R_rw_pinv = 0.5*[1/cos(beta),0/cos(beta),0.5/sin(beta),0.5;
    0/cos(beta),1/cos(beta),0.5/sin(beta),-0.5;
    -1/cos(beta),0/cos(beta),0.5/sin(beta),0.5;
    0/cos(beta),-1/cos(beta),0.5/sin(beta),-0.5];
R_rw = inv(R_rw_pinv);

% for dynamics
orbit_alt = 600000; %m
R = earthRadius+orbit_alt;
mu=3.986e14;
n = sqrt(mu/R^3);

% Sensor noise
Theta_noise = 0*(0.01*pi/180)^2*[1;1;1];
Omega_noise = 0*(0.001 *pi/180)^2*[1;1;1];

% PID gains
Kp_t=[1;1;1]*1e-2;  
Kd_t= 1e-4;
Ki_t=[1;1;10]*1e-8*0;
Kp_w=1.1*[1;1 ;1]*1e-1;
Kd_w=1e-1;
Ki_w=0*1e-4;

% Gains for the momentum management system
K_unload=1.5*[1;1;1]*1e-1;  
K_redist = [1;1;5]*1e-0;
    
out=sim('PID_time_scale_sep.slx')

%%
figure(1)
plot(out.X.Time,out.X.Data(:,1),out.X.Time,out.X.Data(:,2),out.X.time,out.X.Data(:,3))
legend('theta1','theta2','theta3')
title('Attitude Euler angle')

figure(2)
plot(out.X.Time,out.X.Data(:,4),out.X.Time,out.X.Data(:,5),out.X.time,out.X.Data(:,6))
legend('omega1','omega2','omega3')
title('Angular velocity')

figure(3)
plot(out.Q.Time,squeeze(out.Q.Data(1,:,:)),out.Q.Time,squeeze(out.Q.Data(2,:,:)),out.Q.time,squeeze(out.Q.Data(3,:,:)),out.Q.time,squeeze(out.Q.Data(4,:,:)))
legend('q1','q2','q3','q4')
title('Attitude quaternion')

figure(4)
plot(out.H_rw_tsat_msat.Time,out.H_rw_tsat_msat.Data(:,1),out.H_rw_tsat_msat.Time,out.H_rw_tsat_msat.Data(:,2),out.H_rw_tsat_msat.time,out.H_rw_tsat_msat.Data(:,3),out.H_rw_tsat_msat.time,out.H_rw_tsat_msat.Data(:,4))
legend('Wheel1','Wheel2','Wheel3','Wheel4')
title('Reaction wheel momentum')

figure(5)
plot(out.T_rw_tsat_msat.Time,out.T_rw_tsat_msat.Data(:,1),out.T_rw_tsat_msat.Time,out.T_rw_tsat_msat.Data(:,2),out.T_rw_tsat_msat.time,out.T_rw_tsat_msat.Data(:,3),out.T_rw_tsat_msat.time,out.T_rw_tsat_msat.Data(:,4))
legend('Wheel1','Wheel2','Wheel3','Wheel4')
title('Reaction wheel Torque')

figure(6)
plot(out.T_c.Time,out.T_c.Data(:,1),out.T_c.Time,out.T_c.Data(:,2),out.T_c.time,out.T_c.Data(:,3))
legend('T_c1','T_c2','T_c3')
title('Control torque')

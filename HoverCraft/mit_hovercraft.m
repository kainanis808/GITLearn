%mit_hovercraft.m
%
%Script to recreate the hovercraft solutions from MIT's 2.017
%https://ocw.mit.edu/courses/mechanical-engineering/2-017j-design-of-electromechanical-robotic-systems-fall-2009/assignments/MIT2_017JF09_p29.pdf
%
%A Trimble (atrimble@hawaii.edu)
%2021.11.12

%% PREPARE WORKSPACE
clear
clc

%% INPUTS
x0 = [0 0 0 0 0 0]';
xd = [1 1 pi 0 0 0]';
xd2 = [20 20 pi/2 1 0 0]';
kp = [0.1 0.1 0.1]';
kd = [0 0.3 0.4]';

xdint = [19 15 0 0 0 0]';
dtol  = 1;
flag = 1;

M = eye(3);
Minv = inv(M);

D = [1 0 0;...
      0 0 0;...
      0 0 0];

%% SIMULATION SETUP
t_span = [0,70];
dt = 0.1;

t = t_span(1):dt:t_span(end);
N = length(t);
x = zeros(6,N);
x(:,1) = x0;

x2 = zeros(6,N);
x2(:,1) = x0;

%% CALCULATIONS
%Case 1
for i = 2:N
    eta_i = x(1:3,i-1);
    nu_i = x(4:6,i-1);
    
    e_enu_i = eta_i-xd(1:3);
    
    psi = eta_i(3);
    
    R = [cos(psi) -sin(psi) 0;...
         sin(psi)  cos(psi) 0;...
         0         0        1];
    
    e_b_i = R*e_enu_i;
    
    tau = -diag(kp)*e_b_i-diag(kd)*nu_i;
    
    nu = nu_i + dt*Minv*(tau - D*nu_i);
    eta = eta_i + dt*R*nu_i;
    
    x(:,i) = [eta;nu]; 
end

%Case 2
for i = 2:N
    %Find the current state
    eta_i = x2(1:3,i-1);
    nu_i = x2(4:6,i-1);
    %Find the current transfer function   
    psi = eta_i(3);
    R = [cos(psi) -sin(psi) 0;...
         sin(psi)  cos(psi) 0;...
         0         0        1];
    %Set current target point
    if flag == 0
        target = xd2(1:3);
    elseif flag == 1
        target = xdint(1:3);
    end
    %Find the vector from the body coordinate system to the current 
    %target point.
    eta_tf_f = target;
    eta_bf_f = eta_i;
    eta_tb_f = eta_tf_f-eta_bf_f;
    eta_tb_b = R'*eta_tb_f;
    %Set flag based on distance
    d = sqrt(eta_tb_b(1)^2+eta_tb_b(2)^2);
    if d < dtol
        flag = 0;
    end
  
    %Find the error terms for control
    e_u = eta_tb_b(1);
    beta = atan2(eta_tb_b(2),eta_tb_b(1));
    %Apply the control law
    Fu = kp(1)*e_u-kd(1)*nu_i(1);
    if Fu>1
        Fu = 1;
    elseif Fu<-1
        Fu = -1;
    end
    M = kp(3)*beta-kd(3)*nu_i(3);
    
    tau = [Fu 0 M]';
    
    nu = nu_i + dt*Minv*(tau - D*nu_i);
    eta = eta_i + dt*R*nu_i;
    
    x2(:,i) = [eta;nu]; 
end

%% OUTPUTS
figure(1); clf;

plot(x(1,:),x(2,:),'x');
xlabel('East');
ylabel('North');
axis equal;

hold on;
plot(xd(1),xd(2),'or','markersize',2);

figure(2); clf;

ylabels = {'E','N','/psi','u','v','r'};

for i = 1:6
    subplot(2,3,i);
    plot(t,x(i,:));
    xlabel('t');
    ylabel(ylabels{i});
end

figure(3); clf;

plot(x2(1,:),x2(2,:),'x');
xlabel('East');
ylabel('North');
axis equal;

hold on;
plot(xd2(1),xd2(2),'or','markersize',2);

figure(2); clf;

ylabels = {'E','N','/psi','u','v','r'};

for i = 1:6
    subplot(2,3,i);
    plot(t,x2(i,:));
    xlabel('t');
    ylabel(ylabels{i});
end

function [t,boomState,alpha,Mx,My,Mz,f] = boomerangTrajectory(S,B,tmax)

%% Controllable Paramaters
%Throwing in x-direction
V0 = S.V0; %initial speed magnitude(m/s)
Z0 = S.Z0; %(m)
w0 = S.w0; %(rad/s)

thetaHor0 = S.thetaHor0; %(rad)
thetaLay0 = S.thetaLay0; %(rad)
alpha0 = S.alpha0; %(rad)

dt = S.dt; %(s)
tmax = S.tmax; %(s)
%% Boomerang Parameters
m = B.m; %mass (kg)
R = B.R; %length of blade (m)
Roffset = B.Roffset; %Distance from center of lift to axis of symettry of blade(m)
C = B.C; %chord of blade (m)
I = B.I; %Inertia Tensor(kgm^2)
A = B.A; %area of boomn (m^2)

%% Environmental Parameters
rho = 1.225; %density of air (kg/m^3)
g = 9.81; %gravitational acceleration (m/s^2)

%% Transformation Matrices
% Euler Angles
Tbi = @(phi,theta,xi) [cos(phi)*cos(xi)+sin(phi)*sin(theta)*sin(xi), cos(phi)*sin(xi)-sin(phi)*sin(theta)*cos(xi), -sin(phi)*cos(theta);
sin(phi)*cos(xi)-cos(phi)*sin(theta)*sin(xi), sin(phi)*sin(xi)+cos(phi)*sin(theta)*cos(xi), cos(theta)*cos(phi);
    cos(theta)*sin(xi), -cos(theta)*cos(xi), sin(theta)];
Tib = @(phi,theta,xi) Tbi(phi,theta,xi)';

% Initial Condition Angles
TibIc = @(hor,lay,a) [cos(hor)*cos(a)-sin(hor)*sin(lay)*sin(a), -cos(hor)*sin(a)-sin(hor)*sin(lay)*cos(a), -sin(hor)*cos(lay);
    cos(lay)*sin(a), cos(lay)*cos(a), -sin(lay);
sin(hor)*cos(a)+cos(hor)*sin(lay)*sin(a), -sin(hor)*sin(a)+cos(hor)*sin(lay)*cos(a), cos(hor)*cos(lay)];
TbiIc = @(hor,lay,a) Tib(hor,lay,a);

% From Rotational Velocities to Rate of Change of Euler Angles
Twe = @(phi,theta,xi) [cos(theta)*sin(xi),cos(xi),0;-cos(theta)*cos(xi),sin(xi),0;-sin(theta),0,-1]^-1;

%Inertial Axes
i1 = [1;0;0];
i2 = [0;1;0];
i3 = [0;0;1];
%% Loop Paramaters
% Iterator
n=1;
% Time:
t(1) = 0;

%% Initial Conditions
% Boomerang State:
theta0 = asin(-sin(thetaHor0)*sin(alpha0)+cos(thetaHor0)*sin(thetaLay0)*cos(alpha0)); %Precession angle(rad)
phi0 = acos(cos(thetaLay0)*cos(alpha0)/cos(theta0)); %Layover angle(rad)
xi0 = asin((sin(phi0)*sin(theta0)*cos(thetaHor0)+cos(theta0)*sin(thetaHor0))/cos(alpha0)); %Alignment angle(rad)
boomState(:,1) = [0;... % X - Position(m)
                  0;... % Y - Position(m)
                  Z0;... % Z - Height(m)
                  V0*cos(thetaHor0);... % Vx - Velocity in X-dir(m/s)
                  0;... % Vy - Velocity in Y-dir(m/s)
                  V0*sin(thetaHor0);... % Vz - Velocity in Z-dir(m/s)
                  phi0; % phi - euler-angle(rad)
                  theta0;... % theta - euler-angle(rad)
                  xi0;... % xi - euler-angle(rad)
                  0;... % w1 - Rotation Rate about boom 1-axis(rad/s)
                  0;... % w2 - Rotation Rate about boom 2-axis(rad/s)
                  w0]; % w3 - Rotation Rate about boom 3-axis(rad/s)
% Velocities:
V(1) = sqrt(sum(boomState(4:6,1).^2)); %Speed of Boomerang
Vu = boomState(4:6,1)/V(1); % Velocity Unit Vector
% Non-dimensional Rotational velocity:
Xi(1) = R*w0/V0;
% Body axes:
b1 = Tbi(boomState(7,n),boomState(8,n),boomState(9,n))*i1; %b1 expressed in inertial frame coordinates
b2 = Tbi(boomState(7,n),boomState(8,n),boomState(9,n))*i2; %b2 expressed in inertial frame coordinates
b3 = Tbi(boomState(7,n),boomState(8,n),boomState(9,n))*i3; %b3 expressed in inertial frame coordinates
% Angle of Attack
alpha(1) = alpha0; %(rad)
% Moments and Forces
[CL, CD, CMX(1), CMY, CMZ(n)] = boomerangCoefs(Xi(1),alpha(1),B); %Boomerang coefficient function
%CD = 0; CMZ(n) = 0; CL = 0; CMX(n) = 0; CMY = 0;
L(1) = .5*rho*V0^2*A*CL; %Lift(perpindicular to velocity, coplanar with velocity and b3 axis)(N)
D(1) = .5*rho*V0^2*A*CD; %Drag(in direction of velocity)(N)
Mx(1) = .5*rho*V0^2*A*R*CMX(1); %Rolling Moment(perpindicular to velocity vector)(Nm)
My(1) = .5*rho*V0^2*A*Roffset*CMY; %Pitching moment(parallel to velocity vector)(Nm)
Mz(1) = .5*rho*V0^2*A*R*CMZ(n); %Moment along b3 axis(Nm)
% Inertial Forces
LI = L(1)*cross(cross(Vu,b3),Vu); %Lift Vector
DI = D(1)*-Vu; %Drag Vector
GI = i3*m*-g; %Gravity Vector
F(:,1) = LI+DI+GI;
% Body Moments
M1 = -Mx(1)*dot(b1,(cross(b3,cross(Vu,b3))))+My(1)*dot(b1,cross(Vu,b3)); %Moment about body-1 axis(N)
M2 = -Mx(1)*dot(b2,cross(b3,cross(Vu,b3)))+My(1)*dot(b2,cross(Vu,b3)); %Moment about body-2 axis(N)
M3 = -Mz(1); %Moment about body-3 axis(N)
M(:,1) = [M1;M2;M3];

%% Loop
while t(n)<tmax
    % Iterator
    n = n+1;
    % Time
    t(n) = t(n-1)+dt;
    % Update State:
    boomState(:,n) = RK4(@boomN,boomState(:,n-1),F(:,n-1),M(:,n-1),B,dt);
    if boomState(3,n)<0
        t(end) = [];
        boomState(:,end) = [];
        break
    end
    % Velocities:
    V(n) = sqrt(sum(boomState(4:6,n).^2)); %Speed of Boomerang
    Vu = boomState(4:6,n)/V(n); % Velocity Unit Vector
    % Non-dimensional rotational velocity
    Xi(n) = boomState(12,n)*R/V(n);
    % Body axes
    b1 = Tbi(boomState(7,n),boomState(8,n),boomState(9,n))*i1; %b1 expressed in inertial frame coordinates
    b2 = Tbi(boomState(7,n),boomState(8,n),boomState(9,n))*i2; %b2 expressed in inertial frame coordinates
    b3 = Tbi(boomState(7,n),boomState(8,n),boomState(9,n))*i3; %b3 expressed in inertial frame coordinates
    % Angle of Attack
    alpha(n) = asin(dot(-b3,Vu));
    % Moments and Forces
    [CL, CD, CMX(n), CMY, CMZ(n)] = boomerangCoefs(Xi(n),alpha(n),B);
    %CD = 0; 
    %CMZ(n) = 0;
    %CD = 0; CMZ(n) = 0; CL = 0; CMX(n) = 0; CMY = 0;
    L(n) = .5*rho*V(n)^2*A*CL;
    D(n) = .5*rho*V(n)^2*A*CD;
    Mx(n) = .5*rho*V(n)^2*A*R*CMX(n);
    My(n) = .5*rho*V(n)^2*A*Roffset*CMY;
    Mz(n) = .5*rho*V(n)^2*A*R*CMZ(n);
    % Inertial Forces
    LI = L(n)*cross(cross(Vu,b3),Vu);
    DI = D(n)*-Vu;
    GI = i3*m*-g;
    F(:,n) = LI+DI+GI;
    % Apply Moments to rotational accelerations
    M1 = -Mx(n)*dot(b1,(cross(b3,cross(Vu,b3))))+My(n)*dot(b1,cross(Vu,b3));
    M2 = -Mx(n)*dot(b2,cross(b3,cross(Vu,b3)))+My(n)*dot(b2,cross(Vu,b3));
    M3 = -Mz(n);
    M(:,n) = [M1;M2;M3];
    
end

f = figure; hold on;
title('Boomerang Trajectory');
plot3(boomState(1,:),boomState(2,:),boomState(3,:)); view(-45,25);
plot3(boomState(1,1),boomState(2,1),boomState(3,1),'g*'); plot3(boomState(1,end),boomState(2,end),boomState(3,end),'r*');
axis equal;
hold off;

figure; hold on;
sgtitle('Boomerang Angles');
subplot(6,1,1); hold on; title('\phi');
plot(t,boomState(7,:)*180/pi);
xlabel('Time[s]'); ylabel('\phi[deg]');
hold off;
subplot(6,1,2); hold on; title('\theta');
plot(t,boomState(8,:)*180/pi);
xlabel('Time[s]'); ylabel('\theta[deg]');
hold off;
subplot(6,1,3); hold on; title('\alpha');
plot(t,alpha*180/pi);
xlabel('Time[s]'); ylabel('\alpha[deg]');
hold off;
subplot(6,1,4); hold on; title('w3');
plot(t,boomState(12,:)/(2*pi));
xlabel('Time[s]'); ylabel('w3[rotations/s]');
hold off;
subplot(6,1,5); hold on; title('Velocity');
plot(t,sqrt(sum(boomState(4:6,:).^2)));
xlabel('Time[s]'); ylabel('Velocity[m/s]');
hold off;
subplot(6,1,6); hold on; title('\Xi');
plot(t,Xi);
xlabel('Time[s]'); ylabel('\Xi');
hold off;

Energy = 1/2*m*sum(boomState(4:6,:).^2)+m*g*boomState(3,:)+1/2*(I(1,1)*boomState(10,:)+I(2,2)*boomState(11,:)+I(3,3)*boomState(12,:));
figure; hold on; title('Energy of Boomerang');
plot(t,Energy);
xlabel('Time[s]');
ylabel('Energy[J]');

figure; 
subplot(4,1,1);
plot(t,CMX)
xlabel('Time[s]');
ylabel('Coefficient of Rolling Moment');
subplot(4,1,2);
plot(t,Mx)
xlabel('Time[s]');
ylabel('Rolling Moment');
subplot(4,1,3);
plot(t,CMZ)
xlabel('Time[s]');
ylabel('Coefficient of Drag Moment');
subplot(4,1,4);
plot(t,Mz)
xlabel('Time[s]');
ylabel('Drag Moment');

% figure; hold on;
% plot(t,M(1,:));
% plot(t,M(2,:));
% plot(t,M(3,:));
% plot(t,Mx);
% plot(t,My);
% plot(t,Mz);
% plot(t,sqrt(M(1,:).^2+M(2,:).^2+M(3,:).^2));
% plot(t,sqrt(Mx.^2+My.^2+Mz.^2));
% legend('M1','M2','M3','Mx','My','Mz','Mag123','Magxyz');
% xlabel('Time[s]');
% ylabel('Moments[Nm]');

end

function N = boomN(boomState,F,M,B)

N = [boomState(4);...
    boomState(5);...
    boomState(6);...
    F(1)/B.m;...%+boomState(12)*boomState(5)-boomState(11)*boomState(6);...
    F(2)/B.m;...%+boomState(10)*boomState(6)-boomState(12)*boomState(4);...
    F(3)/B.m;...%+boomState(11)*boomState(4)-boomState(10)*boomState(5);...
    sin(boomState(9))/cos(boomState(8))*boomState(10)-cos(boomState(9))/cos(boomState(8))*boomState(11);...
    cos(boomState(9))*boomState(10)+sin(boomState(9))*boomState(11);...
    -sin(boomState(9))*tan(boomState(8))*boomState(10)+cos(boomState(9))*tan(boomState(8))*boomState(11)-boomState(12);...
    (M(1)+(B.I(2,2)-B.I(3,3))*boomState(11)*boomState(12))/B.I(1,1);...
    (M(2)+(B.I(3,3)-B.I(1,1))*boomState(10)*boomState(12))/B.I(2,2);...
    (M(3)+(B.I(1,1)-B.I(2,2))*boomState(10)*boomState(11))/B.I(3,3)];

end

function nextState = RK4(computeN,currentState,F,M,B,dt)

f1 = computeN(currentState,F,M,B);
f2 = computeN(currentState+dt*f1/2,F,M,B);
f3 = computeN(currentState+dt*f2/2,F,M,B);
f4 = computeN(currentState+dt*f3,F,M,B);

nextState = currentState+dt*(f1/6+(f2+f3)/3+f4/6);

end



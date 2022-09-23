function [t,boomState,flightParams] = boomerangTrajectorySimplified(S,B)

clear global

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
global rho; %density of air (kg/m^3)
rho = 1.225;
global g; %gravitational acceleration (m/s^2)
g = 9.81;
global Vwind;
Vwind = -S.Vw*[cosd(S.WindAng);sind(S.WindAng);0];

%% Inertial Axes
global i1 i2 i3;
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
                  w0] % w3 - Rotation Rate about boom 3-axis(rad/s)
              
%% Initial Forces, Moments, AOA, and Boomerang Non-Dimensional Coefficient:
[L,D,Mx,My,Mz,alpha,Xi,xi] = FandM(boomState,B);

%% Parameters to Return:
global flightParams
flightParams.L(1) = L;
flightParams.D(1) = D;
flightParams.Mx(1) = Mx;
flightParams.My(1) = My;
flightParams.Mz(1) = Mz;
flightParams.alpha(1) = alpha;
flightParams.Xi(1) = Xi;
flightParams.xi(1) = xi;

%% Loop
while t(n)<tmax && boomState(3,n)>=0
    % Iterator
    n = n+1;
    % Time
    t(n) = t(n-1)+dt;
    % Update State:
    [boomState(:,n)] = RK4(@boomN,boomState(:,n-1),@FandM,B,dt);
end

end

function [L,D,Mx,My,Mz,alpha,Xi,xi] = FandM(boomState,B)

%Initialize Global Variables:
global rho i2 i3 Vwind;
% Velocities:
V = sqrt(sum(boomState(4:6).^2)); %Speed of Boomerang
Vu = boomState(4:6)/V; % Velocity Unit Vector
Vw = sqrt(sum((boomState(4:6)-Vwind).^2));
Vwu = (boomState(4:6)-Vwind)/Vw;
% Non-dimensional rotational velocity
Xi = boomState(9)*B.R/Vw;
% Body Axis
b2 = Tbi(boomState(7),boomState(8),0)*i2; %b2 expressed in inertial frame coordinates
b3 = Tbi(boomState(7),boomState(8),0)*i3; %b3 expressed in inertial frame coordinates
% Angle of Attack
alpha = asin(dot(-b3,Vwu));
% Moments and Forces
[CL, CD, CMX, CMY, CMZ] = boomerangCoefsSimplified(Xi,alpha,B);
%CD = 0; CMZ = 0;
%CD = 0; CMZ = 0; CL = 0; CMX = 0; CMY = 0;
L = .5*rho*Vw^2*B.A*CL;
D = 2*.5*rho*Vw^2*B.A*CD;
Mx = .5*rho*Vw^2*B.A*B.R*CMX;
My = .5*rho*Vw^2*B.A*B.Roffset*CMY;
Mz = 2*.5*rho*Vw^2*B.A*B.R*CMZ;
xi = asin(-dot(Vwu,b2)/cos(alpha));
end

function [N,L,D,Mx,My,Mz,alpha,Xi,xi] = boomN(boomState,FandM,B)

[L,D,Mx,My,Mz,alpha,Xi,xi] = FandM(boomState,B);
global g
Vmag = norm(boomState(4:6));

N = [boomState(4);...
    boomState(5);...
    boomState(6);...
    L/(B.m*cos(alpha))*(-sin(boomState(7))*cos(boomState(8)) + sin(alpha)*boomState(4)/Vmag)-D*boomState(4)/(B.m*Vmag);...
    L/(B.m*cos(alpha))*(cos(boomState(7))*cos(boomState(8)) + sin(alpha)*boomState(5)/Vmag)-D*boomState(5)/(B.m*Vmag);...
    L/(B.m*cos(alpha))*(sin(boomState(8)) + sin(alpha)*boomState(6)/Vmag)-D*boomState(6)/(B.m*Vmag)-g;...
    1/(B.I(3,3)*boomState(9)*cos(boomState(8)))*(Mx*cos(xi)+My*sin(xi));...
    1/(B.I(3,3)*boomState(9))*(My*cos(xi)-Mx*sin(xi));...
    -Mz/B.I(3,3)];

end

function [nextState] = RK4(computeN,currentState,FandM,B,dt)

[f1,~,~,~,~,~,~,~] = computeN(currentState,FandM,B);
[f2,~,~,~,~,~,~,~] = computeN(currentState+dt*f1/2,FandM,B);
[f3,~,~,~,~,~,~,~] = computeN(currentState+dt*f2/2,FandM,B);
[f4,L,D,Mx,My,Mz,alpha,Xi,xi] = computeN(currentState+dt*f3,FandM,B);

nextState = currentState+dt*(f1/6+(f2+f3)/3+f4/6);

global flightParams
flightParams.L(end+1) = L;
flightParams.D(end+1) = D;
flightParams.Mx(end+1) = Mx;
flightParams.My(end+1) = My;
flightParams.Mz(end+1) = Mz;
flightParams.alpha(end+1) = alpha;
flightParams.Xi(end+1) = Xi;
flightParams.xi(end+1) = xi;

end

function T = Tbi(phi,theta,xi)

T = [cos(phi)*cos(xi)+sin(phi)*sin(theta)*sin(xi), cos(phi)*sin(xi)-sin(phi)*sin(theta)*cos(xi), -sin(phi)*cos(theta);
sin(phi)*cos(xi)-cos(phi)*sin(theta)*sin(xi), sin(phi)*sin(xi)+cos(phi)*sin(theta)*cos(xi), cos(theta)*cos(phi);
    cos(theta)*sin(xi), -cos(theta)*cos(xi), sin(theta)];

end


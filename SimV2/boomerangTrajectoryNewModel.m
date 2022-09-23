function [t,boomState,flightParams] = boomerangTrajectoryNewModel(S,B)

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
                  xi0;... % xi - euler-angle(rad)
                  0;... % w1 - Rotation Rate about boom 1-axis(rad/s)
                  0;... % w2 - Rotation Rate about boom 2-axis(rad/s)
                  w0]; % w3 - Rotation Rate about boom 3-axis(rad/s)
              
%% Initial Forces, Moments, AOA, and Boomerang Non-Dimensional Coefficient:
[F,M,alpha,theta] = FandM(boomState,B);

%% Parameters to Return:
global flightParams
flightParams.F(:,1) = F;
flightParams.M(:,1) = M;
flightParams.alpha(1) = alpha;
flightParams.theta(1) = theta;

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

function [F,M,alpha,theta] = FandM(boomState,B)

global rho i3 g

b1 = Tbi(boomState(7),boomState(8),boomState(9))*[1;0;0]; %b1 expressed in inertial frame coordinates(aligned with boom wing)
b3 = Tbi(boomState(7),boomState(8),boomState(9))*[0;0;1]; %b3 expressed in inertial frame coordinates

V = boomState(4:6);
Vmag = norm(V);
if Vmag == 0
    Vu = zeros(3,1);
    alpha = 0;
    theta = 0;
else
    Vu = V/Vmag;
    alpha = asin(dot(-Vu,b3));
    Vcb3 = cross(Vu,b3)/norm(cross(Vu,b3));
    bcV = cross(b3,Vcb3)/norm(cross(b3,Vcb3));
    theta = atan2(dot(b1,bcV),dot(b1,Vcb3));
end

Rmin = tand(30)*B.C;
R = B.R;

FEASegs = 50;
segLengths = (R-Rmin)/FEASegs;
r = linspace(Rmin,R-segLengths,FEASegs)+segLengths/2;

thetas = [theta,theta+2/3*pi,theta+4/3*pi];
blade1 = b1;
blade2 = blade1*cos(2/3*pi) + cross(b3,blade1)*sin(2/3*pi)+b3*dot(b3,blade1)*(1-cos(2/3*pi));
blade3 = blade1*cos(4/3*pi) + cross(b3,blade1)*sin(4/3*pi)+b3*dot(b3,blade1)*(1-cos(4/3*pi));
blades = [blade1,blade2,blade3];

F = zeros(3,1);
M = zeros(3,1);

% figure(1); hold on; axis equal
% quiver3(0,0,0,R*blades(1,1),R*blades(2,1),R*blades(3,1),'g');
% quiver3(0,0,0,R*blades(1,2),R*blades(2,2),R*blades(3,2),'k');
% quiver3(0,0,0,R*blades(1,3),R*blades(2,3),R*blades(3,3),'k');
% quiver3(0,0,0,b3(1),b3(2),b3(3),'k');

for i = 1:length(thetas)
    for j = 1:length(r)
        alphaFoil = atan2(Vmag*sin(alpha),(Vmag*cos(alpha)*cos(thetas(i))+boomState(12)*r(j)));
        VFoil = sqrt(Vmag^2*(sin(alpha)^2+cos(alpha)^2*cos(thetas(i))^2)+2*boomState(12)*r(j)*Vmag*cos(alpha)*cos(thetas(i))+boomState(12)^2*r(j)^2);
        [CL,CD] = Rhode32Coefs(alphaFoil);
        L = .5*rho*segLengths*B.C*CL*VFoil^2;
        %L = 0;
        D = .5*rho*segLengths*B.C*CD*VFoil^2;
        %D = 0;
        
        
        bcb3 = cross(blades(:,i),b3);
        Dhat = bcb3*cos(alphaFoil) + cross(blades(:,i),bcb3)*sin(alphaFoil)+blades(:,i)*dot(blades(:,i),bcb3)*(1-cos(alphaFoil));
        Lhat = bcb3*cos(alphaFoil-pi/2) + cross(blades(:,i),bcb3)*sin(alphaFoil-pi/2)+blades(:,i)*dot(blades(:,i),bcb3)*(1-cos(alphaFoil-pi/2));
        
        F = F + D*Dhat+L*Lhat;
        
        M = M + cross(blades(:,i)*r(j)-bcb3*B.Roffset,D*Dhat+L*Lhat);

%         quiver3(r(j)*blades(1,i),r(j)*blades(2,i),r(j)*blades(3,i),Lhat(1),Lhat(2),Lhat(3),'r');
%         quiver3(r(j)*blades(1,i),r(j)*blades(2,i),r(j)*blades(3,i),Dhat(1),Dhat(2),Dhat(3),'b');
    end
end

% F = F - i3*g*B.m;
% quiver3(0,0,0,F(1)/sqrt(sum(F.^2)),F(2)/sqrt(sum(F.^2)),F(3)/sqrt(sum(F.^2)),'g');
% quiver3(0,0,0,M(1)/sqrt(sum(M.^2)),M(2)/sqrt(sum(M.^2)),M(3)/sqrt(sum(M.^2)),'r');

%dot(F,b3)/(.5*B.rho*B.C*(R-Rmin)*Vmag^2)
% dot(M,b3)

end

function [N,F,M,alpha,theta] = boomN(boomState,FandM,B)

[F,M,alpha,theta] = FandM(boomState,B);

Mb = Tib(boomState(7),boomState(8),boomState(9))*M;

N = [boomState(4);...
    boomState(5);...
    boomState(6);...
    F/B.m;...%+cross(boomState(4:6),Tib(boomState(7),boomState(8),boomState(9))*boomState(10:12));...
    Twe(boomState(7),boomState(8),boomState(9))*[boomState(10);boomState(11);boomState(12)];...
    (Mb(1)+(B.I(2,2)-B.I(3,3))*boomState(11)*boomState(12))/B.I(1,1);...
    (Mb(2)+(B.I(3,3)-B.I(1,1))*boomState(10)*boomState(12))/B.I(2,2);...
    (Mb(3)+(B.I(1,1)-B.I(2,2))*boomState(10)*boomState(11))/B.I(3,3)];

end

function [nextState] = RK4(computeN,currentState,FandM,B,dt)

[f1,~,~,~,~] = computeN(currentState,FandM,B);
[f2,~,~,~,~] = computeN(currentState+dt*f1/2,FandM,B);
[f3,~,~,~,~] = computeN(currentState+dt*f2/2,FandM,B);
[f4,F,M,alpha,theta] = computeN(currentState+dt*f3,FandM,B);

nextState = currentState+dt*(f1/6+(f2+f3)/3+f4/6);

global flightParams
flightParams.F(:,end+1) = F;
flightParams.M(:,end+1) = M;
flightParams.alpha(end+1) = alpha;
flightParams.theta(end+1) = theta;

end

function T = Tbi(phi,theta,xi)

T = [cos(phi)*cos(xi)+sin(phi)*sin(theta)*sin(xi), cos(phi)*sin(xi)-sin(phi)*sin(theta)*cos(xi), -sin(phi)*cos(theta);
sin(phi)*cos(xi)-cos(phi)*sin(theta)*sin(xi), sin(phi)*sin(xi)+cos(phi)*sin(theta)*cos(xi), cos(theta)*cos(phi);
    cos(theta)*sin(xi), -cos(theta)*cos(xi), sin(theta)];

end

function T = Tib(phi,theta,xi)

T = Tbi(phi,theta,xi)';

end

function T = Twe(phi,theta,xi)

T = [sin(xi)/cos(theta),-cos(xi)/cos(theta),0;cos(xi),sin(xi),0;-tan(theta)*sin(xi),tan(theta)*cos(xi),-1];

end


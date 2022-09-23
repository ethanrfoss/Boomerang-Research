
%% Controllable Simulation Paramaters
%Throwing in x-direction
V0 = 17; %25; %initial speed magnitude(m/s)
Z0 = 1.5; %(m)
w0 = 5*2*pi; %15*2*pi; %(rad/s)

thetaHor0 = 0*pi/180; %(rad)
thetaLay0 = 90*pi/180; %(rad)
alpha0 = 15*pi/180; %(rad)

Vw = 0; % Wind Strength(m/s)
WindAng = 80; % Angle of Wind Against Boomerang Throw(deg)

dt = 1/10000; %(s)
tmax = 20; %(s)
%% New Simulation Object:
S = newBoomerangSimulation(V0,Z0,w0,thetaHor0,thetaLay0,alpha0,Vw,WindAng,dt,tmax);

%% Boomerang Parameters:
R = .208; %length of blade (m)
C = .04; %chord of blade (m)
A = .0249; %3*R*C;  %area of boomn (m^2)
l = .01; %Boomerang Thickness
m = .09779; %mass (kg)
rho = 1.225; %density (kg/m^3)
Roffset = .01; %Distance from center of lift to axis of symettry of blade(m)
I = 7e-04; % I33
I = [.000739279,0,0;0,.000740591,0;0,0,.001479488];%[I/2,0,0;0,I/2,0;0,0,I]; %Inertia Tensor(kgm^2)
n = 3; % 3 bladed boomerang

%% Boomerang Airfoil:
% Foil.CL0p = 0.39; 
% Foil.CL0m = -0.39; 
% Foil.CLap = 4.5; 
% Foil.CLam = 4.5;
% Foil.CD0p = 0.05; 
% Foil.CD0m = 0.05;
% Foil.CDap = 0.1;
% Foil.CDam = 0.1;
Foil.CL0p = 0.617; 
Foil.CL0m = -0.617; 
Foil.CLap = 5.3; 
Foil.CLam = 5.3;
Foil.CD0p = 0.01526; 
Foil.CD0m = 0.01526;
Foil.CDap = 0.1150;
Foil.CDam = 0.1150;

%% New Boomerang Object:
B = boomerang(Foil,R,C,A,Roffset,l,m,rho,I,n);

%% Iterate Through Alphas

thetaHor0 = 0*pi/180;
thetaLay0 = 90*pi/180;
alpha0 = [0:45]*pi/180;

for i = 1:length(alpha0)

    theta0 = real(asin(-sin(thetaHor0)*sin(alpha0(i))+cos(thetaHor0)*sin(thetaLay0)*cos(alpha0(i)))); %Precession angle(rad)
    phi0 = real(acos(cos(thetaLay0)*cos(alpha0(i))/cos(theta0))); %Layover angle(rad)
    xi0 = real(asin((sin(phi0)*sin(theta0)*cos(thetaHor0)+cos(theta0)*sin(thetaHor0))/cos(alpha0(i)))); %Alignment angle(rad)

    boomState = [0;... % X - Position(m)
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
    
    [F,M] = boomForces(boomState,B);
    Vu = boomState(4:6)/norm(boomState(4:6));
    V = sqrt(sum(boomState(4:6).^2)); %Speed of Boomerang
    T = [cos(phi0)*cos(xi0)+sin(phi0)*sin(theta0)*sin(xi0), cos(phi0)*sin(xi0)-sin(phi0)*sin(theta0)*cos(xi0), -sin(phi0)*cos(theta0);
    sin(phi0)*cos(xi0)-cos(phi0)*sin(theta0)*sin(xi0), sin(phi0)*sin(xi0)+cos(phi0)*sin(theta0)*cos(xi0), cos(theta0)*cos(phi0);
    cos(theta0)*sin(xi0), -cos(theta0)*cos(xi0), sin(theta0)];
    b1 = T(:,1); b2 = T(:,2); b3 = T(:,3);
    
    if V0 == 0
        D(i) = 0;
        L(i) = dot(F,b3);
        G(i) = real(sqrt(norm(F)^2-D(i)^2-L(i)^2));
    else
        D(i) = dot(Vu,F);
        L(i) = dot(F,cross(cross(Vu,b3),Vu)/norm(cross(cross(Vu,b3),Vu)));
        G(i) = real(sqrt(norm(F)^2-D(i)^2-L(i)^2));
    end
    
    %M1(i) = dot(b1,M);
    %M2(i) = dot(b2,M);
    %M3(i) = dot(b3,M); %???????????
    Mb = T*M;
    M1(i) = Mb(1);
    M2(i) = Mb(2);
    M3(i) = Mb(3);
    
    Xi = boomState(12)*B.R/V;
    % Moments and Forces
    [CL, CD, CMX, CMY, CMZ] = boomerangCoefs(Xi,alpha0(i),B);
    %CD = 0; CMZ = 0;
    %CD = 0; CMZ = 0; CL = 0; CMX = 0; CMY = 0;
    Lp(i) = .5*rho*V^2*B.A*CL;
    Dp(i) = 2*.5*rho*V^2*B.A*CD;
    Mxp(i) = .5*rho*V^2*B.A*B.R*CMX;
    Myp(i) = .5*rho*V^2*B.A*B.Roffset*CMY;
    Mzp(i) = 2*.5*rho*V^2*B.A*B.R*CMZ;
    Mp(:,i) = T*(-Mxp(i)*cross(b3,cross(Vu,b3))/norm(cross(b3,cross(Vu,b3)))+Myp(i)*cross(Vu,b3)/norm(cross(Vu,b3))-Mzp(i)*b3);
    
end

figure(1); 
subplot(6,1,1); hold on; title('Drag');
plot(alpha0*180/pi,-D);
plot(alpha0*180/pi,Dp);
legend('Ethan Model','Prasad Model');
xlabel('\alpha'); ylabel('D[N]');
subplot(6,1,2); hold on; title('Lift');
plot(alpha0*180/pi,L);
plot(alpha0*180/pi,Lp);
xlabel('\alpha'); ylabel('L[N]');
subplot(6,1,3); hold on; title('Side Force');
plot(alpha0*180/pi,G);
plot(alpha0*180/pi,zeros(1,length(alpha0)));
xlabel('\alpha'); ylabel('G[N]');
subplot(6,1,4); hold on; title('Body-1 Moment');
plot(alpha0*180/pi,M1);
plot(alpha0*180/pi,Mxp);
xlabel('\alpha'); ylabel('M_1[Nm]');
subplot(6,1,5); hold on; title('Body-2 Moment');
plot(alpha0*180/pi,M2);
plot(alpha0*180/pi,Myp);
xlabel('\alpha'); ylabel('M_2[Nm]');
subplot(6,1,6); hold on; title('Body-3 Moment');
plot(alpha0*180/pi,M3);
plot(alpha0*180/pi,Mzp);
xlabel('\alpha'); ylabel('M_3[Nm]');


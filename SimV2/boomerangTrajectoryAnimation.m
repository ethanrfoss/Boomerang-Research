function boomerangTrajectoryAnimation(boomState,flightParams,t,dt)
%% Objective:
% Animate boomerang trajectory with animated boomerang
% input variables:
%   phi
%   theta
%   xi
%   X
%   Y
%   Z
%   Vx
%   Vy
%   Vz
%   Ax
%   Ay
%   Az
%% Initialize Inputs:
X = boomState(1,:);
Y = boomState(2,:);
Z = boomState(3,:);
Vx = boomState(4,:);
Vy = boomState(5,:);
Vz = boomState(6,:);
phi = boomState(7,:);
theta = boomState(8,:);
xi = boomState(9,:);

Mx = flightParams.Mx;
My = flightParams.My;
Mz = flightParams.Mz;
alpha = flightParams.alpha;

%% Calculate Inertial Frame Accelerations:
Ax = diff(Vx)/dt;
Ay = diff(Vy)/dt;
Az = diff(Vz)/dt;

%% Transformation matrix
Tbi = @(p,t,x) [cos(p)*cos(x)+sin(p)*sin(t)*sin(x), cos(p)*sin(x)-sin(p)*sin(t)*cos(x), -sin(p)*cos(t);
sin(p)*cos(x)-cos(p)*sin(t)*sin(x), sin(p)*sin(x)+cos(p)*sin(t)*cos(x), cos(t)*cos(p);
    cos(t)*sin(x), -cos(t)*cos(x), sin(t)];
Tib = @(p,t,x) Tbi(p,t,x)';

%% Load Boomerang Shape data
BScale = 1; % Boomerang Scale
load('boomerangShape.mat');
Boom = BScale*[pos'; zeros(1,length(pos))];

%% Request Gif Creation
createGif = input('Create Gif of Animation? Type yes or no: ','s');
if isequal(createGif,'yes')
    saveName = input('Enter Gif Save Name: ','s');
end
    
%% Initialize Velocity
V0 = sqrt(Vx(1).^2+Vy(1).^2+Vz(1).^2);
A0 = sqrt(Ax(1).^2+Ay(1).^2+Az(1).^2);
M0 = sqrt(Mx(1).^2+My(1).^2+Mz(1).^2);

%% Initialize Quiver Objects
h = figure('WindowState','maximized'); subplot(4,2,[1 3 5 7]); hold on; view(-45,45); 
axis equal; axis([min(X)-1,max(X)+1,min(Y)-1,max(Y)+1,-1,max(Z)+1]); 
qb3 = quiver3(0,0,0,0,0,0,'AutoScaleFactor',1,'MaxHeadSize',.5);
qv = quiver3(0,0,0,0,0,0,'AutoScaleFactor',BScale/V0,'MaxHeadSize',.5);
qF = quiver3(0,0,0,0,0,0,'AutoScaleFactor',BScale/A0,'MaxHeadSize',.5);
qM = quiver3(0,0,0,0,0,0,'AutoScaleFactor',BScale/M0,'MaxHeadSize',.5);
pob = plot3(0,0,0);
lB = plot3(0,0,0,'r*');

%% Plot Trajectory and other figure objects
p = plot3(X,Y,Z);
L = legend([qb3,qv,qF,qM,pob,lB,p],{'Boomerang Normal Axis',sprintf('Velocity Vector of Boomerang(V = %.2fm/s)(\\alpha = %.2fdeg)',sqrt(Vx(1).^2+Vy(1).^2+Vz(1).^2),alpha(1)*180/pi),sprintf('Acceleration Vector of Boomerang(A = %.2fm/s^2)',sqrt(Ax(1).^2+Ay(1).^2+Az(1).^2)),sprintf('Moment Vector of Boomerang(M = %.2fNm)',sqrt(Mx(1).^2+My(1).^2+Mz(1).^2)),'Boomerang',sprintf('Position(X = %.2fm,Y = %.2fm,Z = %.2fm)',X(1),Y(1),Z(1)),'Boomerang Trajectory'},'Location','southoutside');
ta = title(sprintf('Boomerang Animation(t = %.2f)',t(1)));
hold off;

%% Other Subplots
subplot(4,2,2); hold on;
plot(t,alpha*180/pi);
pa = plot(0,0,'r*');
title('AOA');
xlabel('Time[s]');
ylabel('\alpha[deg]'); 
hold off;

subplot(4,2,4); hold on;
plot(t,phi*180/pi);
pp = plot(0,0,'r*');
title('\phi');
xlabel('Time[s]');
ylabel('\phi[deg]');
hold off;

subplot(4,2,6); hold on;
plot(t,theta*180/pi);
pt = plot(0,0,'r*');
title('\theta');
xlabel('Time[s]');
ylabel('\theta[deg]');
hold off;

subplot(4,2,8); hold on;
plot(t,xi*180/pi);
px = plot(0,0,'r*');
title('\xi');
xlabel('Time[s]');
ylabel('\xi[deg]');
hold off;

%% Animating Loop
for i=1:ceil(.01/dt):length(phi)-1
    %% Determine b3 and object coords
    b3 = Tbi(phi(i),theta(i),xi(i))*[0;0;1];
    Boomb = Tbi(phi(i),theta(i),xi(i))*Boom;
    
    %% Set Data on Plot
    set(qb3,'XData',X(i),'YData',Y(i),'ZData',Z(i),'UData',b3(1),'VData',b3(2),'WData',b3(3));
    set(qv,'XData',X(i),'YData',Y(i),'ZData',Z(i),'UData',Vx(i),'VData',Vy(i),'WData',Vz(i),'Visible','off');
    set(qF,'XData',X(i),'YData',Y(i),'ZData',Z(i),'UData',Ax(i),'VData',Ay(i),'WData',Az(i),'Visible','off');
    set(qM,'XData',X(i),'YData',Y(i),'ZData',Z(i),'UData',Mx(i),'VData',My(i),'WData',Mz(i));
    set(pob,'XData',X(i)+Boomb(1,:),'YData',Y(i)+Boomb(2,:),'ZData',Z(i)+Boomb(3,:));
    set(lB,'XData',X(i),'YData',Y(i),'ZData',Z(i));
    set(L,'String',{'Boomerang Normal Axis',sprintf('Velocity Vector of Boomerang(V = %.2fm/s)(\\alpha = %.2fdeg)',sqrt(Vx(i).^2+Vy(i).^2+Vz(i).^2),alpha(i)*180/pi),sprintf('Acceleration Vector of Boomerang(A = %.2fm/s^2)',sqrt(Ax(i).^2+Ay(i).^2+Az(i).^2)),sprintf('Moment Vector of Boomerang(M = %.2fNm)',sqrt(Mx(i).^2+My(i).^2+Mz(i).^2)),'Boomerang',sprintf('Position(X = %.2fm,Y = %.2fm,Z = %.2fm)',X(i),Y(i),Z(i)),'Boomerang Trajectory'});
    set(ta,'String',sprintf('Boomerang Animation(t = %.2f)',t(i)));
    set(pa,'XData',t(i),'YData',alpha(i)*180/pi);
    set(pp,'XData',t(i),'YData',phi(i)*180/pi);
    set(pt,'XData',t(i),'YData',theta(i)*180/pi);
    set(px,'XData',t(i),'YData',xi(i)*180/pi);
    drawnow;
    
    
    %% Optional Gif
    if isequal(createGif,'yes')
        frame = getframe(h);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i == 1
            imwrite(imind,cm,[cd '\SavedGifs\' saveName '.gif'],'gif','DelayTime',0, 'Loopcount',inf); 
        else
            imwrite(imind,cm,[cd '\SavedGifs\' saveName '.gif'],'gif','DelayTime',0,'WriteMode','append'); 
        end
    end
end
end
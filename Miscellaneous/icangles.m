function icangles(hor,lay,a,objectcoords)
%% Objective:
%   Plot Body fixed coordinate system with respect to inertial coordinate
%   system based off of euler angles
%   
%
% input variables:
%   phi
%   theta
%   psi
% output variables:
%   none
% functions called:
%   none
%
%
%% Transformation matrix
Tib = [cos(hor)*cos(a)-sin(hor)*sin(lay)*sin(a), -cos(hor)*sin(a)-sin(hor)*sin(lay)*cos(a), -sin(hor)*cos(lay);
    cos(lay)*sin(a), cos(lay)*cos(a), -sin(lay);
sin(hor)*cos(a)+cos(hor)*sin(lay)*sin(a), -sin(hor)*sin(a)+cos(hor)*sin(lay)*cos(a), cos(hor)*cos(lay)];
%% Inertial Frame
i1 = [1 0 0]'; i2 = [0 1 0]'; i3 = [0 0 1]';
%% Body Fixed Frame
b1 = Tib*i1;
b2 = Tib*i2;
b3 = Tib*i3;

%% n-s
n = [cos(hor) 0 sin(hor)];
s = [-sin(lay)*sin(hor) cos(lay) cos(hor)*sin(lay)];

%% Transform Object to Body frame(boomerang initially in XZ plane)
for i = 1:length(objectcoords)
    temp = [objectcoords(i,1); 0; objectcoords(i,2)];
    temp2 = Tib*temp;
    xob(i) = temp2(1);
    yob(i) = temp2(2);
    zob(i) = temp2(3);
end

%% Find Velocity along b3 vector
VI = [cos(hor) 0 sin(hor)];
%% Plot
v = view(3); hold on; view(-100,30); 
xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
qi1 = quiver3(0,0,0,i1(1),i1(2),i1(3),1,'color','k');
qi2 = quiver3(0,0,0,i2(1),i2(2),i2(3),1,'color','k');
qi3 = quiver3(0,0,0,i3(1),i3(2),i3(3),1,'color','k');

qb1 = quiver3(0,0,0,b1(1),b1(2),b1(3),1,'color','b');
qb2 = quiver3(0,0,0,b2(1),b2(2),b2(3),1,'color','b');
qb3 = quiver3(0,0,0,b3(1),b3(2),b3(3),1,'color','b');

qv = quiver3(0,0,0,VI(1),VI(2),VI(3),1,'color','m');

qn = quiver3(0,0,0,n(1),n(2),n(3),1,'--','color','g');
qs = quiver3(0,0,0,s(1),s(2),s(3),1,'--','color','g');

ti1 = text(i1(1),i1(2),i1(3)+.1,'i1');
ti2 = text(i2(1),i2(2),i2(3)+.1,'i2');
ti3 = text(i3(1),i3(2),i3(3)+.1,'i3');

tb1 = text(b1(1),b1(2),b1(3)+.1,'b1');
tb2 = text(b2(1),b2(2),b2(3)+.1,'b2');
tb3 = text(b3(1),b3(2),b3(3)+.1,'b3');

tn = text(n(1),n(2),n(3)+.1,'n');

tve = text(VI(1),VI(2),VI(3)+.1,'V');

pob = plot3(xob,yob,zob);


%% Draw Angles
%i1 to n - Hor
 xang = linspace(i1(1),n(1),10);
 yang = linspace(i1(2),n(2),10);
 zang = linspace(i1(3),n(3),10);
 norm = sqrt(xang.^2+yang.^2+zang.^2);
 normx = xang./(4*norm); normy = yang./(4*norm); normz = zang./(4*norm);
 plot3(normx,normy,normz,'--','color','k')
 text(normx(end/2), normy(end/2), normz(end/2)+.1,'\ThetaH');
 
 %i2 to s - Lay 
 xang = linspace(i2(1),s(1),10);
 yang = linspace(i2(2),s(2),10);
 zang = linspace(i2(3),s(3),10);
 norm = sqrt(xang.^2+yang.^2+zang.^2);
 normx = xang./(4*norm); normy = yang./(4*norm); normz = zang./(4*norm);
 plot3(normx,normy,normz,'--','color','k')
 text(normx(end/2), normy(end/2), normz(end/2)+.1,'\ThetaL');
 
  %s to b2 - a
 xang = linspace(s(1),b2(1),10);
 yang = linspace(s(2),b2(2),10);
 zang = linspace(s(3),b2(3),10);
 norm = sqrt(xang.^2+yang.^2+zang.^2);
 normx = xang./(4*norm); normy = yang./(4*norm); normz = zang./(4*norm);
 plot3(normx,normy,normz,'--','color','k')
 text(normx(end/2), normy(end/2), normz(end/2)+.1,'\alpha');
 
 %velocity to v3
%  xang = linspace(velocity(1),v3(1),10);
%  yang = linspace(velocity(2),v3(2),10);
%  zang = linspace(velocity(3),v3(3),10);
%  norm = sqrt(xang.^2+yang.^2+zang.^2);
%  normx = xang./(4*norm); normy = yang./(4*norm); normz = zang./(4*norm);
%  plot3(normx,normy,normz,'--','color','k')
%  text(normx(end/2), normy(end/2), normz(end/2)+.1,'1-\alpha');
 
 %% Finding Tib
%  Rphii3 = cos(phi)*eye(3) +(1-cos(phi))*i3*i3'-sin(phi)*[0 i3(3) -i3(2);-i3(3) 0 i3(1); i3(2), -i3(1) 0];
% Rthetan = cos(theta)*eye(3) +(1-cos(theta))*n*n'-sin(theta)*[0 n(3) -n(2);-n(3) 0 n(1); n(2), -n(1) 0];
% Rpsib3 = cos(psi)*eye(3) +(1-cos(psi))*b3*b3'-sin(psi)*[0 b3(3) -b3(2);-b3(3) 0 b3(1); b3(2), -b3(1) 0];
% Rib = Rpsib3*Rthetan*Rphii3;

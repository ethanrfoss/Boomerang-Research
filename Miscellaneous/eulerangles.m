function v = eulerangles(phi, theta,psi,objectcoords,velocity)
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
Tib = [cos(phi)*cos(psi)-sin(phi)*cos(theta)*sin(psi), sin(phi)*cos(psi)+cos(phi)*cos(theta)*sin(psi), sin(theta)*sin(psi);
-cos(phi)*sin(psi)-sin(phi)*cos(theta)*cos(psi), -sin(phi)*sin(psi)+cos(phi)*cos(theta)*cos(psi), sin(theta)*cos(psi);
    sin(phi)*sin(theta), -cos(phi)*sin(theta), cos(theta)]';
%% Inertial Frame
i1 = [1 0 0]'; i2 = [0 1 0]'; i3 = [0 0 1]';
%% Body Fixed Frame
b1 = Tib*i1;
b2 = Tib*i2;
b3 = Tib*i3;

%% n-s
n = cos(phi)*i1+sin(phi)*i2;
s = -sin(psi)*i1+cos(psi)*i2;

%% Transform Object to Body frame
for i = 1:length(objectcoords);
    temp = [objectcoords(i,1); objectcoords(i,2); 0];
    temp2 = Tib*temp;
    xob(i) = temp2(1);
    yob(i) = temp2(2);
    zob(i) = temp2(3);
end

%% Find Velocity along b3 vector
v3 = Tib*(velocity.*i3)
%% Plot
v = view(3); hold on; view(135,45); 
whos v
xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
qi1 = quiver3(0,0,0,i1(1),i1(2),i1(3),1,'color','k');
qi2 = quiver3(0,0,0,i2(1),i2(2),i2(3),1,'color','k');
qi3 = quiver3(0,0,0,i3(1),i3(2),i3(3),1,'color','k');

qb1 = quiver3(0,0,0,b1(1),b1(2),b1(3),1,'color','b');
qb2 = quiver3(0,0,0,b2(1),b2(2),b2(3),1,'color','b');
qb3 = quiver3(0,0,0,b3(1),b3(2),b3(3),1,'color','b');

qn = quiver3(0,0,0,n(1),n(2),n(3),1,'--','color','g');
%quiver3(0,0,0,s(1),s(2),s(3),1,'--','color','g');

qve = quiver3(0,0,0,velocity(1),velocity(2),velocity(3),1,'color','m');
qve3 = quiver3(0,0,0,v3(1),v3(2),v3(3),1,'color','m');

ti1 = text(i1(1),i1(2),i1(3)+.1,'i1');
ti2 = text(i2(1),i2(2),i2(3)+.1,'i2');
ti3 = text(i3(1),i3(2),i3(3)+.1,'i3');

tb1 = text(b1(1),b1(2),b1(3)+.1,'b1');
tb2 = text(b2(1),b2(2),b2(3)+.1,'b2');
tb3 = text(b3(1),b3(2),b3(3)+.1,'b3');

tn = text(n(1),n(2),n(3)+.1,'n');

tve = text(velocity(1),velocity(2),velocity(3)+.1,'V');
tve3 = text(v3(1),v3(2),v3(3)+.1,'V3');

pob = plot3(xob,yob,zob);


%% Draw Angles
%i1 to n - phi
 xang = linspace(i1(1),n(1),10);
 yang = linspace(i1(2),n(2),10);
 zang = linspace(i1(3),n(3),10);
 norm = sqrt(xang.^2+yang.^2+zang.^2);
 normx = xang./(4*norm); normy = yang./(4*norm); normz = zang./(4*norm);
 plot3(normx,normy,normz,'--','color','k')
 text(normx(end/2), normy(end/2), normz(end/2)+.1,'\phi');
 
 %i3 to b3 - theta 
 xang = linspace(i3(1),b3(1),10);
 yang = linspace(i3(2),b3(2),10);
 zang = linspace(i3(3),b3(3),10);
 norm = sqrt(xang.^2+yang.^2+zang.^2);
 normx = xang./(4*norm); normy = yang./(4*norm); normz = zang./(4*norm);
 plot3(normx,normy,normz,'--','color','k')
 text(normx(end/2), normy(end/2), normz(end/2)+.1,'\theta');
 
  %n to b1 - psi
 xang = linspace(n(1),b1(1),10);
 yang = linspace(n(2),b1(2),10);
 zang = linspace(n(3),b1(3),10);
 norm = sqrt(xang.^2+yang.^2+zang.^2);
 normx = xang./(4*norm); normy = yang./(4*norm); normz = zang./(4*norm);
 plot3(normx,normy,normz,'--','color','k')
 text(normx(end/2), normy(end/2), normz(end/2)+.1,'\psi');
 
 %velocity to v3
 xang = linspace(velocity(1),v3(1),10);
 yang = linspace(velocity(2),v3(2),10);
 zang = linspace(velocity(3),v3(3),10);
 norm = sqrt(xang.^2+yang.^2+zang.^2);
 normx = xang./(4*norm); normy = yang./(4*norm); normz = zang./(4*norm);
 plot3(normx,normy,normz,'--','color','k')
 text(normx(end/2), normy(end/2), normz(end/2)+.1,'1-\alpha');
 
 %% Finding Tib
%  Rphii3 = cos(phi)*eye(3) +(1-cos(phi))*i3*i3'-sin(phi)*[0 i3(3) -i3(2);-i3(3) 0 i3(1); i3(2), -i3(1) 0];
% Rthetan = cos(theta)*eye(3) +(1-cos(theta))*n*n'-sin(theta)*[0 n(3) -n(2);-n(3) 0 n(1); n(2), -n(1) 0];
% Rpsib3 = cos(psi)*eye(3) +(1-cos(psi))*b3*b3'-sin(psi)*[0 b3(3) -b3(2);-b3(3) 0 b3(1); b3(2), -b3(1) 0];
% Rib = Rpsib3*Rthetan*Rphii3;
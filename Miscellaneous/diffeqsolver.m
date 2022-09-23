function eulerangles(phi, theta,psi)
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
    -cos(phi)*sin(psi)-sin(phi)*cos(theta)*cos(psi), sin(phi)*sin(psi)+cos(phi)*cos(theta)*cos(psi), sin(theta)*cos(psi);
    sin(phi)*sin(theta), -cos(phi)*sin(theta), cos(theta)];

%% Inertial Frame
i1 = [1 0 0]'; i2 = [0 1 0]'; i3 = [0 0 1]';
%% Body Fixed Frame
b1 = Tib*i1;
b2 = Tib*i2;
b3 = Tib*i3;
%% Plot
plot3([0 i1(1)],[0 i1(2)],[0 i1(3)]);

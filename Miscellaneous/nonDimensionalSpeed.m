%%Boomerang
clear all; close all; clc;
%% Objective:
%   Plot Customn Boomerang
%   
%
% input variables:
%   none
% output variables:
%   none
% functions called:
%   none
%
%
%% Psi

C = zeros(1001,1001);
X=.7:.01:1.5;
r= 0:.01:1;
R=1;
for m = 1:length(X)
for i =1:1001
    for j =1:1001
        if (i-501)^2+(j-501)^2<=(1001/2)^2
            C(i,j) = (i-501)/sqrt((i-501)^2+(j-501)^2)+X(m)*sqrt((i-501)^2+(j-501)^2)/1001*R;

        end
    end
end
figure(1); pcolor(linspace(-1,1,1001),linspace(-1,1,1001),C); shading interp; title(sprintf('Vn/V, X = %f',X(m)));
cv = colorbar;
%cv.Limits = [-1,2];
end

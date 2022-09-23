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
%   eulerangles
%
%
%% Plot Boomerang, Get Shape
figure(1); hold on;
axis([-.15 .15 -.15 .15]);
filename = 'tribladerfront.jpg';
y = imread(filename);
image([-.15 .15], [-.15 .15],y);
set(gca,'YDir','reverse');
h = imfreehand;
pos = h.getPosition
close all;
save('boomerangShape.mat','pos');

%% Load bommerang Shape
load('boomerangShape.mat');
%% Call Euler Angles
icangles(10*pi/180,10*pi/180,.01*pi/180,pos);
drawnow;
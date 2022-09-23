clc;
clear all;

%Boomerang Parameters (Reference was Kaiser)
R = (26)*(1/100); %Blade Length, (m)
lw = (18)*(1/100); %Wrist Length, (m)
le = (24)*(1/100); %Elbow Length
m = (50.6)*(1/1000);  %Mass of boomerang (kg)
%lam = linspace(0, 180, 181); %Joint angles 
lam = 75;


%Calculations 

d = sqrt((R - (R/2)*(cosd(lam/2)).^2).^2 + ((R/2).*cosd(lam/2).*sind(lam/2)).^2);

Icm = ((m*R^2)/3)*((sind(lam/2)).^2+((cosd(lam/2)).^2)/4);

Iw = Icm + m*(lw^2 + 2.*d.*lw + d.^2);

Iww = Iw - Icm; 

Ie = Icm + m*(lw^2 + 2*le*lw + 2.*d.*lw + 2.*d.*le + le^2 + d.^2);

Iee = Ie - Icm;

w0test = 10.45;

we = 1.5; %(rad/s)
ww = 2; %(rad/s)



w0 = (ww + we + (Iw/Icm)*ww + (Ie/Icm)*we)*(1/(2*pi)) 
 
%Non-Dimensionalized Values

Ide = Iee./Icm;

Idw = Iww./Icm;

%Plot 

%figure(01)
plot(lam, Iw, 'LineWidth', 2)
hold on 
plot(lam, Ie, 'LineWidth', 2)
hold on 
plot(lam, Iww, 'LineWidth', 2)
hold on 
plot(lam, Iee, 'LineWidth', 2)
hold on 
plot(lam, Icm, 'LineWidth', 2)
legend('Moment of Inertia - Wrist', 'Moment of Inertia - Elbow', 'Moment of Inertia -  // Wrist', 'Moment of Inertia - // Elbow', 'Moment of Inertia - Center of Mass', 'location', 'northwest');
xlabel('Joint Angle (Degrees)');
ylabel('Moment of Inertias (kg m^2)');
xlim([0 190]);
set(gca,'FontSize',15,'LineWidth',2); 

%figure(02)
plot(lam, Idw, 'LineWidth', 2)
hold on 
plot(lam, Ide, 'LineWidth', 2)
legend('Iw// divided by Icm', 'Ie// divided Icm', 'location', 'northwest');
xlabel('Joint Angle (Degrees)');
xlim([0 190]);
set(gca,'FontSize',15,'LineWidth',2); 

function [F,M] = boomForces(boomState,B)


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

FEASegs = 10;
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
% quiver3(0,0,0,R*blades(1,2),R*blades(2,2),R*blades(3,2),'r');
% quiver3(0,0,0,R*blades(1,3),R*blades(2,3),R*blades(3,3),'k');
% quiver3(0,0,0,b3(1),b3(2),b3(3),'k');

for i = 1:length(thetas)
    for j = 1:length(r)
        alphaFoil = atan2(Vmag*sin(alpha),(Vmag*cos(alpha)*cos(thetas(i))+boomState(12)*r(j)));
        VFoil = sqrt(Vmag^2*(sin(alpha)^2+cos(alpha)^2*cos(thetas(i))^2)+2*boomState(12)*r(j)*Vmag*cos(alpha)*cos(thetas(i))+boomState(12)^2*r(j)^2);
        [CL,CD] = Rhode32Coefs(alphaFoil);

        L = .5*B.rho*segLengths*B.C*CL*VFoil^2;
        %L = 0;
        D = .5*B.rho*segLengths*B.C*CD*VFoil^2;
        %D = 0;
        
        bcb3 = cross(blades(:,i),b3);
        Dhat = bcb3*cos(-alphaFoil) + cross(blades(:,i),bcb3)*sin(-alphaFoil)+blades(:,i)*dot(blades(:,i),bcb3)*(1-cos(-alphaFoil));
        Lhat = bcb3*cos(-alphaFoil-pi/2) + cross(blades(:,i),bcb3)*sin(-alphaFoil-pi/2)+blades(:,i)*dot(blades(:,i),bcb3)*(1-cos(-alphaFoil-pi/2));
        
        F = F + D*Dhat+L*Lhat;
        
        M = M + cross(blades(:,i)*r(j)-bcb3*B.Roffset,D*Dhat+L*Lhat);

%         quiver3(r(j)*blades(1,i),r(j)*blades(2,i),r(j)*blades(3,i),Lhat(1),Lhat(2),Lhat(3),'r');
%         quiver3(r(j)*blades(1,i),r(j)*blades(2,i),r(j)*blades(3,i),Dhat(1),Dhat(2),Dhat(3),'b');
        
%         quiver3(r(j)*blades(1,i),r(j)*blades(2,i),r(j)*blades(3,i),L*Lhat(1)+D*Dhat(1),L*Lhat(2)+D*Dhat(2),L*Lhat(3)+D*Dhat(3),'r');
    end
end

% quiver3(0,0,0,F(1)/sqrt(sum(F.^2)),F(2)/sqrt(sum(F.^2)),F(3)/sqrt(sum(F.^2)),'g');
% quiver3(0,0,0,M(1)/sqrt(sum(M.^2)),M(2)/sqrt(sum(M.^2)),M(3)/sqrt(sum(M.^2)),'r');
% quiver3(0,0,0,2*Vu(1),2*Vu(2),2*Vu(3));
% 
% %dot(F,b3)/(.5*B.rho*B.C*(R-Rmin)*Vmag^2)
% 
% dot(M,b3)

end

function T = Tbi(phi,theta,xi)

T = [cos(phi)*cos(xi)+sin(phi)*sin(theta)*sin(xi), cos(phi)*sin(xi)-sin(phi)*sin(theta)*cos(xi), -sin(phi)*cos(theta);
sin(phi)*cos(xi)-cos(phi)*sin(theta)*sin(xi), sin(phi)*sin(xi)+cos(phi)*sin(theta)*cos(xi), cos(theta)*cos(phi);
    cos(theta)*sin(xi), -cos(theta)*cos(xi), sin(theta)];

end
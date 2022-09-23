%3D Trajectory Main Script

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%NOTES BEFORE USING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%1:The precession moment, layover moment, and drag moment are Mx, My, and
%Mz in this code. The corresponding numerical integration functions are
%roll, pitch and yaw, respectively. Pitch is not here since we only use
%semi-porous lifting surface model which does not require numerical
%integration.

%2:Be careful with the AOI. It is the most important quantity in
%determining the trajectory. Whenever you see the trajectory going crazy,
%it is because the AOI is. This can be avoided by using nonlinear  lift
%theory, but thus far our models have not implemented that. That will be the subject of
%future work.

%3:Although this code right now finds the trajectory of a symmetric tri-blader, the code
%can actually be used for any boomerang as long as the numerical integration functions used to find the aerodynamic forces and
%moments are changed to that of the new boomerang.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%USER INPUTS (initial conditions desired by user)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%steps
s = 2000; %number of iterations for main loop (needs to be high (in thousands) or else lift won't be normal to velocity!)
s2 = 50; %number of steps for functions using numerical integration outside this function (the aerodynamic forces and moments)

%Time of Flight
T = 4; %in seconds (s)

%Aerodynamic constants (p is indicates the leading edge flow region, and m indicates the trailing edge flow region)
Clo_p = 0.39; %always positive
Clo_m = -0.39; %always negative
Cla_p = 4.5; %always positive (rad^-1)
Cla_m = 4.5; %always positive (rad^-1)
Cdo_p = 0.05; %always positive 
Cdo_m = 0.05; %always positive
Cda_p = 0.1; %always positive (rad^-1)
Cda_m = 0.1; %always positive (rad^-1)
C_My = 0.5; %constant for My using semi-porous lifting surface model (should be <1 (around 0.4 but not 100% sure) from simulations)

%set physical characteristics
rho = 1.225; %density of air (kg/m^3)
g = 9.8; %gravitational acceleration (m/s^2)
m = .09779; %mass (kg)
R = .208; %length of blade (m)
C = .04; %chord of blade (m)
C_I = 1/1.8; %coefficient of mom of inertia
I = .001479488;%C_I*m*R^2; %moment of inertia about x3 axis (spin axis)
A = .0249;%3*R*C; %area of boomn (m^2)

%set initial conditions for motion
V_o = 17; %initial translational velocity (m/s)
w_o = 20*2*pi; %initial rotational velocity (rad/s)
Z_o = 1.5; %initial throwing height (m)

%set init throwing angles (automatically converted to radians because degrees*pi/180=radians)
init_horiz = 9*pi/180; %initial angle to horizontal (rad)
init_layover = 30*pi/180; %initial layover angle (in frame of thrower looking up at horizontal angle) (rad)
AOI_o = 0.01*pi/180; %initial angle of incidence (rad) -> CAN'T make this zero because it will cause an infinity in the initial condition for e_phi


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SIMULATION (no user inputs needed)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%allocate variables
dt = T/s; %time differential
t = dt:dt:T; %time vector for graphing paramters as fn of time

V = 1:s; %norm of velocity
w = 1:s; %w is not constant due to drag
Vx = 1:s;
Vy= 1:s;
Vz= 1:s;
L = 1:s;
D = 1:s;
Mx = 1:s;
My = 1:s;
Mz = 1:s;
Vx_dot = 1:s;
Vy_dot = 1:s;
Vz_dot = 1:s;
V_dot = 1:s; %velocity and acceleration dot prod. Gives rate of change of total velocity

X = 1:s;
Y = 1:s;
Z = 1:s;

e_phi = 1:s; %precession euler angle
e_theta = 1:s; %layover euler angle
e_xi = 1:s; %alignment euler angle
e_phi_dot = 1:s; %pressecion euler angle derivative
e_theta_dot = 1:s; %layover euler angle derivative

a = 1:s; %angle of incidence

%INITIAL CONDITIONS
e_theta(1) = asin(-sin(init_horiz)*sin(AOI_o) + cos(init_horiz)*sin(init_layover)*cos(AOI_o));
e_phi(1) = AOI_o/abs(AOI_o) * acos(cos(init_layover)*cos(AOI_o)/cos(e_theta(1))); %the AOI_o/abs(AOI_o)is just to make sure e_phi is the same sign as AOI_o. Make sure AOI_o is not zero! Or there will be an infinity.
e_xi(1) = asin((sin(e_phi(1))*sin(e_theta(1))*cos(init_horiz) + cos(e_theta(1))*sin(init_horiz))/cos(AOI_o));
V(1) = V_o;
w(1) = w_o;
a(1) = AOI_o;
Vx(1) = V(1)*cos(init_horiz);
Vy(1) = 0;
Vz(1) = V(1)*sin(init_horiz);
Vx_dot(1) = 0;
Vy_dot(1) = 0;
Vz_dot(1) = 0;
V_dot(1) = 0;
X(1) = 0;
Y(1) = 0;
Z(1) = Z_o; %in meters
L(1) = lift(R,w(1),C,V(1),rho,a(1),Clo_p,Clo_m,Cla_p,Cla_m,s2);
D(1) = drag(R,w(1),C,V(1),rho,a(1),Cdo_p,Cdo_m,Cda_p,Cda_m,s2);
Mx(1) = roll(R,w(1),C,V(1),rho,a(1),Clo_p,Clo_m,Cla_p,Cla_m,s2);
My(1) = C_My*1/2*rho*A*V(1)^2*a(1)*R/2; %semi porous lifting surface model. C_My is an experimental constant
Mz(1) = yaw(R,w(1),C,V(1),rho,a(1),Cdo_p,Cdo_m,Cda_p,Cda_m,s2);

%main loop
for n=2:s
    %first find the new Euler Angles
    e_xi(n) = asin(1/V(n-1)*(Vx(n-1)*sin(e_phi(n-1))*sin(e_theta(n-1)) - Vy(n-1)*cos(e_phi(n-1))*sin(e_theta(n-1)) + Vz(n-1)*cos(e_theta(n-1))));

    e_phi_dot(n-1) = 1/(I*w(n-1)*cos(e_theta(n-1)))*(My(n-1)*sin(e_xi(n-1)) + Mx(n-1)*cos(e_xi(n-1)));
    e_phi(n) = e_phi(n-1) + e_phi_dot(n-1)*dt;

    e_theta_dot(n-1) = 1/(I*w(n-1))*(My(n-1)*cos(e_xi(n-1)) - Mx(n-1)*sin(e_xi(n-1)));
    e_theta(n) = e_theta(n-1) + e_theta_dot(n-1)*dt;
    
    %[e_phi(n),e_theta(n)] = e_solve(e_phi(n-1),e_theta(n-1),e_xi(n-1),Mx(n-1),My(n-1),I,w(n-1),dt);
    
    %find new velocities and w
    Vx_dot(n) = L(n-1)/m*(-sin(e_phi(n))*cos(e_theta(n)) + sin(a(n-1))*Vx(n-1)/V(n-1))/cos(a(n-1)) - D(n-1)/m*Vx(n-1)/V(n-1);
    Vy_dot(n) = L(n-1)/m*(cos(e_phi(n))*cos(e_theta(n)) + sin(a(n-1))*Vy(n-1)/V(n-1))/cos(a(n-1)) - D(n-1)/m*Vy(n-1)/V(n-1);
    Vz_dot(n) = L(n-1)/m*(sin(e_theta(n)) + sin(a(n-1))*Vz(n-1)/V(n-1))/cos(a(n-1)) - D(n-1)/m*Vz(n-1)/V(n-1) - g;
    Vx(n) = Vx(n-1) + Vx_dot(n)*dt;
    Vy(n) = Vy(n-1) + Vy_dot(n)*dt;
    Vz(n) = Vz(n-1) + Vz_dot(n)*dt;
    V(n) = (Vx(n)^2+Vy(n)^2+Vz(n)^2)^(1/2); %norm of velocity
    
    w_dot = -Mz(n-1)/I;
    w(n) = w(n-1) + w_dot*dt;

    %find new angle of incidence (or attack)
    a(n) = asin(1/V(n)*(Vx(n)*sin(e_phi(n))*cos(e_theta(n)) - Vy(n)*cos(e_phi(n))*cos(e_theta(n)) - Vz(n)*sin(e_theta(n)))); %does dot product since e_trans gives a column vector

    %find new lift, drag and torques
    L(n) = lift(R,w(n),C,V(n),rho,a(n),Clo_p,Clo_m,Cla_p,Cla_m,s2);
    D(n) = drag(R,w(n),C,V(n),rho,a(n),Cdo_p,Cdo_m,Cda_p,Cda_m,s2); 
    Mx(n) = roll(R,w(n),C,V(n),rho,a(n),Clo_p,Clo_m,Cla_p,Cla_m,s2);
    My(n) = C_My*1/2*rho*A*V(n)^2*5*pi/180*R/2; %semi porous lifting surface model. C_My is an experimental constant
    Mz(n) = yaw(R,w(n),C,V(n),rho,a(n),Cdo_p,Cdo_m,Cda_p,Cda_m,s2);

end

%now find positions to then plot final trajectory
for n=2:s
    if Z(n) > 0
        X(n) = X(n-1) + Vx(n-1)*dt;
        Y(n) = Y(n-1) + Vy(n-1)*dt;
        Z(n) = Z(n-1) + Vz(n-1)*dt;
    else
        break;
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ANALYTICAL POSITION IN XY PLANE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Here I give you the analytical XY trajectory. If you want to implement the
%analytical Euler angles, AOI, and Z position, then you can just refer to
%the paper and code them easily. However, this is the most important thing
%to find analytcially so I have done it for you. Also, I think this should
%be the only thing we should compare to experiment, honestly.

chi = R*w./V; %non dimensional rotational velocity
%Taylor expanded expressions from Prasad's paper (needed because we need
%basic and AOA forces and moments seperated
Lo = 1/2*rho*A*V.^2*Clo_p.*(1/2+chi.^2/3+(1-Clo_m/Clo_p)*(-1/4+chi./pi-chi.^2/7)); %basic lift
La = 1/2*rho*A*V.^2*Cla_p.*(chi./2+(1+Cla_m/Cla_p)*(1/pi-chi./4+chi.^2/(5*pi))); %AOA lift
Mxo = 1/2*rho*R*A*V.^2*Clo_p.*(chi./3+1/3*(1-Clo_m/Clo_p)*(1/pi-chi./2+3/(4*pi)*chi.^2)); %basic precession moment (rolling moment)
Mxa = 1/2*rho*R*A*V.^2*Cla_p.*(1/4-(1+Cla_m/Cla_p)*(1/8-chi./(3*pi))); %AOA precession moment (rolling moment)

AOI_eq = (Lo(1)/(m*V(1)) - Mxo(1)/(I*w(1)))/(Mxa(1)/(I*w(1)) - La(1)/(m*V(1))); %equilibrium AOI
w_f = Lo(1)/(m*V(1))+AOI_eq*La(1)/(m*V(1)); %fundamental frequency
t_r = 1/(La(1)/(m*V(1)) - Mxa(1)/(I*w(1))); %relaxation time
r_f = V(1)/w_f; %fundamental radius

%find analytical positions in XY plane
q = t_r*La(1)/(m*V(1))*(AOI_eq-AOI_o); %a constant to make the expression look nicer
mu_sum = 1:s; %the infinite sum part of mu, where mu is the complex position
mu_sum(1) = 0; %initialize
%find the infinite sum
for n=2:s
    mu_sum = mu_sum + (1i*q)^n/factorial(n)*(exp(1i*w_f-n/t_r)*t-1)/(1i*w_f-n/t_r);
end
mu = (r_f*(exp(1i*w_f*t)-1)/1i + V(1)*((1i*q)^1/factorial(1)*(exp((1i*w_f-1/t_r)*t)-1)/(1i*w_f-1/t_r) + (1i*q)^2/factorial(2)*(exp(1i*w_f-2/t_r)*t-1)/(1i*w_f-2/t_r) + (1i*q)^3/factorial(3)*(exp(1i*w_f-3/t_r)*t-1)/(1i*w_f-3/t_r)))*exp(-1i*q);
Xa = real(mu); %a is for analytical
Ya = imag(mu);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PLOTS AND MISCELLANEOUS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%plot final trajectory in XY plane
figure
plot(X,Y)
hold on
plot(Xa,Ya)
hold off
legend('simulation','analytical')
xlim([-10 20])
ylim([-10 20])
title("V="+V_o+"  w=2pi*"+w_o/(2*pi)+"  H="+init_horiz*180/pi+"  L="+init_layover*180/pi+"  a="+AOI_o*180/pi)
xlabel('X')
ylabel('Y')

%convert the huge (size 2000) analytcial and simulation X and Y vectors
%into size s3 vectors so you can graph them in excel easier
s3 = 40;
X_sim = 1:s3;
Y_sim = 1:s3;
X_anal = 1:s3;
Y_anal = 1:s3;
X_sim(1) = 0;
Y_sim(1) = 0;
X_anal(1) = 0;
Y_anal(1) = 0;
for n=2:s3
    X_sim(n) = X(round(n*s/s3));
    Y_sim(n) = Y(round(n*s/s3));
    X_anal(n) = X_anal(round(n*s/s3));
    Y_anal(n) = Y_anal(round(n*s/s3));
end

%use X_sim,Y_sim for the simulation X and Y
%use X_anal,Y_anal for the analytical X and Y
%graph these along with the experimentally measured XY trajectory


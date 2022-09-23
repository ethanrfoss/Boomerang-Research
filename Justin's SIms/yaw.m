function Mz = yaw(R, w, C, V, rho, a, Cdo_p, Cdo_m, Cda_p, Cda_m, s)
%calcs prasad's lift and torques using numerical integration
chi = R*w/V;

r = R/s:R/s:R;%starts at R/s then increments by step size R/s till R

dMzo_p = 1:s; %makes vector for dLo_p so we can integrate over r later
for dr = 1:s
    ang_psi = 0:real(pi-acos(r(dr)*chi/R))/s:real(pi-acos(r(dr)*chi/R)); %create values for psi range for trapz
    fMzo = (cos(ang_psi)+r(dr)*chi/R).^2.*r(dr); %function we integrate over (a vector since ang_psi is vector)
    dMzo_p(dr) = trapz(ang_psi,fMzo); 
end
Mzo_p = (1/pi)*trapz(r, dMzo_p);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we repeat the process for Lo_m

dMzo_m = 1:s;
for dr = 1:s
    ang_psi = real(pi-acos(r(dr)*chi/R)):real(pi-(pi-acos(r(dr)*chi/R)))/s:pi; %create values for psi
    fMzo = (cos(ang_psi)+r(dr)*chi/R).^2*-r(dr); %note negative sign!
    dMzo_m(dr) = trapz(ang_psi,fMzo); 
end
Mzo_m = (1/pi)*trapz(r, dMzo_m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate non-dimensional basic lift coeff
Cdo_ND = (Cdo_p*Mzo_p + Cdo_m*Mzo_m)/R; %divide by R since the constant multiplying Clo_ND in the lift equation contains an R which was taken out from the integration

Mzo = 1/2*R*rho*V^2*C*Cdo_ND;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we calculate the lift due to angle of attack
dMza_p = 1:s;
for dr = 1:s
    ang_psi = 0:real(pi-acos(r(dr)*chi/R))/s:real(pi-acos(r(dr)*chi/R)); %create values for psi
    fMza = abs(cos(ang_psi)+r(dr)*chi/R)*r(dr);
    dMza_p(dr) = trapz(ang_psi,fMza);
end
Mza_p = (1/pi)*trapz(r, dMza_p);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we repeat the process for La_m
dMza_m = 1:s;
for dr = 1:s
    ang_psi = real(pi-acos(r(dr)*chi/R)):real(pi-(pi-acos(r(dr)*chi/R)))/s:pi; %create values for psi
    fMza = abs(cos(ang_psi)+r(dr)*chi/R)*-r(dr); %note negative sign!
    dMza_m(dr) = trapz(ang_psi,fMza);
end
Mza_m = (1/pi)*trapz(r, dMza_m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate non-dimensional basic lift coeff
Cda_ND = (Cda_p*Mza_p + Cda_m*Mza_m)/R; %divide by R since the constant multiplying Cla_ND in the lift equation contains an R which was taken out from the integration

Mza = 1/2*R*rho*V^2*C*a*Cda_ND; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate total lift
Mz = 3*(Mzo + Mza); %for 3 blades so mult by 3. ALSO make negative since leading edge flow corr. to spin slow down.

end


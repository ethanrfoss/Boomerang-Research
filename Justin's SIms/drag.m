function D = drag(R, w, C, V, rho, a, Cdo_p, Cdo_m, Cda_p, Cda_m, s)
%calcs prasad's lift and torques using numerical integration
chi = R*w/V;

r = R/s:R/s:R;%starts at R/s then increments by step size R/s till R

dDo_p = 1:s; %makes vector for dLo_p so we can integrate over r later
for dr = 1:s
    ang_psi = 0:real(pi-acos(r(dr)*chi/R))/s:real(pi-acos(r(dr)*chi/R)); %create values for psi range for trapz
    fDo = (cos(ang_psi)+r(dr)*chi/R).^2.*cos(ang_psi); %function we integrate over (a vector since ang_psi is vector)
    dDo_p(dr) = trapz(ang_psi,fDo); %if Clo is constant over the whole blade
end
Do_p = (1/pi)*trapz(r, dDo_p);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we repeat the process for Lo_m

dDo_m = 1:s;
for dr = 1:s
    ang_psi = real(pi-acos(r(dr)*chi/R)):real(pi-(pi-acos(r(dr)*chi/R)))/s:pi; %create values for psi
    fDo = (cos(ang_psi)+r(dr)*chi/R).^2.*-cos(ang_psi); %note the negative sign in front of the cosine!! This is important.
    dDo_m(dr) = trapz(ang_psi,fDo); 
end
Do_m = (1/pi)*trapz(r, dDo_m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate non-dimensional basic lift coeff
Cdo_ND = (Cdo_p*Do_p + Cdo_m*Do_m)/R; %divide by R since the constant multiplying Clo_ND in the lift equation contains an R which was taken out from the integration

Do = 1/2*R*rho*V^2*C*Cdo_ND;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we calculate the lift due to angle of attack
dDa_p = 1:s;
for dr = 1:s
    ang_psi = 0:real(pi-acos(r(dr)*chi/R))/s:real(pi-acos(r(dr)*chi/R)); %create values for psi
    fDa = abs(cos(ang_psi)+r(dr)*chi/R).*cos(ang_psi);
    dDa_p(dr) = trapz(ang_psi,fDa);
end
Da_p = (1/pi)*trapz(r, dDa_p);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we repeat the process for La_m
dDa_m = 1:s;
for dr = 1:s
    ang_psi = real(pi-acos(r(dr)*chi/R)):real(pi-(pi-acos(r(dr)*chi/R)))/s:pi; %create values for psi
    fDa = abs(cos(ang_psi)+r(dr)*chi/R).*cos(ang_psi); 
    dDa_m(dr) = trapz(ang_psi,fDa);
end
Da_m = (1/pi)*trapz(r, dDa_m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate non-dimensional basic lift coeff
Cda_ND = (Cda_p*Da_p + Cda_m*Da_m)/R; %divide by R since the constant multiplying Cla_ND in the lift equation contains an R which was taken out from the integration

Da = 1/2*R*rho*V^2*C*a*Cda_ND; %where a is the angle of attack (alpha) and it has abs value since drag always points backwards

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate total lift
D = 3*(Do + Da); %for 3 blades so mult by 3

end


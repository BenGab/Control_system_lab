%%Plant
time =  1:0.001:200; 
A = 1; T1=12.4; T2=5; T3 = 1.3;
num_plant = A;
den_plant = conv([T1 1], conv([T2 1],[T3 1]));
W_PLANT = tf(num_plant, den_plant)
figure('Name','Step response for W_PLANT');
step(W_PLANT)
figure('Name', 'Bode diagram for w_PLANT');
margin(W_PLANT)
%%Classic P controller for Phase margin = 70, numerical
num_p1 = 1;
den_p1 = 1;
Wp_1 = tf(num_p1, den_p1);
Wo_1 = series(Wp_1,W_PLANT);
[mag_p1,phase_p1,w_p1] = bode(Wo_1);
phase_margin_wp = 70;
phi_wp_p1 = abs((180+phase_p1)-phase_margin_wp);

[min_value_p1, phi_index_p1] = min(phi_wp_p1);
amp_p1 = mag_p1(phi_index_p1);
phase_p1(phi_index_p1);
w_p1(phi_index_p1);
AP_numerical_p1 = 1/amp_p1;

num_wpv2 = AP_numerical_p1;
den_wpv2 = 1;
Wp_v2 = tf(num_wpv2,den_wpv2);
WOp_v2 = series(Wp_v2, W_PLANT);
figure('Name','Bode for open loop with pid');
margin(WOp_v2)

WcP_v2 = feedback(WOp_v2, tf(1,1));
figure('Name', 'Step response for closed loop ');
step(WcP_v2,time)

%P controller for error signal
error_p = 0.1;
K_error = (1-error_p)/error_p;
K_error_plant = dcgain(W_PLANT);
K_error_p = K_error/K_error_plant;

num_wc_error = K_error_p;
den_wc_error = 1;
W_error_p = tf(num_wc_error, den_wc_error);
WO_error = series(W_error_p, W_PLANT);
margin(WO_error)
W_Closed_error_p = feedback(WO_error, tf(1,1));
[y,t,x] = step(W_Closed_error_p,time);
error_check = 1 - y(end);
figure('Name', 'Step response for in the prescribed error');
step(W_Closed_error_p,time);
hold
oo = ones(1, length(time));
plot(t,1.1*oo,'r',t,0.9*oo,'r') 
hold off

%PD controller for cutting frequent
wc_freq = 0.8;
TD_p_pd = T2/1.1;
TC_p_pd = 0.1*TD_p_pd;
num_p_pd = [TD_p_pd+TC_p_pd 1];
den_p_pd = [TC_p_pd 1];
W_p_pd = tf(num_p_pd, den_p_pd)
WO_p_pd = series(W_p_pd, W_PLANT)

[mag, phase] = bode(WO_p_pd, wc_freq);
Ap_p_pd = 1/mag

num_pd = Ap_p_pd * num_p_pd;
den_pd = den_p_pd;
W_pd = tf(num_pd, den_pd);

WO_pd = series(W_pd, W_PLANT);
figure('name','Bode for PD controller');
subplot(2,1,1)
margin(WO_pd)
subplot(2,1,2)
step(WO_pd,time)
W_CL_PD = feedback(WO_pd, tf(1,1));
figure('name', 'Bode and step response for closed loop')
subplot(2,1,1)
margin(W_CL_PD)
subplot(2,1,2)
step(W_CL_PD,time)

%PI controller for phase margin
pm_pi = 70;
Ti =  T1;
num_p_pi = [Ti 1];
den_p_pi = [1 0];
W_p_pi = tf(num_p_pi, den_p_pi);

WO_p_pi = series(W_p_pi, W_PLANT)

[mag_p_pi,phase_p_pi,w_p_pi] = bode(WO_p_pi);
phi = abs((180+phase_p_pi)-pm_pi);
[min_value_pi, phi_index_pi] = min(phi)
Ap_pi = Ti/mag_p_pi(phi_index_pi)

num_pi = [Ap_pi*Ti Ap_pi * 1];
den_pi = [Ti 0];
W_pi = tf(num_pi, den_pi);
WO_pi = series(W_pi, W_PLANT)
figure('name', 'Bode and step response PI controller for open loop');
margin(WO_pi);

WC_PI = feedback(WO_pi, tf(1,1));
figure('name', 'Step response PI closed loop');
step(WC_PI, time)

%PID controller for phase margin
tau1 = T1;
tau2 = T2;

Td_PID = min(roots([0.11 -1.1*(tau1+tau2) tau1*tau2])); 

if Td_PID < 0
    Td_PID = max(roots([0.11 -1.1*(tau1+tau2) tau1*tau2])); 
end

Ti_PID =  tau1+tau2-(0.1*Td_PID)
num_p_pid = conv([tau1 1],[tau2 1]); 
den_p_pid = [0.1*Td_PID 1 0];
W_p_pid = tf(num_p_pid, den_p_pid);
WO_p_pid = minreal(series(W_p_pid, W_PLANT))

[mag_pid,phase_pid,w_pid] = bode(WO_p_pid);
phi_pid = abs((180+phase_pid)-pm_pi);
[min_value_pid, phi_index_pid] = min(phi_pid);
Ap_pid = Ti_PID/mag_pid(phi_index_pid)
Tc_pid = 0.1*Td_PID
num_pid = Ap_pid/Ti_PID * conv([tau1 1],[tau2 1]); 
den_pid = [0.1*Td_PID 1 0];
W_PID = tf(num_pid, den_pid);
WO_PID = minreal(series(W_PID, W_PLANT))
figure('name', 'bode and step response for ppen loop PID')
margin(WO_PID)
WCCP_PID = feedback(WO_PID, tf(1,1));
figure('name', 'step response for closed PID loop')
step(WCCP_PID, time)
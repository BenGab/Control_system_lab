%%Plant
time =  1:0.001:200; 
num_plant = 1;
den_plant = conv([12.4 1], conv([5 1],[1.3 1]));
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
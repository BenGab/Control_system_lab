%%Plant
time =  0:2:100 
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
num_plant = 1;
den_plant = conv([12.4 1], conv([5 1], [1.3 1]));
W_PLANT = tf(num_plant, den_plant)

T1 = 12.4;
T2 = 5;
Tsum = 1.3;
Kp = 1;
Kr = 1/(2*Kp*Tsum);
s = tf('s');
WPID = (Kr/s)*(1+(s*T1)*(1+(s*T2)));
x_pid = (1+(s*T1)) * (1+(s*T2))

P = minreal((Kr/s)*17.4*s)
I = minreal((Kr/s)*1)
D = minreal((Kr/s)*62*s^2)
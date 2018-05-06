TU = 4;
KP = 4;
TR = 12;
TI = 2 * TU;
TD = TU;
PARAM_PID = 1.2;
RHO = TU/TR;

AP = PARAM_PID/(KP *RHO);
s = tf('s');

WPID = AP * (1 + (1/(s*TI)) + (s*TD));

num_PID = [28.8 7.2 0.9];
den_PID = [8 0];

WSYSPID = minreal(tf(num_PID, den_PID))
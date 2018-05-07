num_plant = 1;
den_plant = conv([12.4 1], conv([5 1],[1.3 1]));
W_PLANT = tf(num_plant, den_plant);
Tu = 3;
Ku = 5.167;
Pu = 26.174;
Ap = 0.45 * Ku;
Ti = 0.85 * Pu;

s = tf('s');
WPI = Ap *(1 + (1/(s*Ti)))
num_PI = [51.73 2.325];
den_PI = [22.25 0];
WPIv1 = tf(num_PI, den_PI)

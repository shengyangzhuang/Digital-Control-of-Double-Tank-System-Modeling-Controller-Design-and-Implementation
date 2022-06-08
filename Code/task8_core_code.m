G = c2d(F,Ts,'ZOH');
[A_discretized,B_discretized,C_discretized,D_discretized] = tf2ss(G.num{1},G.den{1});
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hybrid and Embedded control systems
% Homework 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%clear
init_tanks;
g = 9.82;
Tau = 1/alpha1*sqrt(2*tank_h10/g);
tau=Tau;
k_tank = 60*beta*Tau;
k=k_tank;
gamma_tank = alpha1^2/alpha2^2;
gamma=gamma_tank;
uss = alpha2/beta*sqrt(2*g*tank_init_h2)*100/15; % steady state input
yss = 40; % steady state output

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uppertank = tf(k,[tau,1]); % Transfer function for upper tank
lowertank = tf(gamma,[gamma*tau,1]); % Transfer function for upper tank
sys = uppertank*lowertank; % Transfer function from input to lower tank level

% Calculate PID parameters
chi=0.5;
zeta=0.8;
omega0=0.2;
[K, Ti, Td, N] = polePlacePID(chi, omega0, zeta,tau,gamma,k);
F = K*(tf(1)+tf(1,[Ti,0])+tf([Td*N,0],[1,N]));

% plot(aa1(:,1),aa1(:,2),'b','LineWidth',2)
% grid on
% hold on
% plot(aa2(:,1),aa2(:,2),'r','LineWidth',2)
% plot(aa(:,1),aa(:,2),'g','LineWidth',2)
% plot(aa3(:,1),aa3(:,2),'k','LineWidth',2)
% plot(aa4(:,1),aa4(:,2),'c','LineWidth',2)
% plot(aa5(:,1),aa5(:,2),'m','LineWidth',2)
% set(gca,'FontSize',20);
% legend('no ZOH','Ts=0.5','Ts=0.7','Ts=1.0','Ts=2.0','Ts=5.0')
% title('\chi=0.5,\zeta=0.8,\omega0=0.2')
% 
% legend('without a ZOH','with a ZOH')
% title('\chi=0.5,\zeta=0.8,\omega0=0.2')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discretize the continous controller, save it in state space form
%sysd = c2d(F,Ts,'ZOH');
%[A_discretized,B_discretized,C_discretized,D_discretized] = idssdata(sysd);
Ts1 = 4; % Sampling time
G = c2d(F,Ts1,'ZOH');
[A_discretized1,B_discretized1,C_discretized1,D_discretized1] = tf2ss(G.num{1},G.den{1});

Ts2 = 0.05; % Sampling time
G = c2d(F,Ts2,'ZOH');
[A_discretized2,B_discretized2,C_discretized2,D_discretized2] = tf2ss(G.num{1},G.den{1});

Ts3 = 0.10; % Sampling time
G = c2d(F,Ts3,'ZOH');
[A_discretized3,B_discretized3,C_discretized3,D_discretized3] = tf2ss(G.num{1},G.den{1});

Ts4 = 0.14; % Sampling time
G = c2d(F,Ts4,'ZOH');
[A_discretized4,B_discretized4,C_discretized4,D_discretized4] = tf2ss(G.num{1},G.den{1});

Ts5 = 0.20; % Sampling time
G = c2d(F,Ts5,'ZOH');
[A_discretized5,B_discretized5,C_discretized5,D_discretized5] = tf2ss(G.num{1},G.den{1});

%Ts=1.0;

 plot(aa6(:,1),aa6(:,2),'b','LineWidth',2)
 grid on
% hold on
% plot(aa7(:,1),aa7(:,2),'r','LineWidth',2)
% plot(aa8(:,1),aa8(:,2),'g','LineWidth',2)
% plot(aa9(:,1),aa9(:,2),'k','LineWidth',2)
% plot(aa10(:,1),aa10(:,2),'c','LineWidth',2)
 set(gca,'FontSize',20);
% legend('Ts=0.01','Ts=0.05','Ts=0.10','Ts=0.14','Ts=0.20')
 title('\chi=0.5,\zeta=0.8,\omega0=0.2')

legend('Ts=4s')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Discretize the continous state space system, save it in state space form
% [Phi,Gamma,C,D] = 

% Observability and reachability
Wc = 1;
Wo = 1;

% State feedback controller gain
L = 1;
% observer gain
K = 1;
% reference gain
lr = 1;

% augmented system matrices
Aa = 1;
Ba = 1;

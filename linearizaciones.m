clear all
% Sistemas de Control Contínuo
% Ingeniería Electrónica - Universidad de Antioquia - 20232

%Thanks to MathWorks Inc.
% Bellow are some of the model parameters regarding dynamic behaviour
g = 9.80665;         % gravity constant
m_r = 0.2948;        % mass of the rod
m_w = 0.0695;        % mass of the inertia wheel
R = 0.05;            % radius of the inertia wheel
r = 0.02;            % cross section radius of the rod
l = 0.13;            % corresponding lengths
l_AD = l;
l_AC = l;            % assume wheel is mounted on the top of the pendulum
l_AB = l/2;
I_w_C = 0.5*m_w*R^2; % corresponding inertias
I_w_A = I_w_C + m_w*l_AC^2;
I_r_B = (1/12)*m_r*(3*r^2+l_AD^2);
I_r_A = I_r_B + m_r*l_AB^2;

Ts = 0.01;
% Define las variables simbólicas para los estados del sistema y las entradas
syms theta theta_dot omega
syms tau_m

% Define las ecuaciones de estado a partir de las variables simbolicas y de
% los parametros del modelo en modelParameters.m ahora estan vacios

%Las derivadas de los estados:
%x1 = theta_dot
%x2 = theta_ddot
%x3 = omega_dot

x1 = theta_dot;
x2 = (g * sin(theta) * (m_r * l_AB + m_w * l_AC) - tau_m) / ...
         (m_w * (0.5 * R^2 + l_AC^2) + (1/12) * m_r * (3 * r^2 + l_AD) + m_r * l_AB^2);
x3 = tau_m / I_w_C - theta_dot;

%La matriz F:
F = [x1; x2; x3];

% Calculando A y B
A_symbolic = jacobian(F, [theta, theta_dot, omega]);
B_symbolic = jacobian(F, tau_m);


% Evaluando en el punto de equilibrio
A_eval = simplify(subs(A_symbolic, [theta, theta_dot, omega, tau_m], [0, 0, 0, 0]));
B_eval = simplify(subs(B_symbolic, [theta, theta_dot, omega, tau_m], [0, 0, 0, 0]));

%%Ahora se tienen las matrices A y B del SS
A_eval=eval(A_eval)
B_eval=eval(B_eval)

% Define la matriz C y D para el SS
C = [1,0,0];            
D = [0];                   

% Crea el sistema en espacio de estados
sysLTI = ss(A_eval, B_eval, C, D);

% figure(1)
% step(sysLTI);
% title("Respuesta al impulso (theta)");

eig(A_eval);
%Se tiene 3 polos entonces se procede a construir el ubicador de polos

Mp = 0.1; %%Sobrenivel deseado
ts = 2; %Tiempo de establecimiento deseado

%Construyamos los polos deseados para un sistema de segundo orden
zita = sqrt((log(Mp)^2)/(pi^2 + (log(Mp)^2)))
wn = 4.6/(zita*ts)
den = [1 2*zita*wn wn^2]
polos = roots(den)

%Completemos los polos hasta el tercer orden
polos_des = [polos(1) polos(2) -70];

% Vector de k
k = acker(A_eval,B_eval,polos_des)
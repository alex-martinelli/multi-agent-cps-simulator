clc; clear all; close all;

% definisco le variabili simboliche di stato
% Le variabili di stato corrispondono alle posizioni e alle velocità
% rispetto x e y
syms x1 x2 x3 x4 real %x= (x1=px, x2=vx, x3=py, x4=vy)
x= [x1; x2; x3; x4];

% definisco le variabili simboliche per gli ingressi
% Ingressi usati per il controllo su x e y
syms ux_sym uy_sym real
u_sym= [ux_sym; uy_sym];


%% MODELLO AGENTE 1

% x_dot1= f*x + g*u
x_dot1= [x2                                                                 % x1' = x2 = vx velocità rispetto ad x
         (-0.35 * x2 - 0.08 * x2 * abs(x2) + (1 / 1.2) * ux_sym);           % x2' = prima equazione del sistema
         x4                                                                 % x3' = x4 = vy
         (-0.45*x4 -0.1*x4*abs(x4) + (1/1.2)*uy_sym)];                      % x4' = seconda equazione del sistema

% y= (px, py)
y_sym1= [x1; x3];

% effettuo la FL per l'agente 1
[u1, phi1, B1]= io_fl_mimo(x, u_sym, y_sym1, x_dot1);                       % Funzione che generalizza il calcolo delle matrici linearizzate

% stampa dei risultati
disp("----- AGENT 1 -----")
disp('phi1: ');
disp(phi1);
disp('B1: ');
disp(B1);
disp('u1: ');
disp(u1);


%% MODELLO AGENTE 2

% x_dot2= f*x + g*u
x_dot2= [x2 %px_dot= vx
    (-0.4*x2 +(1/(1+0.25*(sin(x1)^2)))*ux_sym);
    x4
    (-0.55*x4 +(1/(1+0.3*(cos(x3)^2)))*uy_sym)];

% y= (px, py)
y_sym2= [x1; x3];

% effettuo la FL per l'agente 1
[u2, phi2, B2]= io_fl_mimo(x, u_sym, y_sym2, x_dot2);

% stampa dei risultati
disp("----- AGENT 2 -----")
disp('phi2: ');
disp(phi2);
disp('B2: ');
disp(B2);
disp('u2: ');
disp(u2);


%% MODELLO AGENTE 3

% x_dot3= f*x + g*u
x_dot3= [x2 %px_dot= vx
    (-0.3*x2 +0.15*sin(x3)*x4 + (1+0.2*(cos(x3)^2))*ux_sym +0.08*sin(x1)*uy_sym);
    x4
    (-0.35*x4 + 0.15*cos(x1)*x2 + 0.08*sin(x3)*ux_sym + (1+0.2*(sin(x1)^2))*uy_sym)];

% y= (px, py)
y_sym3= [x1; x3];

% effettuo la FL per l'agente 1
[u3, phi3, B3]= io_fl_mimo(x, u_sym, y_sym3, x_dot3);

% stampa dei risultati
disp("----- AGENT 3 -----")
disp('phi3: ');
disp(phi3);
disp('B3: ');
disp(B3);
disp('u3: ');
disp(u3);


%% ESPORTAZIONE DELLE FUNZIONI NUMERICHE PER LA SIMULAZIONE

% creo le variabili simboliche di nu per la simulazione
nu_sym= sym('nu', [2 1], 'real');

% creo le matlabFunction con le funzioni per le leggi di controllo corrispondenti
matlabFunction(u1, 'File', 'control_law_agent1', 'Vars', {x, nu_sym});
matlabFunction(u2, 'File', 'control_law_agent2', 'Vars', {x, nu_sym});
matlabFunction(u3, 'File', 'control_law_agent3', 'Vars', {x, nu_sym});





clc; clear all; close all;

% ==========================================================================
%  SISTEMA MULTIAGENTE CON CONTROLLORE DI FORMAZIONE
%  Topologia: catena diretta  1 → 2 → 3
%  Formazione: fila indiana   R1 ── R2 ── R3
%  Dinamica:   nonlineare + feedback linearization → doppio integratore
%  Controllo:  PD distribuito (Loop-Shaping) su grafo pinnato
% ==========================================================================

%% =========================================================================
%  1. MATRICI DEL SISTEMA LINEARIZZATO (doppio integratore 2D per agente)
% ==========================================================================
% Dopo la feedback linearization, ogni agente si comporta come:
% xi_dot = A0*xi + B0*nu_i
% con xi = [px; vx; py; vy]  e  nu_i = [nu_x; nu_y] (accelerazioni)

A0 = [0, 1, 0, 0;
      0, 0, 0, 0;
      0, 0, 0, 1;
      0, 0, 0, 0];

B0 = [0, 0;
      1, 0;
      0, 0;
      0, 1];

% sistema globale  X = [x1; x2; x3]  (12x1)
A_global = blkdiag(A0, A0, A0);   % 12x12
B_global = blkdiag(B0, B0, B0);   % 12x6

%% =========================================================================
%  2. TOPOLOGIA DI COMUNICAZIONE — catena diretta  1 → 2 → 3
% ==========================================================================
% Aadj(i,j) = 1  =>  agente i RICEVE dati dall'agente j
%
%   R1 è il leader (pinnato al riferimento)
%   R2 riceve solo da R1
%   R3 riceve solo da R2

Aadj = [0, 0, 0;
        1, 0, 0;
        0, 1, 0];

% Laplaciano in-degree della catena
L_chain = diag(sum(Aadj, 2)) - Aadj;

% Guadagno di pinning  (solo nodo 1)
gamma = 1.2;
Pin   = diag([1, 0, 0]);

% Laplaciano pinnato  (triangolare inferiore → autovalori = {gamma, 1, 1})
Lp      = L_chain + gamma * Pin;
rho     = eig(Lp);
rho_min = min(rho);

fprintf('=== TOPOLOGIA: catena  1→2→3 ===\n');
fprintf('Autovalori Lp: %s\n', num2str(rho'));
fprintf('rho_min = %.4f\n\n', rho_min);

% verifica connettività (autovalore nullo di L_chain deve essere uno solo)
eig_L = sort(eig(L_chain));
if abs(eig_L(1)) > 1e-10
    warning('Grafo non connesso!');
else
    fprintf('Connettività verificata.\n');
end

%% =========================================================================
%  3. OFFSET DI FORMAZIONE — triangolo equilatero (lato ell)
% ==========================================================================
% H(:,i) = offset desiderato dell'agente i nello spazio di stato [px;vx;py;vy]
% Le velocità di offset sono nulle (formazione con velocità relativa nulla)
%
%  Geometria (frame globale, R1 in punta, R2/R3 alla base):
%
%           R1  (punta, leader)
%         /    \
%       R2  ——  R3
%
%   R1  →  [  0;               0;  0;      0 ]
%   R2  →  [ -ell*sqrt(3)/2;   0; -ell/2;  0 ]   (base sinistra)
%   R3  →  [ -ell*sqrt(3)/2;   0; +ell/2;  0 ]   (base destra)

ell = 2;   % lato del triangolo equilatero [m]

H = [ [0;                  0;  0;       0], ...   % R1 – punta
      [-ell*sqrt(3)/2;     0; -ell/2;   0], ...   % R2 – base sinistra
      [-ell*sqrt(3)/2;     0; +ell/2;   0] ];     % R3 – base destra  (4x3)

%% =========================================================================
%  4.A DESIGN DEL CONTROLLORE STATE FEEDBACK
% ==========================================================================

% Partiamo con la definizione delle matrici Q e R che useremo in LQR
% Q -> 4x4 -> penalizza l'errore sugli stati (posizioni e velocità).
Q = diag([20, 1, 20, 1]);
% R -> 2x2 -> penalizza lo sforzo di controllo (accelerazioni virtuali). 
R = eye(2) * 0.5;

% Calcolo del guadagno K LQR per il singolo sistema nominale (A0, B0)
[K_lqr, P, E] = lqr(A0, B0, Q, R);

% Calcolo il coupling gain
% Per garantire la stabilità di Lyapunov della rete, c deve compensare
% la topologia peggiore (rho_min).
c_min = 1 / (2 * rho_min);
c_gain = c_min + 0.5;

% Matrice di quadagno 
K_pd = c_gain * K_lqr;

fprintf('=== CONTROLLORE STATE FEEDBACK (LQR) ===\n');
fprintf('Guadagno di accoppiamento c = %.4f\n', c_gain);
fprintf('Matrice K (2x4) applicata:\n');
disp(K_pd);

%% =========================================================================
%  4.B DESIGN DEL CONTROLLORE PD  (Loop-Shaping, 70° margine di fase)
% ==========================================================================
% Pulsazione di taglio desiderata nel caso peggiore (rho_min)
omega_c = 8;   % [rad/s]

Kp = omega_c^2        / (3.236 * rho_min);
Kd = omega_c * 0.951  /          rho_min;

% K_pd (2x4): mappa l'errore di stato (4D) nell'ingresso virtuale (2D)
%
%   nu_x = Kp*(px_err) + Kd*(vx_err)
%   nu_y = Kp*(py_err) + Kd*(vy_err)
%
% Ordine colonne coerente con xi = [px; vx; py; vy]
K_pd = [Kp, Kd,  0,  0;
         0,  0, Kp, Kd];

fprintf('=== CONTROLLORE PD ===\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Kd = %.4f\n\n', Kd);

%% =========================================================================
%  5. TRAIETTORIA DI RIFERIMENTO (per il leader, agente 1)
% ==========================================================================
Rc   = 3;      % raggio del cerchio  [m]
om_r = 0.4;    % velocità angolare   [rad/s]
rc   = [5; 5]; % centro del cerchio  [m]

refpos = @(t) rc + Rc * [cos(om_r*t); sin(om_r*t)];
refvel  = @(t) Rc * om_r * [-sin(om_r*t);  cos(om_r*t)];
refacc  = @(t) -Rc * om_r^2 * [cos(om_r*t); sin(om_r*t)];

%% =========================================================================
%  6. CONDIZIONI INIZIALI
% ==========================================================================
% Le condizioni iniziali possono essere lontane dalla formazione desiderata:
% il controllore convergerà sia sull'errore di formazione che sul tracking.

x1_0 = [rc(1)+Rc;  0;   rc(2);    0];   % R1 vicino al cerchio
x2_0 = [2;         0;  -1;         0];   % R2 casuale
x3_0 = [-1;        0;   3;         0];   % R3 casuale

X0 = [x1_0; x2_0; x3_0];   % stato globale iniziale 12x1

% --- INIZIALIZZAZIONE STRUTTURA PID (per compatibilità retroattiva) ---
is_pid_active = false; 
K_i = 0;
Z_sim = [];

%% =========================================================================
%  7.A SIMULAZIONE (SENZA DISTURBO)
% ==========================================================================
tspan = [0, 30];

% Funzione anonima: cattura tutti i parametri di formazione nel closure
multiagent_dyn = @(t, X) multi_agent_dynamics(t, X,        ...
                                               A_global,    ...
                                               B_global,    ...
                                               Aadj,        ...
                                               H,           ...
                                               K_pd,        ...
                                               gamma,       ...
                                               refpos,      ...
                                               refvel,      ...
                                               refacc);

fprintf('Simulazione ODE45 in corso...\n');
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_sim, X_sim] = ode45(multiagent_dyn, tspan, X0, opts);
fprintf('Completata. %d punti temporali.\n\n', length(t_sim));

%% =========================================================================
%  7.B SIMULAZIONE (CON INTRODUZIONE DI DISTURBO COSTANTE)
% ==========================================================================
tspan = [0, 30];

% --- DEFINIZIONE DEL DISTURBO AMBIENTALE COSTANTE ---
% Simuliamo un vento costante o una pendenza che spinge i robot.
% Assegniamo un'accelerazione di disturbo su X e su Y (in m/s^2)
d_x = 0.5;  
d_y = 0.3;  

% Creiamo un vettore di disturbo 12x1 da sommare alla derivata dello stato.
% Il disturbo agisce solo sulle derivate delle velocità (indici 2,4 per R1; 6,8 per R2; 10,12 per R3)
dist_vector = zeros(12, 1);
dist_vector([2, 6, 10]) = d_x;  % Disturbo sull'asse X per tutti e 3 i robot
dist_vector([4, 8, 12]) = d_y;  % Disturbo sull'asse Y per tutti e 3 i robot

% 1. Calcolo della dinamica nominale (il codice che avevi già prima)
multiagent_dyn_nominal = @(t, X) multi_agent_dynamics(t, X,        ...
                                               A_global,    ...
                                               B_global,    ...
                                               Aadj,        ...
                                               H,           ...
                                               K_pd,        ...
                                               gamma,       ...
                                               refpos,      ...
                                               refvel,      ...
                                               refacc);

% 2. CREAZIONE DEL WRAPPER: Intercettiamo la derivata nominale e aggiungiamo il disturbo reale
multiagent_dyn = @(t, X) multiagent_dyn_nominal(t, X) + dist_vector;

fprintf('Simulazione ODE45 in corso (CON DISTURBO COSTANTE)...\n');
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_sim, X_sim] = ode45(multiagent_dyn, tspan, X0, opts);
fprintf('Completata. %d punti temporali.\n\n', length(t_sim));

%% =========================================================================
%  7.C SIMULAZIONE (CON DISTURBO NON COSTANTE - SINUSOIDALE)
% ==========================================================================
tspan = [0, 30];

% --- PROPRIETÀ DEL DISTURBO SINUSOIDALE ---
A_x = 1.5;       % Ampiezza del disturbo su X (in m/s^2)
omega_x = 2.0;   % Pulsazione del disturbo su X (in rad/s) -> Nota: è minore di omega_c = 8

A_y = 1.0;       % Ampiezza del disturbo su Y (in m/s^2)
omega_y = 4.0;   % Pulsazione del disturbo su Y (in rad/s)

% Funzione anonima che genera il vettore di disturbo 12x1 variabile nel tempo
% Il disturbo entra solo sulle accelerazioni (indici 2, 4, 6, 8, 10, 12)
dist_func = @(t) [
    0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t); ... % Robot 1
    0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t); ... % Robot 2
    0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t)      % Robot 3
];

% 1. Calcolo della dinamica nominale
multiagent_dyn_nominal = @(t, X) multi_agent_dynamics(t, X,        ...
                                               A_global,    ...
                                               B_global,    ...
                                               Aadj,        ...
                                               H,           ...
                                               K_pd,        ...
                                               gamma,       ...
                                               refpos,      ...
                                               refvel,      ...
                                               refacc);

% 2. WRAPPER: Sommiamo il disturbo tempo-variante dist_func(t)
multiagent_dyn = @(t, X) multiagent_dyn_nominal(t, X) + dist_func(t);

fprintf('Simulazione ODE45 in corso (CON DISTURBO SINUSOIDALE)...\n');
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_sim, X_sim] = ode45(multiagent_dyn, tspan, X0, opts);
fprintf('Completata. %d punti temporali.\n\n', length(t_sim));

%% =========================================================================
%  7.D SIMULAZIONE (CON CONTROLLORE PID SUL LEADER & DISTURBO COSTANTE)
% ==========================================================================
tspan = [0, 30];
is_pid_active = true; % Attiviamo il flag per aggiornare il post-processing
K_i = 25;             % Guadagno dell'azione integrale (20-30 azzera l'errore rapidamente)

% --- DEFINIZIONE DEL DISTURBO AMBIENTALE COSTANTE (identico a 7.B) ---
d_x = 0.5;  
d_y = 0.3;  
dist_vector = zeros(12, 1);
dist_vector([2, 6, 10]) = d_x;  
dist_vector([4, 8, 12]) = d_y;  

% 1. Cattura della dinamica nominale (Loop-Shaping o LQR corrente)
multiagent_dyn_nominal = @(t, X) multi_agent_dynamics(t, X,        ...
                                               A_global,    ...
                                               B_global,    ...
                                               Aadj,        ...
                                               H,           ...
                                               K_pd,        ...
                                               gamma,       ...
                                               refpos,      ...
                                               refvel,      ...
                                               refacc);

% 2. CONDIZIONI INIZIALI AUMENTATE: aggiungiamo 2 zeri per l'integratore di R1
X0_augmented = [X0; 0; 0]; 

% 3. FUNZIONE WRAPPER PER L'INTEGRAZIONE DEL PID
multiagent_pid_dyn = @(t, X_aug) PID_wrapper(t, X_aug, multiagent_dyn_nominal, dist_vector, K_i, refpos);

fprintf('Simulazione ODE45 in corso (CON CONTROLLORE PID SUL LEADER)...\n');
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_sim, X_sim_aug] = ode45(multiagent_pid_dyn, tspan, X0_augmented, opts);

% 4. SEPARAZIONE DEGLI STATI: estraiamo lo stato originale a 12 dimensioni per i grafici
X_sim = X_sim_aug(:, 1:12); 
Z_sim = X_sim_aug(:, 13:14); % Salviamo la memoria dell'integratore per calcolare gli ingressi fisici

fprintf('Completata. %d punti temporali.\n\n', length(t_sim));

%% =========================================================================
%  7.E SIMULAZIONE (CON CONTROLLORE PID SUL LEADER & DISTURBO SINUSOIDALE)
% ==========================================================================
tspan = [0, 30];
is_pid_active = true; % Attiviamo il flag per il post-processing
K_i = 25;             % Guadagno integrale

% --- PROPRIETÀ DEL DISTURBO SINUSOIDALE (Identico a 7.C) ---
A_x = 1.5;       
omega_x = 2.0;   
A_y = 1.0;       
omega_y = 4.0;   

dist_func = @(t) [
    0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t); ... % Robot 1
    0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t); ... % Robot 2
    0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t)      % Robot 3
];

% 1. Cattura della dinamica nominale (Loop-Shaping)
multiagent_dyn_nominal = @(t, X) multi_agent_dynamics(t, X,        ...
                                               A_global,    ...
                                               B_global,    ...
                                               Aadj,        ...
                                               H,           ...
                                               K_pd,        ...
                                               gamma,       ...
                                               refpos,      ...
                                               refvel,      ...
                                               refacc);

% 2. CONDIZIONI INIZIALI AUMENTATE: 12 stati dei robot + 2 integratori per R1
X0_augmented = [X0; 0; 0]; 

% 3. FUNZIONE WRAPPER CON DISTURBO TEMPO-VARIANTE
multiagent_pid_sin_dyn = @(t, X_aug) PID_sinusoidal_wrapper(t, X_aug, multiagent_dyn_nominal, dist_func, K_i, refpos);

fprintf('Simulazione ODE45 in corso (CON CONTROLLORE PID & DISTURBO SINUSOIDALE)...\n');
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_sim, X_sim_aug] = ode45(multiagent_pid_sin_dyn, tspan, X0_augmented, opts);

% 4. SEPARAZIONE DEGLI STATI PER I GRAFICI AUTOMATICI
X_sim = X_sim_aug(:, 1:12); 
Z_sim = X_sim_aug(:, 13:14); % Memoria dell'integratore per gli ingressi fisici

fprintf('Completata. %d punti temporali.\n\n', length(t_sim));

%% =========================================================================
%  8. POST-PROCESSING: errori + ingressi fisici
% ==========================================================================
% ODE45 salva solo lo stato X. Qui ricalcoliamo a posteriori:
%   - errori di tracking e formazione
%   - nu_i   (ingresso virtuale del controllore PD)
%   - u_phys_i = control_law_agentX(xi, nu_i)  (ingresso fisico reale)

Nt      = length(t_sim);
n_agent = 4;   % dim stato singolo agente

% Errori
e_track = zeros(1, Nt);
e_form2 = zeros(1, Nt);
e_form3 = zeros(1, Nt);

% Ingressi fisici  ui = [ux; uy]  per ogni agente
U1 = zeros(2, Nt);   % u_phys agente 1
U2 = zeros(2, Nt);   % u_phys agente 2
U3 = zeros(2, Nt);   % u_phys agente 3

for k = 1:Nt
    tk  = t_sim(k);

    % stati
    x1k = X_sim(k, 1:4)';
    x2k = X_sim(k, 5:8)';
    x3k = X_sim(k, 9:12)';
    xvec = {x1k, x2k, x3k};

    % riferimento
    rp = refpos(tk);  rv = refvel(tk);  ra = refacc(tk);
    xr  = [rp(1); rv(1); rp(2); rv(2)];
    uff = [ra(1); ra(2)];

    % ---- errori ----
    e_track(k) = norm(xr - x1k);
    e_form2(k) = norm((x2k - x1k) - (H(:,2) - H(:,1)));
    e_form3(k) = norm((x3k - x1k) - (H(:,3) - H(:,1)));

    % ---- ricalcolo nu_i (stesso protocollo di multi_agent_dynamics) ----
    nu_cell = cell(3,1);
    for i = 1:3
        xi   = xvec{i};
        epsi = zeros(n_agent, 1);
        for j = 1:3
            if Aadj(i,j) > 0
                xj   = xvec{j};
                epsi = epsi + Aadj(i,j) * ((xj - xi) - (H(:,j) - H(:,i)));
            end
        end
        nu_form = K_pd * epsi;
        if i == 1
            nu_cell{i} = nu_form + gamma * K_pd * (xr - xi) + uff;
            % --- INNESTO AZIONE INTEGRALE NEL POST-PROCESSING ---
            if is_pid_active
                nu_cell{1}(1) = nu_cell{1}(1) - K_i * Z_sim(k, 1); % Asse X
                nu_cell{1}(2) = nu_cell{1}(2) - K_i * Z_sim(k, 2); % Asse Y
            end
            % ----------------------------------------------------
        else
            nu_cell{i} = nu_form + uff;
        end
    end

    % ---- ingressi fisici tramite FL ----
    U1(:,k) = control_law_agent1(x1k, nu_cell{1});
    U2(:,k) = control_law_agent2(x2k, nu_cell{2});
    U3(:,k) = control_law_agent3(x3k, nu_cell{3});
end

fprintf('Errore tracking finale  (R1):   %.4f m\n', e_track(end));
fprintf('Errore formazione finale (R2):  %.4f m\n', e_form2(end));
fprintf('Errore formazione finale (R3):  %.4f m\n', e_form3(end));

%% =========================================================================
%  9. GRAFICI
% ==========================================================================
colors = {[0.00 0.45 0.74],   % blu    - R1
          [0.85 0.33 0.10],   % arancio - R2
          [0.47 0.67 0.19]};  % verde   - R3

% =========================================================================
% FIGURA 1 — Posizioni dei 3 robot (traiettorie XY)
% =========================================================================
figure('Name', 'Fig 1 - Posizioni robot');
hold on; grid on; axis equal;

% cerchio di riferimento
th = linspace(0, 2*pi, 300);
plot(rc(1)+Rc*cos(th), rc(2)+Rc*sin(th), 'k--', 'LineWidth', 1.2, ...
    'DisplayName', 'Rif. circolare');

% traiettorie
plot(X_sim(:,1),  X_sim(:,3),  '-',  'Color', colors{1}, 'LineWidth', 2.0, 'DisplayName', 'R1 (leader)');
plot(X_sim(:,5),  X_sim(:,7),  '--', 'Color', colors{2}, 'LineWidth', 1.6, 'DisplayName', 'R2');
plot(X_sim(:,9),  X_sim(:,11), '-.', 'Color', colors{3}, 'LineWidth', 1.6, 'DisplayName', 'R3');

% punti di partenza (cerchio pieno)
plot(X0(1),  X0(3),  'o', 'Color', colors{1}, 'MarkerFaceColor', colors{1}, 'MarkerSize', 9, 'HandleVisibility', 'off');
plot(X0(5),  X0(7),  'o', 'Color', colors{2}, 'MarkerFaceColor', colors{2}, 'MarkerSize', 9, 'HandleVisibility', 'off');
plot(X0(9),  X0(11), 'o', 'Color', colors{3}, 'MarkerFaceColor', colors{3}, 'MarkerSize', 9, 'HandleVisibility', 'off');

% formazione finale (triangolo tra le ultime posizioni)
pf = [X_sim(end,1), X_sim(end,3);
      X_sim(end,5), X_sim(end,7);
      X_sim(end,9), X_sim(end,11)];
fill(pf(:,1), pf(:,2), [0.8 0.8 0.8], 'FaceAlpha', 0.2, ...
    'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'Form. finale');

xlabel('x [m]'); ylabel('y [m]');
title('Posizioni robot — Formazione triangolare (1→2→3)');
legend('Location', 'best');

% =========================================================================
% FIGURA 2 — Formation errors
% =========================================================================
figure('Name', 'Fig 2 - Formation errors');

subplot(3,1,1);
plot(t_sim, e_track, 'w', 'LineWidth', 1.8);
grid on; ylabel('||e_{track}|| [m]');
title('Tracking error — R1 vs riferimento');

subplot(3,1,2);
plot(t_sim, e_form2, 'Color', colors{2}, 'LineWidth', 1.6);
grid on; ylabel('||e_{form}|| [m]');
title('Formation error — R2');

subplot(3,1,3);
plot(t_sim, e_form3, 'Color', colors{3}, 'LineWidth', 1.6);
grid on; ylabel('||e_{form}|| [m]');
title('Formation error — R3');
xlabel('t [s]');

% =========================================================================
% FIGURA 3 — Ingressi fisici dei controllori  u = [ux; uy]
% =========================================================================
figure('Name', 'Fig 3 - Ingressi fisici');

agent_labels = {'R1 (leader)', 'R2', 'R3'};
U_all = {U1, U2, U3};

for i = 1:3
    Ui = U_all{i};

    subplot(3, 2, 2*i-1);
    plot(t_sim, Ui(1,:), 'Color', colors{i}, 'LineWidth', 1.5);
    grid on;
    ylabel('u_x [N]');
    title(sprintf('%s — ingresso X', agent_labels{i}));
    if i == 3, xlabel('t [s]'); end

    subplot(3, 2, 2*i);
    plot(t_sim, Ui(2,:), 'Color', colors{i}, 'LineWidth', 1.5, 'LineStyle', '--');
    grid on;
    ylabel('u_y [N]');
    title(sprintf('%s — ingresso Y', agent_labels{i}));
    if i == 3, xlabel('t [s]'); end
end



%% =========================================================================
%  FUNZIONI HELPER CHIAMATA PER IL PID
% ==========================================================================
function dX_aug = PID_wrapper(t, X_aug, nominal_dyn, dist_vector, K_i, refpos)
    % 1. Separiamo i 12 stati dei robot dai 2 stati dell'integratore
    X_robots = X_aug(1:12);
    z_integral = X_aug(13:14);
    
    % 2. Calcoliamo la derivata nominale (PD/LQR) + il disturbo reale
    dX_robots = nominal_dyn(t, X_robots) + dist_vector;
    
    % 3. Iniettiamo l'azione Correttiva Integrale (-Ki * z) nelle accelerazioni di R1
    dX_robots(2) = dX_robots(2) - K_i * z_integral(1); % Accelerazione X del Leader
    dX_robots(4) = dX_robots(4) - K_i * z_integral(2); % Accelerazione Y del Leader
    
    % 4. Dinamica dell'integratore (la derivata di z è l'errore di posizione attuale di R1)
    p_ref = refpos(t);
    dz = [X_robots(1) - p_ref(1);   % Errore X: posizione_reale - posizione_rif
          X_robots(3) - p_ref(2)];  % Errore Y: posizione_reale - posizione_rif
          
    % 5. Ricompattiamo il vettore derivato totale (14x1)
    dX_aug = [dX_robots; dz];
end

%% =========================================================================
%  FUNZIONI HELPER (In fondo al file principale)
% ==========================================================================
function dX_aug = PID_sinusoidal_wrapper(t, X_aug, nominal_dyn, dist_func, K_i, refpos)
    % 1. Separiamo i 12 stati dei robot dai 2 stati dell'integratore
    X_robots = X_aug(1:12);
    z_integral = X_aug(13:14);
    
    % 2. Dinamica nominale + Disturbo sinusoidale istantaneo dist_func(t)
    dX_robots = nominal_dyn(t, X_robots) + dist_func(t);
    
    % 3. Applichiamo il controllo integrativo sulle accelerazioni di R1
    dX_robots(2) = dX_robots(2) - K_i * z_integral(1); % Accel X del Leader
    dX_robots(4) = dX_robots(4) - K_i * z_integral(2); % Accel Y del Leader
    
    % 4. Dinamica dell'integratore (derivata di z = errore di posizione attuale)
    p_ref = refpos(t);
    dz = [X_robots(1) - p_ref(1);   
          X_robots(3) - p_ref(2)];  
          
    % 5. Ricompattazione del vettore (14x1)
    dX_aug = [dX_robots; dz];
end
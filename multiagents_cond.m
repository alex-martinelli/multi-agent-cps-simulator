clc; clear all; close all;

% ==========================================================================
%  TEST 1
%  SISTEMA MULTIAGENTE CON CONTROLLORE DI FORMAZIONE
%  Topologia: catena diretta  1 → 2 → 3
%  Formazione: fila indiana   R1 ── R2 ── R3
%  Dinamica:   nonlineare + feedback linearization → doppio integratore
%  Controllo:  PD distribuito (Loop-Shaping) su grafo pinnato
% ==========================================================================

% ==========================================================================
%  TEST 2
%  SISTEMA MULTIAGENTE CON CONTROLLORE DI FORMAZIONE
%  Topologia: catena diretta  1 → 2 <-> 3
%  Formazione: fila indiana   R1 ── R2 <-> R3
%  Dinamica:   nonlineare + feedback linearization → doppio integratore
%  Controllo:  PD distribuito (Loop-Shaping) su grafo pinnato
% ==========================================================================

%% =========================================================================
%  0. PANNELLO DI CONTROLLO PRINCIPALE
% ==========================================================================

%   STRUTTURA RETE DI COMUNICAZIONE
%     1 = TEST 1    rete: 1 -> 2 -> 3
%     2 = TEST 2    rete: 1 -> 2 <-> 3

STRUTTURA_RETE = 1;

%   INTEGRAZIONE BOUNDS
%     true -> Attivi
%     false -> Non attivi

ACTIVATE_BOUNDS = true;

%   STRATEGIA_CONTROLLO:
%     1 = PD
%     2 = PID (Azione Integrale sul Leader)
%     3 = State-Feedback protocol (Riccati + Observer)

STRATEGIA_CONTROLLO = 3;

%   SCENARIO_SIMULAZIONE:
%     1 = Nominale (Senza disturbi)
%     2 = Disturbo Costante (Vento/Pendenza)
%     3 = Disturbo Sinusoidale

SCENARIO_SIMULAZIONE = 2;

%   UTILIZZO RUMORE
USE_NOISE = false;

% PATH per la generazione delle imagini
output_dir = 'TEST/Senza_Rumore/Grafo2/PID/DisturboSinusoidale';

%% =========================================================================
%  1. MATRICI DEL SISTEMA LINEARIZZATO (doppio integratore)
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

C0 = [1, 0, 0, 0;  % L'output misurato è solo la posizione (Slide 2)
      0, 0, 1, 0]; 

A_global = blkdiag(A0, A0, A0);   
B_global = blkdiag(B0, B0, B0);   

%% =========================================================================
%  2. TOPOLOGIA DI COMUNICAZIONE E 3. OFFSET DI FORMAZIONE
% ==========================================================================
% Aadj(i,j) = 1  =>  agente i RICEVE dati dall'agente j
%
%   R1 è il leader (pinnato al riferimento)
%   R2 riceve solo da R1
%   R3 riceve solo da R2


switch STRUTTURA_RETE
    % TEST 1
    case 1
        Aadj = [0, 0, 0;
                1, 0, 0;
                0, 1, 0];
    % TEST 2
    case 2
        Aadj = [0, 0, 0;
                1, 0, 1;
                0, 1, 0];
end



% Laplaciano in-degree della catena
L_chain = diag(sum(Aadj, 2)) - Aadj;

% Guadagno di pinning  (solo nodo 1)
gamma = 1.2;
Pin   = diag([1, 0, 0]);

% Laplaciano pinnato  (triangolare inferiore → autovalori = {gamma, 1, 1})
Lp      = L_chain + gamma * Pin;
rho     = eig(Lp);
rho_min = min(eig(Lp));

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

% verifica connettività (autovalore nullo di L_chain deve essere uno solo)
ell = 2;

H = [ [0;                  0;  0;       0], ...   
      [-ell*sqrt(3)/2;     0; -ell/2;   0], ...   
      [-ell*sqrt(3)/2;     0; +ell/2;   0] ];     

%% =========================================================================
%  4. SINTESI DEL CONTROLLORE DISTRIBUITO
% ==========================================================================

switch STRATEGIA_CONTROLLO

    case 1
        % ------------------------------------------------------------------
        % STRATEGIA 1: LOOP-SHAPING PROTOCOL (PD)
        % ------------------------------------------------------------------
        fprintf('=== STRATEGIA ATTIVA: Loop-Shaping (PD) ===\n');
        omega_c = 4.5; % Condizione ottimale 
        % omega_c = 8; -> Prova con bounds troppo aggressiva

        Kp = omega_c^2       / (3.236 * rho_min);
        Kd = omega_c * 0.951 /          rho_min;

        K_pd = [Kp, Kd,  0,  0;
                 0,  0, Kp, Kd];

        fprintf('Kp = %.4f, Kd = %.4f\n\n', Kp, Kd);

        is_pid_active = false;
        K_i   = 0;
        L_obs = zeros(4, 2);

    case 2
        % ------------------------------------------------------------------
        % STRATEGIA 2: PID (PD + Azione Integrale sul Leader)
        % ------------------------------------------------------------------
        fprintf('=== STRATEGIA ATTIVA: PID ===\n');
        omega_c = 4.5; % Condizione ottimale
        % omega_c = 8;

        Kp = omega_c^2       / (3.236 * rho_min);
        Kd = omega_c * 0.951 /          rho_min;
        K_i = Kp / 10;   % Guadagno integrativo proporzionale a Kp


        K_pd = [Kp, Kd,  0,  0;
                 0,  0, Kp, Kd];

        

        fprintf('Kp = %.4f, Kd = %.4f, Ki = %.4f\n\n', Kp, Kd, K_i);

        is_pid_active = true;   % azione integrale sempre attiva
        L_obs = zeros(4, 2);

    case 3
        % ------------------------------------------------------------------
        % STRATEGIA 3: STATE-FEEDBACK (Riccati + Observer)
        % ------------------------------------------------------------------
        fprintf('=== STRATEGIA ATTIVA: State-Feedback (Riccati) ===\n');

        Q = diag([10, 1, 10, 1]);   % Penalizza: [pos_x, vel_x, pos_y, vel_y]
        R = eye(2) * 1;           % Penalizza lo sforzo (aumenta questo se saturi!)

        P     = care(A0, B0, Q, R);
        K_lqr = R \ (B0' * P);

        c_min  = 1 / (2 * rho_min);
        c_gain = c_min + 0.5;
        K_pd   = c_gain * K_lqr;

        fprintf('Guadagno di accoppiamento c = %.4f\n', c_gain);
        disp('Matrice P (Riccati):'); disp(P);
        disp('Matrice K (2x4):');     disp(K_pd);

        poli_desiderati = [-10, -11, -12, -13];
        L_obs = place(A0', C0', poli_desiderati)';

        disp('Matrice Observer Gain L (4x2):'); disp(L_obs);

        is_pid_active = false;
        K_i = 0;
end

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

%% =========================================================================
%  7. SIMULAZIONE DEL SISTEMA
% ==========================================================================
tspan = [0, 30];
%opts  = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
opts  = odeset('RelTol', 1e-3, 'AbsTol', 1e-4);

% --- Controllo di coerenza ---
if STRATEGIA_CONTROLLO == 3 && is_pid_active
    error('ERRORE: State-Feedback non e'' compatibile con azione integrale.');
end

% --- Condizioni iniziali ---
if is_pid_active
    X0_sim = [X0; 0; 0];   % stato aumentato con integratore 14x1
else
    X0_sim = X0;            % stato nominale 12x1
end

% --- Dinamica nominale ---
multiagent_dyn_nominal = @(t, X) multi_agent_dynamics(t, X, ...
    A_global, B_global, Aadj, H, K_pd, gamma, refpos, refvel, ...
    refacc, USE_NOISE, ACTIVATE_BOUNDS);

% --- Wrapper PID con integratore ---
pid_dyn_nominal = @(t, X_aug) PID_sinusoidal_wrapper( ...
    t, X_aug, multiagent_dyn_nominal, @(t) zeros(12,1), K_i, refpos);

switch SCENARIO_SIMULAZIONE

    case 1
        % ------------------------------------------------------------------
        % SCENARIO 1: NOMINALE (senza disturbi)
        % ------------------------------------------------------------------
        fprintf('Simulazione in corso: SCENARIO NOMINALE...\n');

        if is_pid_active
            % PID senza disturbo: usiamo il wrapper con dist_func = zero
            multiagent_dyn = pid_dyn_nominal;
        else
            multiagent_dyn = multiagent_dyn_nominal;
        end

        [t_sim, X_sim_raw] = ode45(multiagent_dyn, tspan, X0_sim, opts);

    case 2
        % ------------------------------------------------------------------
        % SCENARIO 2: DISTURBO COSTANTE
        % ------------------------------------------------------------------
        fprintf('Simulazione in corso: DISTURBO COSTANTE...\n');
        d_x = 0.5; d_y = 0.3;
        dist_vector = zeros(12, 1);
        dist_vector([2, 6, 10]) = d_x;
        dist_vector([4, 8, 12]) = d_y;

        if is_pid_active
            multiagent_dyn = @(t, X_aug) PID_wrapper( ...
                t, X_aug, multiagent_dyn_nominal, dist_vector, K_i, refpos);
        else
            multiagent_dyn = @(t, X) multiagent_dyn_nominal(t, X) + dist_vector;
        end

        [t_sim, X_sim_raw] = ode45(multiagent_dyn, tspan, X0_sim, opts);

    case 3
        % ------------------------------------------------------------------
        % SCENARIO 3: DISTURBO SINUSOIDALE
        % ------------------------------------------------------------------
        fprintf('Simulazione in corso: DISTURBO SINUSOIDALE...\n');
        A_x = 1.5; omega_x = 2.0;
        A_y = 1.0; omega_y = 4.0;
        dist_func = @(t) [
            0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t);
            0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t);
            0; A_x*sin(omega_x*t); 0; A_y*cos(omega_y*t)
        ];

        if is_pid_active
            multiagent_dyn = @(t, X_aug) PID_sinusoidal_wrapper( ...
                t, X_aug, multiagent_dyn_nominal, dist_func, K_i, refpos);
        else
            multiagent_dyn = @(t, X) multiagent_dyn_nominal(t, X) + dist_func(t);
        end

        [t_sim, X_sim_raw] = ode45(multiagent_dyn, tspan, X0_sim, opts);
end

% --- Estrazione stati ---
X_sim = X_sim_raw(:, 1:12);
if is_pid_active
    Z_sim = X_sim_raw(:, 13:14);
else
    Z_sim = [];
end

fprintf('Completata. %d punti temporali elaborati.\n\n', length(t_sim));

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
%  Le figure vengono salvate nella cartella 'risultati/'
% ==========================================================================

colors = {[0.00 0.45 0.74],   % blu    - R1
          [0.85 0.33 0.10],   % arancio - R2
          [0.47 0.67 0.19]};  % verde   - R3

% =========================================================================
% FIGURA 1 — Posizioni dei 3 robot (traiettorie XY)
% =========================================================================
fig1 = figure('Name', 'Fig 1 - Posizioni robot');
hold on; grid on; axis equal;

th = linspace(0, 2*pi, 300);
plot(rc(1)+Rc*cos(th), rc(2)+Rc*sin(th), 'k--', 'LineWidth', 1.2, ...
    'DisplayName', 'Rif. circolare');

plot(X_sim(:,1),  X_sim(:,3),  '-',  'Color', colors{1}, 'LineWidth', 2.0, 'DisplayName', 'R1 (leader)');
plot(X_sim(:,5),  X_sim(:,7),  '--', 'Color', colors{2}, 'LineWidth', 1.6, 'DisplayName', 'R2');
plot(X_sim(:,9),  X_sim(:,11), '-.', 'Color', colors{3}, 'LineWidth', 1.6, 'DisplayName', 'R3');

plot(X0(1),  X0(3),  'o', 'Color', colors{1}, 'MarkerFaceColor', colors{1}, 'MarkerSize', 9, 'HandleVisibility', 'off');
plot(X0(5),  X0(7),  'o', 'Color', colors{2}, 'MarkerFaceColor', colors{2}, 'MarkerSize', 9, 'HandleVisibility', 'off');
plot(X0(9),  X0(11), 'o', 'Color', colors{3}, 'MarkerFaceColor', colors{3}, 'MarkerSize', 9, 'HandleVisibility', 'off');

pf = [X_sim(end,1), X_sim(end,3);
      X_sim(end,5), X_sim(end,7);
      X_sim(end,9), X_sim(end,11)];
fill(pf(:,1), pf(:,2), [0.8 0.8 0.8], 'FaceAlpha', 0.2, ...
    'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'Form. finale');

xlabel('x [m]'); ylabel('y [m]');
title('Posizioni robot — Formazione triangolare (1→2→3)');
legend('Location', 'best');

%saveas(fig1, fullfile(output_dir, 'fig1_posizioni.png'));

% =========================================================================
% FIGURA 2 — Formation errors
% =========================================================================
fig2 = figure('Name', 'Fig 2 - Formation errors');

subplot(3,1,1);
plot(t_sim, e_track, 'Color', colors{1}, 'LineWidth', 1.8);
grid on; ylabel('||e_{track}|| [m]');
xlim([t_sim(1) t_sim(end)]);
title('Tracking error — R1 vs riferimento');

subplot(3,1,2);
plot(t_sim, e_form2, 'Color', colors{2}, 'LineWidth', 1.6);
grid on; ylabel('||e_{form}|| [m]');
xlim([t_sim(1) t_sim(end)]);
title('Formation error — R2');

subplot(3,1,3);
plot(t_sim, e_form3, 'Color', colors{3}, 'LineWidth', 1.6);
grid on; ylabel('||e_{form}|| [m]');
title('Formation error — R3');
xlim([t_sim(1) t_sim(end)]);
xlabel('t [s]');

%saveas(fig2, fullfile(output_dir, 'fig2_formation_errors.png'));

% =========================================================================
% FIGURA 3 — Ingressi fisici dei controllori  u = [ux; uy]
% =========================================================================
fig3 = figure('Name', 'Fig 3 - Ingressi fisici');

agent_labels = {'R1 (leader)', 'R2', 'R3'};
U_all = {U1, U2, U3};

for i = 1:3
    Ui = U_all{i};

    subplot(3, 2, 2*i-1);
    plot(t_sim, Ui(1,:), 'Color', colors{i}, 'LineWidth', 1.5);
    grid on;
    xlim([t_sim(1) t_sim(end)]);
    ylabel('u_x [N]');
    title(sprintf('%s — ingresso X', agent_labels{i}));
    if i == 3, xlabel('t [s]'); end

    subplot(3, 2, 2*i);
    plot(t_sim, Ui(2,:), 'Color', colors{i}, 'LineWidth', 1.5, 'LineStyle', '--');
    grid on;
    xlim([t_sim(1) t_sim(end)]);
    ylabel('u_y [N]');
    title(sprintf('%s — ingresso Y', agent_labels{i}));
    if i == 3, xlabel('t [s]'); end
end

%saveas(fig3, fullfile(output_dir, 'fig3_ingressi.png'));

fprintf('Figure salvate in: %s\n', output_dir);



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


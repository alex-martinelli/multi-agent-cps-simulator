% Funzione che implementa la dinamica del sistema multiagente
% con controllore di formazione PD (Loop-Shaping) integrato.
%
% Architettura del controllo (a due livelli):
%
%   LIVELLO ALTO — formazione (questo file):
%     Calcola il segnale virtuale  nu_i = [nu_x; nu_y]  per ogni agente
%     usando il protocollo PD distribuito su grafo.
%     nu_i comanda le ACCELERAZIONI del sistema già linearizzato.
%
%   LIVELLO BASSO — feedback linearization (control_law_agentX):
%     Riceve (xi, nu_i) e restituisce il comando fisico u_i
%     che fa sì che la dinamica nonlineare si comporti come
%     un doppio integratore:  px_ddot = nu_x,  py_ddot = nu_y.
%
%   SIMULAZIONE:
%     Poiché la FL è perfetta, integrare
%       X_dot = A_global*X + B_global*nu
%     è equivalente a integrare la dinamica nonlineare con u_i applicato.
%     u1_phys / u2_phys / u3_phys vengono calcolati ma non servono
%     nell'ODE; tornano utili se si vuole log del comando fisico.
%
% Firma:
%   multi_agent_dynamics(t, X, A_global, B_global, Aadj, H, K_pd, gamma,
%                        refpos, refvel, refacc, use_noise)
%
% Parametri aggiuntivi rispetto alla versione originale:
%   Aadj   — matrice di adiacenza (3x3), topologia catena 1→2→3
%   H      — offset di formazione nello spazio di stato (4x3)
%               H(:,i) = [px_off; 0; py_off; 0] per agente i
%   K_pd   — guadagno PD (2x4), mappa errore stato → nu
%               K_pd = [Kp Kd 0  0 ;
%                        0  0  Kp Kd]
%   gamma  — guadagno di pinning (solo agente 1, leader)
%   refpos — @(t) → [rx; ry]   posizione di riferimento
%   refvel — @(t) → [rvx; rvy] velocità  di riferimento
%   refacc — @(t) → [rax; ray] accelerazione di riferimento (feedforward)
%   use_noise   - rende condizionale il rumore nella funzione
%   activate_bounds - rende condizionale l'attivazione dei limiti agli attuatori

function X_dot_global = multi_agent_dynamics(t, X, A_global, B_global, ...
                                              Aadj, H, K_pd, gamma,    ...
                                              refpos, refvel, refacc, ...
                                              use_noise, activate_bounds)

    N = 3;   % numero di agenti
    n = 4;   % dimensione stato singolo agente  [px; vx; py; vy]

    % PARAMETRO DI RUMORE (Deviazione standard, es. 2 centimetri di errore)
    std_noise = 0.02;

    %% 1. Estrazione stati
    x1 = X(1:4);   % [px1; vx1; py1; vy1]
    x2 = X(5:8);
    x3 = X(9:12);
    xvec = {x1, x2, x3};

    %% 2. Riferimento per il leader (agente 1, pinnato)
    rp  = refpos(t);            % [rx; ry]
    rv  = refvel(t);            % [rvx; rvy]
    ra  = refacc(t);            % [rax; ray]  feedforward

    % stato di riferimento nello stesso formato degli agenti
    xr  = [rp(1); rv(1); rp(2); rv(2)];  % 4x1
    uff = [ra(1); ra(2)];                 % 2x1  ingresso virtuale di FF

    %% 3. Controllore PD distribuito  (nu_i per ogni agente)
    %
    % Protocollo di consenso con offset di formazione H:
    %
    %   eps_i = sum_j  Aadj(i,j) * [ (xj - xi) - (H(:,j) - H(:,i)) ]
    %         = errore di formazione distribuito dell'agente i
    %
    %   nu_i_form = K_pd * eps_i          (contributo formazione)
    %
    %   Per il leader (i=1, pinnato):
    %     nu_1 = nu_1_form + gamma * K_pd * (xr - x1) + uff
    %
    %   Per i follower (i=2,3):
    %     nu_i = nu_i_form + uff           (feedforward broadcast)

    nu_cell = cell(N, 1);

    % somma contributi dei vicini secondo la topologia Aadj
    for i = 1:N
        xi = xvec{i};

        % --- RUMORE SUL ROBOT i ---
        if use_noise
            rng(round(t * 1000) + i * 100);
            xi_noise = xi;
            xi_noise(1) = xi(1) + std_noise * randn();
            xi_noise(3) = xi(3) + std_noise * randn();
        else
            xi_noise = xi;   % nessun rumore
        end
        % --------------------------

        epsi = zeros(n, 1);

        for j = 1:N
            if Aadj(i, j) > 0
                xj = xvec{j};

                % --- RUMORE SUL VICINO j ---
                if use_noise
                    rng(round(t * 1000) + j * 100 + i * 10);
                    xj_noise = xj;
                    xj_noise(1) = xj(1) + std_noise * randn();
                    xj_noise(3) = xj(3) + std_noise * randn();
                else
                    xj_noise = xj;   % nessun rumore
                end
                % ---------------------------

                epsi = epsi + Aadj(i,j) * ((xj_noise - xi_noise) - (H(:,j) - H(:,i)));
            end
        end

        nu_form = K_pd * epsi;

        if i == 1
            nu_cell{i} = nu_form + gamma * K_pd * (xr - xi_noise) + uff;
        else
            nu_cell{i} = nu_form + uff;
        end
    end

    nu1 = nu_cell{1};
    nu2 = nu_cell{2};
    nu3 = nu_cell{3};

    %% --- STUDIO DEI BOUNDS DEGLI ATTUATORI ---
    
    if activate_bounds
        % Impostiamo un limite massimo all'accelerazione virtuale (es. 1.5 m/s^2)
        nu_max = 3; % Limite fisico perfetto  
    
        % Applichiamo la saturazione matematica: taglia i valori sopra nu_max e sotto -nu_max
        nu1 = max(min(nu1, nu_max), -nu_max);
        nu2 = max(min(nu2, nu_max), -nu_max);
        nu3 = max(min(nu3, nu_max), -nu_max);
        % -----------------------------------------
    end

    %% 4. Leggi di controllo fisiche
    %
    % control_law_agentX(xi, nu_i)  →  u_i fisico
    % (calcolate ma non necessarie nell'ODE se si simula il sistema lin.)
    u1_phys = control_law_agent1(x1, nu1);  %#ok<NASGU>
    u2_phys = control_law_agent2(x2, nu2);  %#ok<NASGU>
    u3_phys = control_law_agent3(x3, nu3);  %#ok<NASGU>

    %% 5. Dinamica globale
    %
    % X_dot = A_global * X + B_global * nu
    % Equivalente alla dinamica nonlineare con FL applicata.
    nu = [nu1; nu2; nu3];
    X_dot_global = A_global * X + B_global * nu;
end
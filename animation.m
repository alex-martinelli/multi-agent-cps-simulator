%% =========================================================================
%  10. ANIMAZIONE FORMAZIONE (Velocità 0.75x - Formazione Triangolo)
% ==========================================================================
N = 3; 
n = 4; 

figure('Name','Animazione Formazione','Color','k', ...
       'Position',[100 100 900 700]);
ax = axes('Color','k','XColor','w','YColor','w','GridColor',[0.3 0.3 0.3]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');

% Limiti asse
margin = 4;
xlim(ax, [rc(1)-Rc-margin, rc(1)+Rc+margin+ell*2]);
ylim(ax, [rc(2)-Rc-margin, rc(2)+Rc+margin+ell*2]);
xlabel(ax,'x [m]','Color','w','FontSize',12);
ylabel(ax,'y [m]','Color','w','FontSize',12);
title(ax,'Animazione: Topologia a Catena (1->2->3) | Fisica a Triangolo','Color','w','FontSize',14);

% Traccia il cerchio di riferimento (statico)
theta_ref = linspace(0, 2*pi, 300);
plot(ax, rc(1)+Rc*cos(theta_ref), rc(2)+Rc*sin(theta_ref), ...
     '--', 'Color',[0.5 0.5 0.5], 'LineWidth',1.2);

% Colori robot
robot_colors = {[0.0  0.45 0.74],   % blu
                [0.85 0.33 0.10],   % arancio
                [0.47 0.67 0.19]};  % verde

% Oggetti grafici per le tracce
trail_len = 300;   
h_trail = gobjects(N,1);
h_robot = gobjects(N,1);
h_tri   = plot(ax, NaN, NaN, 'w-', 'LineWidth', 2);   % connessione fisica (triangolo)
h_dot   = plot(ax, NaN, NaN, 'w+', 'MarkerSize',10,'LineWidth',2); % riferimento istantaneo

for i = 1:N
    h_trail(i) = plot(ax, NaN, NaN, '-', ...
                      'Color', [robot_colors{i}, 0.5], 'LineWidth', 1);
    h_robot(i) = plot(ax, NaN, NaN, 'o', ...
                      'Color', robot_colors{i}, ...
                      'MarkerFaceColor', robot_colors{i}, ...
                      'MarkerSize', 12);
end
h_time = text(ax, rc(1)-Rc-margin+0.5, rc(2)+Rc+margin-0.5, '', ...
              'Color','w','FontSize',12,'FontWeight','bold');
legend(ax, [h_trail; h_dot], ...
       {'Robot 1','Robot 2','Robot 3','Riferimento'}, ...
       'TextColor','w','Color',[0.15 0.15 0.15],'Location','northwest');

% ---- Setup Velocità Animazione ----
speed_factor = 0.1;          % Moltiplicatore di velocità desiderato
skip = 1;                     % Campioni saltati per frame (fluidità grafica)
dt_sim = t_sim(2) - t_sim(1); % Passo temporale reale della simulazione
pause_time = (dt_sim * skip) / speed_factor; % Calcolo automatico della pausa

fprintf('Avvio animazione a velocità %.2fx...\n', speed_factor);

% ---- Loop Animazione ----
Nt = length(t_sim);

for k = 1:skip:Nt
    % Indici dello stato di ogni robot
    px = zeros(N,1); py = zeros(N,1);
    for i = 1:N
        px(i) = X_sim(k, (i-1)*n+1);
        py(i) = X_sim(k, (i-1)*n+3);
    end
    
    % Aggiorna scie
    k0 = max(1, k-trail_len);
    for i = 1:N
        ix = (i-1)*n+1; iy = (i-1)*n+3;
        set(h_trail(i), 'XData', X_sim(k0:k, ix), 'YData', X_sim(k0:k, iy));
    end
    
    % Aggiorna il triangolo chiudendo il poligono (1 -> 2 -> 3 -> 1)
    tri_x_now = [px(1), px(2), px(3), px(1)];
    tri_y_now = [py(1), py(2), py(3), py(1)];
    set(h_tri, 'XData', tri_x_now, 'YData', tri_y_now);
    
    % Aggiorna posizioni robot
    for i = 1:N
        set(h_robot(i), 'XData', px(i), 'YData', py(i));
    end
    
    % Punto di riferimento istantaneo
    rp = refpos(t_sim(k));
    set(h_dot, 'XData', rp(1), 'YData', rp(2));
    
    % Tempo
    set(h_time, 'String', sprintf('t = %.2f s', t_sim(k)));
    
    drawnow limitrate;
    pause(pause_time);
end
fprintf('Animazione completata.\n');
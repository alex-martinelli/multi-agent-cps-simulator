% funzione che effettua la feedback linearization in un sistema MIMO
function [u, phi, B]=io_fl_mimo(x,u_sym,y,x_dot)

    % valori di ingresso:
    % - x=(px, vx, py. vy) variabile simbolica
    % - u_sym=(ux, uy) variabile simbolica
    % - y=(px, py) output del sistema
    % - x_dot= f*x+g*u = dinamica del sistema
    % valori di uscita:
    % - u= legge di controllo feedback linearizzante
    % - phi= termine di deriva (a in siso)
    % - B= termine di disaccoppiamento (b in siso)
    
    % vettore grado relavitivo noto = (rx,ry)= (2,2)
    
    % 1) derivo la prima volta l'uscita (grado 1)
    y_dot= jacobian(y,x)*x_dot; %Lfh dove h=y
    
    % 2) derivo la seconda volta l'uscita (grado 2)
    y_dot2= jacobian(y_dot,x)*x_dot; %Lf(Lfh)
    
    % 3) calcolo la matrice di disaccoppiamento (LgLf)
    B= jacobian(y_dot2,u_sym);
    
    % 4) calcolo il termine di deriva => "deriva"= no input nella derivata finale => sostituisco u con [0,0]
    phi= subs(y_dot2, u_sym, [0; 0]); 
    
    % semplifico le espressioni per comodità
    B= simplify(B);
    phi= simplify(phi);
    
    % 5) verifico che B non sia singolare = non abbia determinante nullo
    detB= simplify(det(B));
        
    if isAlways(detB == 0)
        warning("B Singular => ERROR! \n");
    else
        fprintf("B Non-Singular => OK! \n");
    end
    
    % 6) calcolo la legge di controllo u
    
    % imposto le variabili simboliche 
    nu= sym('nu',[2 1], 'real');
    
    % calcolo la formula del contro u = B^(-1)*(nu-phi)
    u= simplify(inv(B)*(nu-phi));

end

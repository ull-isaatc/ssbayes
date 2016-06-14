%Funcion de seguimiento de objetivos, el robot tratara de describir una 
%curva suavizada.

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [w,v] = Seguir_objetivos(w,v,W,V) 

     %El metodo consiste en realizar una interpolacion con el suavizado
     %entre tramos de la trayectoria.
    dist = v;
    if w > W
        w = W;
    end
    if w < -W
        w = -W;
    end 

    if (v > V)
        v = V;
    end

    if (v < 0)
        v = 0;
    end

    if  abs(w) > 0.05 && abs(w) < 0.15 && dist > V
        v = V/4;
    else
        if abs(w) > 0.01 && abs(w) < 0.05 && dist > V
            v = V/2;
        else
            if abs(w) > 0.15
                v = 0;
            end
        end
    end
end


% Calcula el ángulo relativo entre el robot y su objetivo

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [w] = Angulo_relativo(pose,objetivo )

%   Realizamos el cálculo correcto para determinar el angulo que tenemos
%   entre el robot y el objetivo que se dispone a alcanzar.
    w = atan2(objetivo(2,1)-pose(2,1),objetivo(1,1)-pose(1,1))-pose(3,1);
    while w > pi
        w = w-2*pi;
    end
    while w < -pi
        w = w+2*pi;
    end

end


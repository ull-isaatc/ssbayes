%Función para calcular las direcciones de medida desde las balizas

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [ dx_dy ] = Dir_medida(len,Y_r)

%   Esta función calcula las direcciones de medida que tienen las balizas
%   cuando tomamos la medida.

    dx_dy = [];
    
    for i=1:size(len,2)
        dx_dy(1,i) = len(i)*cos(Y_r(i,1));
        dx_dy(2,i) = len(i)*sin(Y_r(i,1));
    end


end


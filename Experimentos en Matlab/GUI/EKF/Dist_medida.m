%Función que mide las distancias tomadas desde las balizas

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [ len ] = Dist_medida(Y,Balizas,dist_max)

%   Esta función toma la sitancia entre el robot y las balizas para poder
%   realizar la represetanción
len=[];
    for i=1:size(Balizas,2)
        len(i) = sqrt((Y(1,1)-Balizas(1,i))^2 + (Y(2,1)-Balizas(2,i))^2);
        if len(i) <= dist_max
            len(i) = len(i);
        else
            len(i) = 0;
        end
    end

end


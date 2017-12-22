% Modelo de medida de distancia tipo telémetro

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function Y = bot_dist_h(x,s)
%   Implementamos nuestra funcion de medida la cual calcula la distancia
%   entre el robot y las balizas que estan colocadas en el mapa.

    Y = zeros(size(s,2),size(x,2));

    for i=1:size(s,2)
        h = sqrt((x(1,:)-s(1,i)).^2 + (x(2,:)-s(2,i)).^2);
        Y(i,:) = h;
    end
end


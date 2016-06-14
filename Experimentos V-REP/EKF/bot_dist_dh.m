% Derivada del modelo de medida de distancia tipo telémetro

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function dY = bot_dist_dh( x,s )
%   Implementamos nuestra funcion de medida la cual calcula la distancia
%   entre el robot y las balizas que estan colocadas en el mapa.

    dY = zeros(size(s,2),size(x,1));
    
    for i=1:size(s,2)
        dh = [(x(1)-s(1,i))/(sqrt((x(1)-s(1,i)).^2 + (x(2)-s(2,i)).^2));...
              (x(2)-s(2,i))/(sqrt((x(1)-s(1,i)).^2 + (x(2)-s(2,i)).^2));...
              zeros(size(x,1)-2,1)]';
        dY(i,:) = dh;
    end
    

end


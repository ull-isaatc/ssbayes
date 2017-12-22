%Función que mide la pose del robot por medio de la distancia

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [Y_dist] = Medidas_dist( Balizas,pose,sd_balizas )

%   Medimos la posición del robot con ayuda de la distancia euclídea a cada
%   una de las balizas dentro de la escena.
    y_dist=[];
    Y_dist=[];
    for i=1:size(Balizas,2)
        y_dist = sqrt((pose(2,1)-Balizas(2,i))^2 + (pose(1,1)-Balizas(1,i))^2) + sd_balizas*randn;
        Y_dist = [Y_dist ; y_dist];
    end



end


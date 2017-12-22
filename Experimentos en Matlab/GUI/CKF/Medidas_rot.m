%Función que mide la pose del robot mediante las orientaciones de las
%balizas

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [ Y_r ] = Medidas_rot(Balizas,pose,sd_balizas)

%   Función que calcula según las balizas disponibles dentro de la escena
%   los bearings que presenta nuestro robot dentro de ella.
    y_r=[];
    Y_r=[];
    for i=1:size(Balizas,2)
        y_r = atan2(pose(2,1)-Balizas(2,i), pose(1,1)-Balizas(1,i)) + sd_balizas*randn;
        Y_r = [Y_r ; y_r];
    end
end


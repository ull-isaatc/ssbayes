% DYNAMIC_MODEL Función me implementa el modelo del UKF

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [ f ] = Dynamic_model( A,X)

%   Función en la que implementamos el cálculo de la matriz resultante para
%   el modelo.

f = A*X ;

end


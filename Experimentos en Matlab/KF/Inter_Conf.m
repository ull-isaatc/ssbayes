% Transformación las coordenadas del intervalo de confianza

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [ cc_transformada ] = Inter_Conf( cc,x,y,alfa)

%   Realizamos una transformación del intervalo de confianza para que la
%   represetanción del robot se pueda realizar correctamente.

%Necesitamos un origen y un sistema de coordenadas
o0 = [0
      0
      1];
%Origen y matriz de traslación del robot

T01 = [cos(alfa) -sin(alfa) x
       sin(alfa)  cos(alfa) y
       0              0     1];
   
%Calculamos la transformación de nuestro intervalo de confianza.

cc_matriz = [cc(1,:); cc(2,:) ; ones(size(cc(2,:)))];

cc_transformada = T01*cc_matriz;


end


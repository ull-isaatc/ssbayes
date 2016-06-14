% Función para limitar la distancia de medida como si fuera real de un 
% telémetro láser

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [ Y_adap,Balizas_adap,Y_completo ] = Limitar_medida(Y_dist,Y_r,Func_medida,Balizas,Max_dist)

%   Con esta función lo que buscamos hacer es limitar la medida de tal
%   forma que simule la limitación de distancia que tiene un telémetro. La
%   función nos devolverá los parámetros que Kalman necesita para realizar
%   la estimación de forma correcta sin generar ningún tipo de error.Para
%   realizar el cálculo usaremos las medidas tomadas de rotación y
%   distancia que hemos tomado. Además dependendiendo de la función de
%   medida utilizada podremos determinar cuales son las balizas que han
%   generado dichas medidas.
    Y_adap = [];
    Balizas_adap = [];
    Y_completo = [];
    
    if Func_medida == 1
        for i=1:size(Y_dist,1)
            if Y_dist(i,1) <= Max_dist 
                Y_adap = [Y_adap ; Y_dist(i,1)];
                Y_completo = [Y_completo ; Y_dist(i,1)];
                Balizas_adap = [Balizas_adap Balizas(:,i)];
            else
                Y_completo = [Y_completo ; 0];
            end
        end
    else
        for i=1:size(Y_dist,1)
            if Y_dist(i,1) <= Max_dist
                Y_adap = [Y_adap ; Y_r(i,1)];
                Y_completo = [Y_completo ; Y_r(i,1)];
                Balizas_adap = [Balizas_adap Balizas(:,i)];
            else
                Y_completo = [Y_completo ; 0];
            end
        end
    end
end
                
                
    







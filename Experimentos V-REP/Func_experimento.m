% Script para ejecutar los experimentos que queramos desde V-REP y así
% disponer de un reinicio total de las variables en cada iteración.

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.


clc;
clear;
Inf =[];
Inf_2 =[];

for h=0:5
    [Robot_1,ekf_rms] = V_rep_funciones_V3 ;
    Inf = [Inf Robot_1];
    Inf_2 = [Inf_2 ekf_rms];
end

%Modelo de odometría.

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [ u_t ] = odometry_motion(pose_ant,pose_act)

%Este modelo implementa el modelo de odometría de tal
%manera que convertimos los cambios entre dos poses del robot en una
%rotación, una traslación y por último otra rotación. La salida de la
%función será el vector de control que introduciremos al filtro de Kalman.

Rotacion_1 = atan2(pose_act(2,1) - pose_ant(2,1),pose_act(1,1) - pose_ant(1,1)) - pose_act(3,1);
Traslacion = sqrt((pose_ant(1,1) - pose_act(1,1))^2 + (pose_ant(2,1) - pose_act(2,1))^2 );
Rotacion_2 = pose_act(3,1) - pose_ant(3,1) - Rotacion_1;

u_t = [Traslacion*cos(pose_ant(3,1));Traslacion*sin(pose_ant(3,1));Rotacion_1 + Rotacion_2];



end


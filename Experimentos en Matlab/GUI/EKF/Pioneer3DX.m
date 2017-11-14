%PIONEER3DX Representación 2D del robot Pioneer

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(x,y,alfa,R)

%   Con esta función implementamos las matrices de transformación para
%   hacer posible que se muestre durante la simulación el aspecto de
%   nuestro robot y sus cambios de rotación, etc.

%Necesitamos un origen y un sistema de coordenadas

o0 = [0
      0
      1];

%Origen y matriz de traslacion del robot  
 
 T01 = [cos(alfa) -sin(alfa) x
       sin(alfa)  cos(alfa) y
          0         0     1];

%Representamos la punta del robot

phi=[-pi/2:0.1:pi/2];
xc=R/2+R*1.1*cos(phi);
yc=R*1.1*sin(phi);
unos=ones(1,32);

p1=[xc; yc;unos]; %Matriz que contiene toda la información sobre la circunferencia de nuestro robot

punto=T01*p1; %Transformación de coordenadas

%Representamos los laterales

Lateral_sup = [xc(32) R/4;
               yc(32) R*0.85;
               1 1];
Lateral_sup1 = T01*Lateral_sup;

Lateral_inf = [xc(1) R/4;
               yc(1) -R*0.85;
               1 1];
Lateral_inf1 = T01*Lateral_inf;


%Representamos la parte trasera

phi=[-pi/2:0.1:pi/2];

xc=R/2+(R*0.83)*cos(phi);
xc=xc*(-1);
yc=(R*0.83)*sin(phi)*(-1);
unos=ones(1,32);

p1=[xc; yc;unos]; %Matriz que contiene toda la información sobre la circunferencia de nuestro robot

trasera=T01*p1; %Transformación de coordenadas

linea_abajo = [xc(32) Lateral_inf(1,2) ;
               yc(32) Lateral_inf(2,2) ;
               1 1];
linea_abajo1 = T01*linea_abajo;

linea_arriba = [Lateral_sup(1,2) xc(1);
               Lateral_sup(2,2) yc(1);
               1 1];
linea_arriba1 = T01*linea_arriba;

%Representamos las ruedas del robot

%Rueda derecha

ruedader=[ R/3 R/3 -R/3 -R/3 R/3;
          16*R/21 19*R/21 19*R/21 16*R/21 16*R/21;
          1 1 1 1 1 ]; %Definimos los puntos de nuestra rueda

ruedader1=T01*ruedader; %Transformamos las coordenadas

%Rueda izquierda

ruedaizq=[  R/3 R/3 -R/3 -R/3 R/3;
         -16*R/21 -19*R/21 -19*R/21 -16*R/21 -16*R/21;
          1 1 1 1 1 ]; %Matriz que contiene la información del robot


ruedaizq1=T01*ruedaizq; %Transformada


end


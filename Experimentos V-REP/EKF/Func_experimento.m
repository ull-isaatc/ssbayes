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
Escena = 19;
sd_baliza = 0.05;
Modelo_medida = 1;
Inf_kf =[];
Inf_ekf =[];
Inf_ukf =[];
Inf_ckf =[];


for h=0:2
    [Robot_kf,kf_rms,objetivos,Pos_bal] = V_rep_KF(Escena,sd_baliza) ;
    Inf_kf = [Inf_kf kf_rms];
    [Robot_ekf,ekf_rms] = V_rep_EKF(Escena,sd_baliza,Modelo_medida) ;
    Inf_ekf = [Inf_ekf ekf_rms];
    [Robot_ukf,ukf_rms] = V_rep_UKF(Escena,sd_baliza,Modelo_medida) ;
    Inf_ukf = [Inf_ukf ukf_rms];
    [Robot_ckf,ckf_rms] = V_rep_CKF(Escena,sd_baliza,Modelo_medida) ;
    Inf_ckf = [Inf_ckf ckf_rms];
    
end

kf_rms= mean(Inf_kf(1,:));
ekf_rms= mean(Inf_ekf(1,:));
ukf_rms= mean(Inf_ukf(1,:));
ckf_rms= mean(Inf_ckf(1,:));

trayectoria=Robot_ckf.Trayectoria_real;

Est_ckf = Robot_ckf.Trayectoria_est;
Est_ukf = Robot_ukf.Trayectoria_est;
Est_ekf = Robot_ekf.Trayectoria_est;
Est_kf = Robot_kf.Trayectoria_est;

x_1 = min(objetivos(1,:))-2;
x_2 = max(objetivos(1,:))+2;
y_1 = min(objetivos(2,:))-2;
y_2 = max(objetivos(2,:))+2;

h = plot(trayectoria(1,:),trayectoria(2,:),'b-',... 
               objetivos(1,:),objetivos(2,:),'b^',... 
               Pos_bal(1,:),Pos_bal(2,:),'go',... 
               Est_kf(1,:),Est_kf(2,:),'m--',... 
               Est_ekf(1,:),Est_ekf(2,:),'g--',... 
               Est_ukf(1,:),Est_ukf(2,:),'k--',... 
               Est_ckf(1,:),Est_ckf(2,:),'c--'); 
      legend('Location','NorthWest','Trayectoria real','Objetivos','Balizas',...
             'KF','EKF','UKF','CKF');
      title('Odometría: 7% deslizamiento')
      


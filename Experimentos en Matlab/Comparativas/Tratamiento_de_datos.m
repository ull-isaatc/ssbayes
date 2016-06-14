% Script para tratar los datos devueltos en el workspace

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

rng(1); %Colocamos la misma semilla para que el script siempre se ejecute en las mismas condiciones

h_func = @bot_dist_h;
dh_dx_func = @bot_dist_dh;

Balizas = Pos_bal;
sd_baliza = 0.05 ; %Parámetro de afectación de ruido de nuestro robot
sd_odometria = 0.05 ; %Parámetro de ruido que aplicamos a la odometría 
dt = 0.05 ; %Diferencial de tiempo por el que realizamos nuestra simulación
Modelo_medida = 1;
dist_max = 5 ; %distancia maxima que es capaz de medir el telemetro.

%Definimos la pose inicial que tendra nuestro robot
x_inicial= 0;
y_inicial= 0;
alfa_inicial = 0;
pose = [x_inicial;y_inicial;alfa_inicial]; %Guardamos la pose inicial del robot real insertado.

%Definimos el primer punto de la trayectoria que vamos a
%seguir, lógicamente el primer punto es la posición del robot.
trayectoria = Robot_ckf.Trayectoria_real;
X_mod_ruido = trayectoria;

Y = [];
Y_r = [];
U_t = [0;0;0];
%% Inicialización de parámetros del filtro

for i=1:size(trayectoria,2)
    [y_dist] = Medidas_dist(Balizas,trayectoria(:,i),sd_baliza);
    Y = [Y y_dist];
    [y_r] = Medidas_rot(Balizas,trayectoria(:,i),0);
    Y_r = [Y_r y_r];
    if i > 1
    u_t= odometry_motion(trayectoria(:,i-1),trayectoria(:,i));
    U_t = [U_t u_t];
    end
end

Odom = Robot_ckf.Trayectoria_real + randn*10 ;
Y_kf = Odom;
 %% Inicialización de los filtros
 
 M_kf = [0;0;0];
 P_kf = diag([0 0 0 ]); %Inicializamos los vectores que contendrán la información inicial para del filtro clásico.
 
 M_ekf = [0;0;0];
 P_ekf = diag([0 0 0 ]); %Inicializamos los vectores que contendrán la información inicial para del filtro extendido.
 
 M_ukf = [0;0;0];
 P_ukf = diag([0.7 0.7 10]); %Inicializamos los vectores que contendrán la información inicial para del filtro clásico.
 
 M_ckf = [0;0;0];
 P_ckf = diag([0.1 0.1 0.1]);
 
 R = sd_baliza^2 ; %Introducimos el ruido que tendrá en cuenta el filtro en las ecuaciones.
 
 %% Construcción de matrices A y Q
 %Construimos las matrices A y Q necesarias para el ciclo de predicción.
 qx = 0.1 ;
 qy = 0.1 ;
 
 A = [1 dt 0;
      0 1 0;
      0 0 1];
  
 Q = [(1/3)*(dt^3)*qx (1/2)*(dt^2)*qx 0;
      (1/2)*(dt^2)*qx (dt)*qx         0;
               0         0       dt*qy];
      
 %Seguimiento y animación de nuestro robot
 
 
 MM_kf = zeros(size(M_kf,1),size(Y_kf,2)); %Inicialicamos el vector que contendrá toda la información del filtro clásico.
 PP_kf = zeros(size(M_kf,1),size(M_kf,1),size(Y_kf,2));
 
 MM_ekf = zeros(size(M_ekf,1),size(Y,2)); %Inicialicamos el vector que contendrá toda la información del filtro extendido.
 PP_ekf = zeros(size(M_ekf,1),size(M_ekf,1),size(Y,2));
 
 MM_ukf = zeros(size(M_ukf,1),size(Y,2)); %Inicialicamos el vector que contendrá toda la información del filtro clásico.
 PP_ukf = zeros(size(M_ukf,1),size(M_ukf,1),size(Y,2));
 
 MM_ckf = zeros(size(M_ckf,1),size(Y,2)); %Inicialicamos el vector que contendrá toda la información del filtro clásico.
 PP_ckf = zeros(size(M_ckf,1),size(M_ckf,1),size(Y,2));

 
 %% Aplicación de los filtros
 Y_completo_EKF = [];
 Y_completo_UKF = [];
 Y_completo_CKF = [];
 
 for k=1:size(Y,2) %La dimensión del bucle es tan grande como la trayectoria o el número de medidas
     
     % Seguimiento con el KF
      [M_kf,P_kf] = kf_predict(M_kf,P_kf,A,Q,eye(3),U_t(:,k));
      [M_kf,P_kf] = kf_update(M_kf,P_kf,Odom(:,k),eye(3),R*eye(3));
      MM_kf(:,k) = M_kf ;
      PP_kf(:,:,k) = P_kf;
     % Seguimiento con el EKF
      
      [M_ekf,P_ekf] = ekf_predict1(M_ekf,P_ekf,A,Q,Odom(:,k));
      [Y_adap,Balizas_adap,y_completo] = Limitar_medida(Y(:,k),Y_r(:,k),Modelo_medida,Balizas,dist_max);
      [M_ekf,P_ekf] = ekf_update1(M_ekf,P_ekf,Y_adap,dh_dx_func,R*eye(size(Balizas_adap,2)),h_func,[],[Balizas_adap]);
      MM_ekf(:,k) = M_ekf ;
      PP_ekf(:,:,k) = P_ekf;
      Y_completo_EKF = [Y_completo_EKF y_completo];
     % Seguimiento con el UKF
     
      %M_ukf = (Odom(:,k));
      [M_ukf,P_ukf] = ukf_predict1(M_ukf,P_ukf,A,Q);
      M_ukf = (8*Odom(:,k)+2*M_ukf)/10;
      [Y_adap,Balizas_adap,y_completo] = Limitar_medida(Y(:,k),Y_r(:,k),Modelo_medida,Balizas,dist_max);
      [M_ukf,P_ukf] = ukf_update1(M_ukf,P_ukf,Y_adap,h_func,R*eye(size(Balizas_adap,2)),[Balizas_adap]);
      
      MM_ukf(:,k) = M_ukf ;
      PP_ukf(:,:,k) = P_ukf;
      
      Y_completo_UKF = [Y_completo_UKF y_completo];
      
      % Seguimiento con el CKF
      
      %M_ckf = (Odom(:,k));
      [M_ckf,P_ckf] = ckf_predict(M_ckf,P_ckf,A,Q);
      M_ckf = (1*Odom(:,k) + 9*M_ckf)/10;
      [Y_adap,Balizas_adap,y_completo] = Limitar_medida(Y(:,k),Y_r(:,k),Modelo_medida,Balizas,dist_max);
      [M_ckf,P_ckf] = ckf_update(M_ckf,P_ckf,Y_adap,h_func,R*eye(size(Balizas_adap,2)),Balizas_adap);
      MM_ckf(:,k) = M_ckf ;
      PP_ckf(:,:,k) = P_ckf;
      Y_completo_CKF = [Y_completo_CKF y_completo];
      
      
      end
     
 %% Cálculo de errores
 
 kf_rms = sqrt(mean((X_mod_ruido(1,:)-MM_kf(1,:)).^2+(X_mod_ruido(2,:)-MM_kf(2,:)).^2)); %Calculamos el error cuadrático medio de la posición estimada usando el filtro clásico de Kalman.
 ekf_rms = sqrt(mean((X_mod_ruido(1,:)-MM_ekf(1,:)).^2+(X_mod_ruido(2,:)-MM_ekf(2,:)).^2)); % Calculamos el RMS de la posición estimada con el filtro extendido de Kalman.
 ukf_rms = sqrt(mean((X_mod_ruido(1,:)-MM_ukf(1,:)).^2+(X_mod_ruido(2,:)-MM_ukf(2,:)).^2)); %Calculamos el error cuadrático medio de la posición estimada usando el filtro clásico de Kalman.
 ckf_rms = sqrt(mean((X_mod_ruido(1,:)-MM_ckf(1,:)).^2+(X_mod_ruido(2,:)-MM_ckf(2,:)).^2));
%Representamos la estimación obtenida para ambos filtros.
 
% h = plot(trayectoria(1,:),trayectoria(2,:),'r-',... 
%                objetivos(1,:),objetivos(2,:),'b^',... 
%                Pos_bal(1,:),Pos_bal(2,:),'go',... 
%                MM_kf(1,:),MM_kf(2,:),'m-.',... 
%                MM_ekf(1,:),MM_ekf(2,:),'g--',... 
%                MM_ukf(1,:),MM_ukf(2,:),'k+',... 
%                MM_ckf(1,:),MM_ckf(2,:),'cx','LineWidth',1.15,'MarkerSize',3.5); 
%       legend('Location','NorthWest','Trayectoria real','Objetivos','Balizas',...
%              'KF','EKF','UKF','CKF');
%       title('Trayectoria general')
%     axis([-10 15 -10 15])

h = plot(trayectoria(1,:),trayectoria(2,:),'r-',... 
               objetivos(1,:),objetivos(2,:),'b^',... 
               Pos_bal(1,:),Pos_bal(2,:),'go',... 
               MM_ekf(1,:),MM_ekf(2,:),'g--','LineWidth',1.15,'MarkerSize',3.5);
      legend('Location','NorthWest','Trayectoria real','Objetivos','Balizas',...
             'EKF');
      title('Trayectoria General - EKF')
       
 
      
      
 
 
 





 



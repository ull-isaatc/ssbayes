% Script para la localización de un robot en un entorno de 2 dimensiones
% comparando distintos tipos de filtros de Kalman.

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.


% EXPLICACIÓN ----------------------------------------------------------
%Script para la localización de un robot en un 
%entorno de 2 dimensiones. Las variables que definirán la pose del robot
%serán la x e y para las coordenadas y TITA para orientación. Usaremos el
%varios filtros de Kalman para realizar la localización de nuestro robot, además
%también se implementará el algortimo del filtro clásico de Kalman para así comparar
%cual presenta mejor comportamiento.
%-----------------------------------------------------------------------

rng(1); %Colocamos la misma semilla para que el script siempre se ejecute en las mismas condiciones
clear % Eliminamos todas las variables que guardamos en el workspace
clc;
clf; %Limpiamos el workSpace y las figuras 

%% Variables de simulacion
%Establecemos la velocidad para la simulación del experimento

velocidad_simulacion = 0.01 ;
mostrar_simulacion = true;
preguntar_simular = true;
depuracion = false ;
steps = 1; %Pasos usados por el bucle para la represetanción gráfica
len = 100 ; %Tamaño asignado para mostrar las mediciones por parte de las balizas
Radio = 0.3 ; %Radio del dibujo
radioruedas = 0.1 ; %definimos el radio de las ruedas de nuestro robot
intervalo_conf = 10;
Modelo_medida = 1 ;

%Importamos el modelo de medida y sus derivadas (en este caso medimos la
%distancia)

if Modelo_medida == 1 %Modelo de medida de distancia
    h_func = @bot_dist_h;
    dh_dx_func = @bot_dist_dh;
end
if Modelo_medida == 2 %Modelo de medida de rotaciones
    h_func = @bot_h;
    dh_dx_func = @bot_dh_dx;
end

%Imprimimos algunos mensajes informativos en el workspace

disp('Seguimiento de un robot móvil usando el EKF,el filtro de Kalman clásico y el filtro UKF')
disp('  EKF = CIAN || KF = ROJO || UKF = VERDE || CKF = NEGRO')
disp('  Autor: Iván Rodríguez Méndez')
disp('  Basado en la toolbox de Simo Särkkä')

%Creamos la trayectoria que seguirá el robot y además colocamos las balizas
%para poder calcular su posición.

disp(' ')
disp('PARÁMETROS DE SIMULACIÓN:')

Baliza_1 = [1;5];
Baliza_2 = [8;5];
Baliza_3 = [5;8];
Baliza_4 = [9;10];
Baliza_5 = [3;1];
Baliza_6 = [4;6];

Balizas = [Baliza_1 Baliza_2 Baliza_3 Baliza_4 Baliza_5 Baliza_6] %Guardamos toda la informacion de las balizas

fprintf(' -> La posición de la primera baliza \n    es X= %f  e Y= %f \n',Baliza_1(1,1),Baliza_1(2,1))
fprintf(' -> La posición de la segunda baliza \n    es X= %f  e Y= %f \n',Baliza_2(1,1),Baliza_2(2,1))

sd_baliza = 0.05 ; %Parámetro de afectación de ruido de nuestro robot
sd_odometria = 0.05 ; %Parámetro de ruido que aplicamos a la odometría 
dt = 0.1 ; %Diferencial de tiempo por el que realizamos nuestra simulación

fprintf(' -> La afectación del ruido es %f (varianza) \n',sd_baliza)
fprintf(' -> La afectación del ruido es %f (varianza) \n',sd_odometria)
fprintf(' -> El tiempo de muestreo es %f segundos \n',dt)
fprintf(' -> El radio de las ruedas es %f metros \n',radioruedas)
%Creamos la trayectoria que nuestro robot deberá seguir 

 t = 0 ; %Inicializamos el tiempo de muestreo (aunque en estos scripts ya no es necesario
 x_mod = [0;0;0];
 X_mod = [];
 Y = []; %Inicialización del vector de medidas (de distancia)
 T = []; %Inicializamos el vector de tiempos
 
 %Generamos las velocidades lineales y angulares de nuestro robot, con
 %ellas conseguiremos que se siga una trayectoria en función de los
 %comandos que le mandemos al robot.
  %% Caracterización del robot
  
%Generamos las velocidades lineales y angulares de nuestro robot
V_LINEAL = 0.5; %Velocidad lineal máxima
V_ANGULAR = 7.2; %Velocidad de rotación máxima
Limite_trayectoria = 3000 ; %Definimos el límite de la trayectoria, es decir la cantidad máxima de puntos que cogemos.
dist_max = 5 ; %distancia maxima que es capaz de medir el telemetro.

%Definimos la pose inicial que tendra nuestro robot
x_inicial= 0;
y_inicial= 0;
alfa_inicial = 0;
pose = [x_inicial;y_inicial;alfa_inicial]; %Guardamos la pose inicial del robot real insertado.


%Definimos los parametros que nos ayudaran a calcular la
%trayectoria

Epsilon = 0.3; %Umbral de distancia máxima permitida (error). Usado para seguir los objetivos.
V = V_LINEAL;
W = V_ANGULAR*pi/(180);

%Definimos el primer punto de la trayectoria que vamos a
%seguir, lógicamente el primer punto es la posición del robot.
trayectoria = [x_inicial;y_inicial;alfa_inicial];

%Calculamos los objetivos que tenemos que alcanzar, es decir
%las coordenadas que tienen para poder alcanzarlas.
disp('Calculando la localización de los objetivos...');
objetivos = [0 0;1 1 ; 2 5 ;3 3; 5 3 ; 7 7 ; 8 12 ]';
disp('posición de los objetivos calculada');

%Definimos un indice para recorrer el vector de objetivos (para
%conocer cual es el objetivo que queremos buscar)
obj_actual = 1;

%Inicializamos los vectores de medida que usaremos en el
%recorrido para la simulación.
%% Inicialización de parámetros de vectores
Y_r = [];
Y_rot = [];
Y_dist = [];
x = [];
X = [];
Odom = [];
Tiempo = [];
tiempo = 0;
U_t = [];

%% Inicialización de parámetros del filtro

%% Trazado de la trayectoria

%Iniciamos un índice para usar dentro del bucle
k=1;
%% Seguimiento de puntos en la trayectoria y aplicación de Kalman
for i=1:size(objetivos,2)
    while norm(trayectoria(1:2,size(trayectoria,2))-objetivos(1:2,obj_actual)) > Epsilon && size(trayectoria,2) <= Limite_trayectoria
         %Dentro de este bucle lo que intentamos hacer es que
         %el robot se mueva de tal manera que se vaya
         %aproximando al siguiente objetivo pendiente en el
         %vector. Básicamente aplicamos un pequeño algoritmo en
         %forma de interpolación lineal entre objetivos.

         %Lo primero que hacemos es recuperar la posición
         %actual del robot y guardarla.
        pose = trayectoria(:,size(trayectoria,2));
        x = pose;
        X = [X x];

        %% Seguimiento de objetivos
       Objetivo = objetivos(:,obj_actual); %Guardamos en una variable el objetivo que debemos alcanzar.
       angulo = Angulo_relativo(pose,Objetivo); % Calculamos el angulo entre la posicion actual y el objetivo que pretendemos alcanzar.
       v = norm(pose(1:2)-objetivos(1:2,obj_actual)); %Calculamos la distancia entre la posicion actual y el objetivo que queremos alcanzar.

       w = angulo; %Guardamos el angulo que hemos calculado para pasar a trabajar con el.

       [w,v] = Seguir_objetivos(w,v,W,V); %Aplicamos nuestro algortimo para que el robot siga los objetivos describiendo una trayectoria curva.
        
       % Guardaremos en trayectoria el recorrido de nuestro robot sin
       % ruido
        trayectoria(3,k) = pose(3,1) + w*dt ;
        trayectoria(1,k) = pose(1,1) + v*cos(trayectoria(3,k))*dt;
        trayectoria(2,k) = pose(2,1) + v*sin(trayectoria(3,k))*dt ;
        
       % En X_mod_ruido guardaremos la trayectoria modificada por ruidos
        X_mod_ruido(3,k) = trayectoria(3,k) + randn*sd_odometria*0.5;
        X_mod_ruido(1,k) = trayectoria(1,k) + randn*sd_odometria;
        X_mod_ruido(2,k) = trayectoria(2,k) + randn*sd_odometria;
        
        [y_rot] = Medidas_rot(Balizas,X_mod_ruido(:,k),0); %Vector de rotaciones para la representacion
       
        Odom(:,k) = trayectoria(:,k); %Vector que guardará una la información de la odometría.
        U_t(:,k) = odometry_motion(pose,trayectoria(:,k));
        if Modelo_medida == 1
            [y_dist] = Medidas_dist(Balizas,X_mod_ruido(:,k),sd_baliza);
            [y_r] = Medidas_rot(Balizas,X_mod_ruido(:,k),0);
            Y = [Y y_dist];
            Y_r = [Y_r y_r];
        end
        if Modelo_medida == 2
            [y_r] = Medidas_rot(Balizas,X_mod_ruido(:,k),sd_baliza);
            [y_dist] = Medidas_dist(Balizas,X_mod_ruido(:,k),0);
            Y_r = [Y_r y_r];
            Y = [Y y_dist];
        end
        
        %Imprimimos dos variables de interes para saber en que
        %paso del bucle nos encontramos y que objetivo
        %pretendemos alcanzar.
        %Guardamos todas las variables relevantes.
        tiempo = tiempo +dt; 
        Y_rot = [Y_rot y_rot];
        Tiempo = [Tiempo tiempo];
        %Mostramos el objetivo actual
        obj_actual
        Obj_real = obj_actual;
        %Mostramos el tamaño del vector de trayectorias.
        size(trayectoria,2)
        k= k +1; %Indices que usamos para Kalman
       
    end
    obj_actual = obj_actual+1; %Cuando salimos del bucle pasamos a buscar el siguiente objetivo.
end
Y_kf = Odom;
clc;
disp('Trayectoria generada con éxito');
plot(trayectoria(1,:),trayectoria(2,:));
hold on
Representar_obj(objetivos); %Representamos la localización de los objetivos en la escena.
 
 %% Inicialización de los filtros
 
 M_kf = [0;0;0];
 P_kf = diag([0 0 0 ]); %Inicializamos los vectores que contendrán la información inicial para del filtro clásico.
 
 M_ekf = [0;0;0];
 P_ekf = diag([0 0 0 ]); %Inicializamos los vectores que contendrán la información inicial para del filtro extendido.
 
 M_ukf = [0;0;0];
 P_ukf = diag([0.7 0.7 10]); %Inicializamos los vectores que contendrán la información inicial para del filtro clásico.
 
 M_ckf = [0;0;0];
 P_ckf = diag([0.1 0.1 0.1]);
 
 R = sd_baliza^2 + sd_odometria^2; %Introducimos el ruido que tendrá en cuenta el filtro en las ecuaciones.
 
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

 %Colocamos una condición para preguntar a la hora de realizar la
 %simulación o hacerla directamente.
 clf;
 %% Preguntar para simular
if preguntar_simular == true 
     
     disp('<Pulsa una tecla para mostrar la simulación>')
     disp(' ')
     Representar_obj(objetivos);
     plot(X_mod_ruido(1,:),X_mod_ruido(2,:),'b')
     x_min = min(objetivos(1,:))-2;
     x_max = max(objetivos(1,:))+2;
     y_min =  min(objetivos(2,:))-2;
     y_max =  max(objetivos(2,:))+2;
     axis([x_min x_max y_min y_max ]);
     title('PULSA UNA TECLA PARA COMENZAR CON LA SIMULACIÓN')
     pause()
 end
 
 disp(' ')
 disp('¡Comienza la simulación!')
 
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
     
      M_ukf = (Odom(:,k));
      [M_ukf,P_ukf] = ukf_predict1(M_ukf,P_ukf,A,Q);
      M_ukf = (9*Odom(:,k)+1*M_ukf)/10;
      [Y_adap,Balizas_adap,y_completo] = Limitar_medida(Y(:,k),Y_r(:,k),Modelo_medida,Balizas,dist_max);
      [M_ukf,P_ukf] = ukf_update1(M_ukf,P_ukf,Y_adap,h_func,R*eye(size(Balizas_adap,2)),[Balizas_adap]);
      
      MM_ukf(:,k) = M_ukf ;
      PP_ukf(:,:,k) = P_ukf;
      
      Y_completo_UKF = [Y_completo_UKF y_completo];
      
      % Seguimiento con el CKF
      
      M_ckf = (Odom(:,k));
      [M_ckf,P_ckf] = ckf_predict(M_ckf,P_ckf,A,Q);
      M_ckf = (9*Odom(:,k) + 1*M_ckf)/10;
      [Y_adap,Balizas_adap,y_completo] = Limitar_medida(Y(:,k),Y_r(:,k),Modelo_medida,Balizas,dist_max);
      [M_ckf,P_ckf] = ckf_update(M_ckf,P_ckf,Y_adap,h_func,R*eye(size(Balizas_adap,2)),Balizas_adap);
      MM_ckf(:,k) = M_ckf ;
      PP_ckf(:,:,k) = P_ckf;
      Y_completo_CKF = [Y_completo_CKF y_completo];
      
end
     
    
 
 
 %% Cálculo de errores
 
 kf_rmse = sqrt(mean((X_mod_ruido(1,:)-MM_kf(1,:)).^2+(X_mod_ruido(2,:)-MM_kf(2,:)).^2)); %Calculamos el error cuadrático medio de la posición estimada usando el filtro clásico de Kalman.
 ekf_rmse = sqrt(mean((X_mod_ruido(1,:)-MM_ekf(1,:)).^2+(X_mod_ruido(2,:)-MM_ekf(2,:)).^2)); % Calculamos el RMS de la posición estimada con el filtro extendido de Kalman.
 ukf_rmse = sqrt(mean((X_mod_ruido(1,:)-MM_ukf(1,:)).^2+(X_mod_ruido(2,:)-MM_ukf(2,:)).^2)); %Calculamos el error cuadrático medio de la posición estimada usando el filtro clásico de Kalman.
 ckf_rmse = sqrt(mean((X_mod_ruido(1,:)-MM_ckf(1,:)).^2+(X_mod_ruido(2,:)-MM_ckf(2,:)).^2));
%Representamos la estimación obtenida para ambos filtros.
 
%% Representación gráfica

if mostrar_simulacion == true 
    %Representaciones gráficas de los resultados de la estimación 
    %Comenzamos con la representación del filtro clásico de Kalman.
    
    %% Representación del filtro clásico
    %FILTRO CLÁSICO DE KALMAN.
      M = MM_kf(:,1);  
      P = PP_kf(:,:,1);
      EST = M;
      
      %Calculamos la representación para la estimación del primer vector de
      %estados
      [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
      
      tt = (0:0.01:1)*2*pi;
      %Calculamos el intervalo de confianza
      
      cc = repmat([0;0],1,length(tt)) + ...
             intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
         
      cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
      %Representamos la trayectoria real junto con la estimación del KF y
      %la posición de las balizas en el mapa.
      
      h = plot(X_mod_ruido(1,:),X_mod_ruido(2,:),'b-',... %Dibuja la línea  que reprensenta la trayectoria real
               M(1),M(2),'ro',... %representa la posición estimada
               EST(1,:),EST(2,:),'r--',... %Dibujamos la estimación
               cc(1,:),cc(2,:),'m-',...
               Baliza_1(1),Baliza_1(2),'g--',... %Medida desde la baliza
               Baliza_1(1),Baliza_1(2),'k^',... %Dibuja el triángulo correspondiente a la baliza
               Baliza_2(1),Baliza_2(2),'g--',... %Dibuja la medida desde la baliza
               Baliza_2(1),Baliza_2(2),'k^',... %Dibuja la segunda baliza
               Baliza_3(1),Baliza_3(2),'g--',...
               Baliza_3(1),Baliza_3(2),'k^',...
               Baliza_4(1),Baliza_4(2),'g--',...
               Baliza_4(1),Baliza_4(2),'k^',...
               Baliza_5(1),Baliza_5(2),'g--',...
               Baliza_5(1),Baliza_5(2),'k^',...
               Baliza_6(1),Baliza_6(2),'g--',...
               Baliza_6(1),Baliza_6(2),'k^');
     
      if Modelo_medida == 1
        title('Localización usando diversos filtros (Distancia)')
      end
      if Modelo_medida == 2
          title('Localización usando diversos filtros (Ángulo)')
      end
      
      x_min = min(objetivos(1,:))-2;
      x_max = max(objetivos(1,:))+2;
      y_min =  min(objetivos(2,:))-2;
      y_max =  max(objetivos(2,:))+2;
      axis([x_min x_max y_min y_max ]);
      hold on
      
      %Marcamos ciertos puntos del mapa con texto para poder identificarlos
      %mejor.
      
      %Colocamos unas marcas de texto en la representación
      
      text(x_inicial-1,y_inicial-1,'SALIDA')
      texto = 'Baliza ';
      b = 1;
      a = texto;
      for tex_i=1:size(Balizas,2)
        text(Balizas(1,tex_i),Balizas(2,tex_i)-1,a)
        b = num2str(b);
        a = [texto b];
        b = str2num(b);
        b = b+1;
        
      end
      
      %Representamos el robot estimado por el filtro de Kalman clásico.
      Robot = plot(punto(1,:),punto(2,:),'r--',...
                   ruedader1(1,:),ruedader1(2,:),'g',...
                   ruedaizq1(1,:),ruedaizq1(2,:),'g',...
                   Lateral_sup1(1,:),Lateral_sup1(2,:),'r--',...
                   Lateral_inf1(1,:),Lateral_inf1(2,:),'r--',...
                   trasera(1,:),trasera(2,:),'r--',...
                   linea_abajo1(1,:),linea_abajo1(2,:),'r--',...
                   linea_arriba1(1,:),linea_arriba1(2,:),'r--');
       hold on
       
       %Calculamos la representación del robot real.
       [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(X_mod_ruido(1,1),X_mod_ruido(2,1),X_mod_ruido(3,1),Radio);
       
       %Representamos el robot real.
       Robot_real = plot(punto(1,:),punto(2,:),'b',...
                   ruedader1(1,:),ruedader1(2,:),'g',...
                   ruedaizq1(1,:),ruedaizq1(2,:),'g',...
                   Lateral_sup1(1,:),Lateral_sup1(2,:),'b',...
                   Lateral_inf1(1,:),Lateral_inf1(2,:),'b',...
                   trasera(1,:),trasera(2,:),'b',...
                   linea_abajo1(1,:),linea_abajo1(2,:),'b',...
                   linea_arriba1(1,:),linea_arriba1(2,:),'b');
       
               
      %Pasamos a realizar los cálculos con el filtro extendido de kalman.
      
      %% Representación del filtro extendido de Kalman
      %FITLRO EXTENDIDO DE KALMAN.
      
      M = MM_ekf(:,1);  
      P = PP_ekf(:,:,1);
      EST = M;
      
      %Calculamos la representación del primer estado para el EKF.
      [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
      
      %Calculamos el intervalo de confianza
      cc = repmat([0;0],1,length(tt)) + ...
             intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
         
      cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
      %Representamos la estimación del primer estado para el EKF.
      
      h_2 = plot(M(1),M(2),'co',... %representa la posición estimada
               EST(1,:),EST(2,:),'c--',... %Dibujamos la estimación
               cc(1,:),cc(2,:),'m-');
      hold on
     
      %Representamos el dibujo de nuestro robot.
      Robot_ekf = plot(punto(1,:),punto(2,:),'c--',...
                   ruedader1(1,:),ruedader1(2,:),'g',...
                   ruedaizq1(1,:),ruedaizq1(2,:),'g',...
                   Lateral_sup1(1,:),Lateral_sup1(2,:),'c--',...
                   Lateral_inf1(1,:),Lateral_inf1(2,:),'c--',...
                   trasera(1,:),trasera(2,:),'c--',...
                   linea_abajo1(1,:),linea_abajo1(2,:),'c--',...
                   linea_arriba1(1,:),linea_arriba1(2,:),'c--');
       hold on
       
       %% Represetanción del UKF
       %FITLRO UKF.
      
      M = MM_ukf(:,1);  
      P = PP_ukf(:,:,1);
      EST = M;
      
      %Calculamos la representación del primer estado para el UKF.
      [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
     
      %Calculamos el intervalo de confianza para estimar la propagación del
      %error.
      cc = repmat([0;0],1,length(tt)) + ...
              intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)]; %Calculamos el intervalo de confianza
          
      cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
      
      %Representamos la estimación del primer estado para el UKF.
      
      h_3 = plot(M(1),M(2),'go',... %representa la posición estimada
               EST(1,:),EST(2,:),'g--',... %Dibujamos la estimación
               cc(1,:),cc(2,:),'m-');
      hold on
     
      %Representamos el dibujo de nuestro robot.
      Robot_ukf = plot(punto(1,:),punto(2,:),'g--',...
                   ruedader1(1,:),ruedader1(2,:),'g',...
                   ruedaizq1(1,:),ruedaizq1(2,:),'g',...
                   Lateral_sup1(1,:),Lateral_sup1(2,:),'g--',...
                   Lateral_inf1(1,:),Lateral_inf1(2,:),'g--',...
                   trasera(1,:),trasera(2,:),'g--',...
                   linea_abajo1(1,:),linea_abajo1(2,:),'g--',...
                   linea_arriba1(1,:),linea_arriba1(2,:),'g--');
       hold on
       
       
      %% Representación del CKF
      %Filtro CKF
      
      M = MM_ckf(:,1);  
      P = PP_ckf(:,:,1);
      EST = M;
      
      %Calculamos la representación del primer estado para el EKF.
      [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
      
      %Calculamos el intervalo de confianza
      cc = repmat([0;0],1,length(tt)) + ...
             intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
         
      cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
      %Representamos la estimación del primer estado para el EKF.
      
      h_4 = plot(M(1),M(2),'ko',... %representa la posición estimada
               EST(1,:),EST(2,:),'k--',... %Dibujamos la estimación
               cc(1,:),cc(2,:),'m-');
      hold on
     
      %Representamos el dibujo de nuestro robot.
      Robot_ckf = plot(punto(1,:),punto(2,:),'k--',...
                   ruedader1(1,:),ruedader1(2,:),'k',...
                   ruedaizq1(1,:),ruedaizq1(2,:),'k',...
                   Lateral_sup1(1,:),Lateral_sup1(2,:),'k--',...
                   Lateral_inf1(1,:),Lateral_inf1(2,:),'k--',...
                   trasera(1,:),trasera(2,:),'k--',...
                   linea_abajo1(1,:),linea_abajo1(2,:),'k--',...
                   linea_arriba1(1,:),linea_arriba1(2,:),'k--');
       hold on
      
       
      EST = []; % Inicializamos un vector para guardar la estimación.
      
      %Usamos un bucle para poder ir actualizando las representaciones de
      %los robots.
      
      %% Bucle de actualización de gráficos
          for k=1:steps:size(Y,2)
           
            %Definimos las longitudes de los impactos en el robot.
            [len] = Dist_medida(X_mod_ruido(:,k),Balizas,dist_max);
            
            %% Actualización del filtro clásico de Kalman
            %Actualizamos la información del FILTRO CLÁSICO.
            M = MM_kf(:,k);
            Media = MM_kf(:,k);
            P = PP_kf(:,:,k);
            EST = MM_kf(:,1:k);

            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
            
            % Intervalo de confianza.

            cc = repmat([0;0],1,length(tt)) + ...
             intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
         
            cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));

            % Direcciones de los impactos.
            dx_dy = Dir_medida(len,Y_rot(:,k));
           
            % Actualizamos las representaciones.
            set(h(2),'xdata',Media(1)); set(h(2),'ydata',Media(2));
            set(h(3),'xdata',EST(1,:)); set(h(3),'ydata',EST(2,:)); 
            set(h(4),'xdata',cc(1,:)); set(h(4),'ydata',cc(2,:)); 
             set(h(5),'xdata',[Baliza_1(1);Baliza_1(1)+dx_dy(1,1)]); set(h(5),'ydata',[Baliza_1(2);Baliza_1(2)+dx_dy(2,1)]); 
            set(h(7),'xdata',[Baliza_2(1);Baliza_2(1)+dx_dy(1,2)]); set(h(7),'ydata',[Baliza_2(2);Baliza_2(2)+dx_dy(2,2)]); 
            set(h(9),'xdata',[Baliza_3(1);Baliza_3(1)+dx_dy(1,3)]); set(h(9),'ydata',[Baliza_3(2);Baliza_3(2)+dx_dy(2,3)]); 
            set(h(11),'xdata',[Baliza_4(1);Baliza_4(1)+dx_dy(1,4)]); set(h(11),'ydata',[Baliza_4(2);Baliza_4(2)+dx_dy(2,4)]);
            set(h(13),'xdata',[Baliza_5(1);Baliza_5(1)+dx_dy(1,5)]); set(h(13),'ydata',[Baliza_5(2);Baliza_5(2)+dx_dy(2,5)]);
            set(h(15),'xdata',[Baliza_6(1);Baliza_6(1)+dx_dy(1,6)]); set(h(15),'ydata',[Baliza_6(2);Baliza_6(2)+dx_dy(2,6)]);
            
            set(Robot(1),'xdata',punto(1,:));set(Robot(1),'ydata',punto(2,:));
            set(Robot(2),'xdata',ruedader1(1,:));set(Robot(2),'ydata',ruedader1(2,:));
            set(Robot(3),'xdata',ruedaizq1(1,:));set(Robot(3),'ydata',ruedaizq1(2,:));
            set(Robot(4),'xdata',Lateral_sup1(1,:));set(Robot(4),'ydata',Lateral_sup1(2,:));
            set(Robot(5),'xdata',Lateral_inf1(1,:));set(Robot(5),'ydata',Lateral_inf1(2,:));
            set(Robot(6),'xdata',trasera(1,:));set(Robot(6),'ydata',trasera(2,:));
            set(Robot(7),'xdata',linea_abajo1(1,:));set(Robot(7),'ydata',linea_abajo1(2,:));
            set(Robot(8),'xdata',linea_arriba1(1,:));set(Robot(8),'ydata',linea_arriba1(2,:));
           
            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(X_mod_ruido(1,k),X_mod_ruido(2,k),X_mod_ruido(3,k),Radio);
            
            set(Robot_real(1),'xdata',punto(1,:));set(Robot_real(1),'ydata',punto(2,:));
            set(Robot_real(2),'xdata',ruedader1(1,:));set(Robot_real(2),'ydata',ruedader1(2,:));
            set(Robot_real(3),'xdata',ruedaizq1(1,:));set(Robot_real(3),'ydata',ruedaizq1(2,:));
            set(Robot_real(4),'xdata',Lateral_sup1(1,:));set(Robot_real(4),'ydata',Lateral_sup1(2,:));
            set(Robot_real(5),'xdata',Lateral_inf1(1,:));set(Robot_real(5),'ydata',Lateral_inf1(2,:));
            set(Robot_real(6),'xdata',trasera(1,:));set(Robot_real(6),'ydata',trasera(2,:));
            set(Robot_real(7),'xdata',linea_abajo1(1,:));set(Robot_real(7),'ydata',linea_abajo1(2,:));
            set(Robot_real(8),'xdata',linea_arriba1(1,:));set(Robot_real(8),'ydata',linea_arriba1(2,:));
            
            %% Actualización del filtro extendido de Kalman
            %Usamos el FILTRO EXTENDIDO DE KALMAN.
            
            %Actualizamos la información del filtro.
            M = MM_ekf(:,k);
            Media = MM_ekf(:,k);
            P = PP_ekf(:,:,k);
            EST = MM_ekf(:,1:k);
            
            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
          
            % Intervalo de confianza

            cc = repmat([0;0],1,length(tt)) + ...
             intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
         
            cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
            % Actualizamos los gráficos correspondientes al EKF.
            set(h_2(1),'xdata',Media(1)); set(h_2(1),'ydata',Media(2));
            set(h_2(2),'xdata',EST(1,:)); set(h_2(2),'ydata',EST(2,:)); 
            set(h_2(3),'xdata',cc(1,:)); set(h_2(3),'ydata',cc(2,:)); 
             
            set(Robot_ekf(1),'xdata',punto(1,:));set(Robot_ekf(1),'ydata',punto(2,:));
            set(Robot_ekf(2),'xdata',ruedader1(1,:));set(Robot_ekf(2),'ydata',ruedader1(2,:));
            set(Robot_ekf(3),'xdata',ruedaizq1(1,:));set(Robot_ekf(3),'ydata',ruedaizq1(2,:));
            set(Robot_ekf(4),'xdata',Lateral_sup1(1,:));set(Robot_ekf(4),'ydata',Lateral_sup1(2,:));
            set(Robot_ekf(5),'xdata',Lateral_inf1(1,:));set(Robot_ekf(5),'ydata',Lateral_inf1(2,:));
            set(Robot_ekf(6),'xdata',trasera(1,:));set(Robot_ekf(6),'ydata',trasera(2,:));
            set(Robot_ekf(7),'xdata',linea_abajo1(1,:));set(Robot_ekf(7),'ydata',linea_abajo1(2,:));
            set(Robot_ekf(8),'xdata',linea_arriba1(1,:));set(Robot_ekf(8),'ydata',linea_arriba1(2,:));
           
            %% Actualización de gráficos del UKF
            %Usamos el UKF
            
            %Actualizamos la información del filtro.
            M = MM_ukf(:,k);
            Media = MM_ukf(:,k);
            P = PP_ukf(:,:,k);
            EST = MM_ukf(:,1:k);
            
            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
          
            % Intervalo de confianza

            cc = repmat([0;0],1,length(tt)) + ...
             intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
         
            cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
            
            % Actualizamos los gráficos correspondientes al EKF.
            set(h_3(1),'xdata',Media(1)); set(h_3(1),'ydata',Media(2));
            set(h_3(2),'xdata',EST(1,:)); set(h_3(2),'ydata',EST(2,:)); 
            set(h_3(3),'xdata',cc(1,:)); set(h_3(3),'ydata',cc(2,:)); 
             
            set(Robot_ukf(1),'xdata',punto(1,:));set(Robot_ukf(1),'ydata',punto(2,:));
            set(Robot_ukf(2),'xdata',ruedader1(1,:));set(Robot_ukf(2),'ydata',ruedader1(2,:));
            set(Robot_ukf(3),'xdata',ruedaizq1(1,:));set(Robot_ukf(3),'ydata',ruedaizq1(2,:));
            set(Robot_ukf(4),'xdata',Lateral_sup1(1,:));set(Robot_ukf(4),'ydata',Lateral_sup1(2,:));
            set(Robot_ukf(5),'xdata',Lateral_inf1(1,:));set(Robot_ukf(5),'ydata',Lateral_inf1(2,:));
            set(Robot_ukf(6),'xdata',trasera(1,:));set(Robot_ukf(6),'ydata',trasera(2,:));
            set(Robot_ukf(7),'xdata',linea_abajo1(1,:));set(Robot_ukf(7),'ydata',linea_abajo1(2,:));
            set(Robot_ukf(8),'xdata',linea_arriba1(1,:));set(Robot_ukf(8),'ydata',linea_arriba1(2,:));
            
            %% Actualización de gráficos del CKF
            %Usamos el UKF
            
            %Actualizamos la información del filtro.
            M = MM_ckf(:,k);
            Media = MM_ckf(:,k);
            P = PP_ckf(:,:,k);
            EST = MM_ckf(:,1:k);
            
            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
          
            % Intervalo de confianza

            cc = repmat([0;0],1,length(tt)) + ...
             intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
         
            cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
            
            % Actualizamos los gráficos correspondientes al EKF.
            set(h_4(1),'xdata',Media(1)); set(h_4(1),'ydata',Media(2));
            set(h_4(2),'xdata',EST(1,:)); set(h_4(2),'ydata',EST(2,:)); 
            set(h_4(3),'xdata',cc(1,:)); set(h_4(3),'ydata',cc(2,:)); 
             
            set(Robot_ckf(1),'xdata',punto(1,:));set(Robot_ckf(1),'ydata',punto(2,:));
            set(Robot_ckf(2),'xdata',ruedader1(1,:));set(Robot_ckf(2),'ydata',ruedader1(2,:));
            set(Robot_ckf(3),'xdata',ruedaizq1(1,:));set(Robot_ckf(3),'ydata',ruedaizq1(2,:));
            set(Robot_ckf(4),'xdata',Lateral_sup1(1,:));set(Robot_ckf(4),'ydata',Lateral_sup1(2,:));
            set(Robot_ckf(5),'xdata',Lateral_inf1(1,:));set(Robot_ckf(5),'ydata',Lateral_inf1(2,:));
            set(Robot_ckf(6),'xdata',trasera(1,:));set(Robot_ckf(6),'ydata',trasera(2,:));
            set(Robot_ckf(7),'xdata',linea_abajo1(1,:));set(Robot_ckf(7),'ydata',linea_abajo1(2,:));
            set(Robot_ckf(8),'xdata',linea_arriba1(1,:));set(Robot_ckf(8),'ydata',linea_arriba1(2,:));
            %Definimos la velocidad para los pasos en la simulación.
            pause(velocidad_simulacion)
         
          end
end

%% Mostrar resultados
disp(' ')
disp('Simulación finalizada')
disp(' ')

fprintf('  El RMS (KF-ROJO) de las coordenadas x e y, \n es de: %f metros \n',kf_rmse)
disp(' ')
fprintf('  El RMS (EKF-CIAN) de las coordenadas x e y, \n es de: %f metros \n',ekf_rmse)
disp(' ')
fprintf('  El RMS (UKF-VERDE) de las coordenadas x e y, \n es de: %f metros \n',ukf_rmse)
disp(' ')
fprintf('  El RMS (CKF-NEGRO) de las coordenadas x e y, \n es de: %f metros \n',ckf_rmse)
disp(' ')
%Creamos un bloque de depuración para comprobar que las dimensiones de las
%matrices son las correctas y que las variables tienen el valor que
%esperamos obtener

%% Zona de depuración de variables
 if depuracion == true 
     disp(' ')
     disp('MOSTRAMOS LAS VARIABLES DE DEPURACIÓN:')
     disp(' ')
     fprintf('El tamaño de X_mod es: %f \n',size(X_mod(1,:)))
     fprintf('El tamaño de Y es: %f \n',size(Y(1,:)))
     fprintf('El tamaño de MM es: %f \n',size(MM(1,:)))
     A
     Q
     
 end
 
 
 
      
      
 
 
 



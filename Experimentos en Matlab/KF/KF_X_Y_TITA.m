% Scripts para la localización de un robot en un entorno de 2 dimensiones
% usando el filtro de Kalman clásico.

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

%EXPLICACION -----------------------------------------------------------
% Script para la localización de un robot en un 
%entorno de 2 dimensiones. Las variables que definirán la pose del robot
%serán la x e y para las coordenadas y TITA para orientación. Usaremos el
%filtro clásico de kalman para realizar la localización de nuestro robot.
%Unicamente podremos sacar en claro cuales son las coordenadas X e Y del
%robot ya que no tendremos clara su orientación (solo nos resulta posible 
%medir la orientación usando la información proporcionada por la odometría).
%-----------------------------------------------------------------------

rng(1); %Colocamos la misma semilla para que el script siempre se ejecute en las mismas condiciones
clear % Eliminamos todas las variables que guardamos en el workspace
clc;
clf; %Limpiamos el workSpace y las figuras 

%Establecemos la velocidad para la simulación del experimento

velocidad_simulacion = 0.01 ;
mostrar_simulacion = true;
preguntar_simular = true;
depuracion = false ;
steps = 1;
Radio = 0.3; %Radio del dibujo
radioruedas = 0.1; %definimos el radio de las ruedas de nuestro robot
Intervalo_conf = 50 ;

%Iniciamos el tiempo de simulación

t = 0;
resolucion = 0.1 ;

%Imprimimos algunos mensajes informativos en el workspace

disp('Seguimiento de un robot móvil usando el KF')
disp('  Autor: Iván Rodríguez Méndez')
disp('  Basado en la toolbox de Simo Särkkä')

%Creamos la trayectoria que seguirá el robot y además colocamos las balizas
%para poder triangular su posición.
disp(' ')
disp('PARÁMETROS DE SIMULACIÓN:')

Baliza_1 = [1;5] ;
Baliza_2 = [6;4];
Baliza_3 = [10;10];
Baliza_4 = [8;13];

Balizas = [Baliza_1 Baliza_2 Baliza_3 Baliza_4]

sd_baliza = 0.05 ; %Parámetro de afectación de ruido de nuestro robot
sd_odometria = 0.00005 ; %Parámetro de ruido que aplicamos a la odometría para simular el deslizamiento
dt = resolucion ; %Diferencial de tiempo por el que realizamos nuestra simulación

fprintf(' -> La afectación del ruido en las balizas es %f \n',sd_baliza)
fprintf(' -> La afectación del ruido en la odometría es %f \n',sd_odometria)
fprintf(' -> El tiempo de muestreo es %f segundos \n',dt)
fprintf(' -> El radio de las ruedas es %f metros \n',radioruedas)

%Definimos el modelo de medida (Únicamente mediremos las coordenadas X e Y
%de nuestro robot. Aunque lo haremos directamente partiendo de la ecuación
%podríamos usar cualquier método de triangulación y aseguraríamos el mismo
%resultado.

H = eye(3);
B = eye(3);

%Creamos la trayectoria que nuestro robot deberá seguir 

 t = 0 ; %Inicializamos el tiempo de muestreo
 x_mod = [0;0;0];
 X_mod = [];
 Y_r = []; %Inicializamos el vector que guarda las rotaciones para la representación gráfica
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

Epsilon = 0.3; %Umbral de distancia máxima permitida (error)
V = V_LINEAL;
W = V_ANGULAR*pi/(180);

%Definimos el primer punto de la trayectoria que vamos a
%seguir, lógicamente el primer punto es la posición del robot.
trayectoria = [x_inicial;y_inicial;alfa_inicial];

%Calculamos los objetivos que tenemos que alcanzar, es decir
%las coordenadas que tienen para poder alcanzarlas.
disp('Calculando la localización de los objetivos...');
objetivos = [0 0;1 1 ; 2 5 ; 5 3 ; 8 9 ; 10 15 ]';
disp('posición de los objetivos calculada');

%Definimos un indice para recorrer el vector de objetivos (para
%conocer cual es el objetivo que queremos buscar)
obj_actual = 1;

%Inicializamos los vectores de medida que usaremos en el
%recorrido para la simulación.
%% Inicialización de parámetros de vectores
y = [];
y_bal = [];
Y_r = [];
Y_real = [];
x = [];
X = [];
odom = [];
Odom = [];
Tiempo = [];
tiempo = 0;
U_t= [];
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

        trayectoria(3,k) = pose(3,1) + w*dt ;
        trayectoria(1,k) = pose(1,1) + v*cos(trayectoria(3,k))*dt;
        trayectoria(2,k) = pose(2,1) + v*sin(trayectoria(3,k))*dt ;
        
        X_mod_ruido(3,k) = trayectoria(3,k) + randn*sd_odometria*0.1;
        X_mod_ruido(1,k) = trayectoria(1,k) + randn*sd_odometria;
        X_mod_ruido(2,k) = trayectoria(2,k) + randn*sd_odometria;
        
        u_t = odometry_motion(trayectoria(:,k),pose);
        Y(:,k) = trayectoria(:,k);
        
        [y_r] = Medidas_rot(Balizas,trayectoria(:,k),0);
        %Imprimimos dos variables de interes para saber en que
        %paso del bucle nos encontramos y que objetivo
        %pretendemos alcanzar.
        tiempo = tiempo +dt;
        U_t = [U_t u_t];
        Y_r = [Y_r y_r];
        Tiempo = [Tiempo tiempo];
        obj_actual
        Obj_real = obj_actual;
        size(trayectoria,2)
        k= k +1; %Indices que usamos para Kalman
       
    end
    obj_actual = obj_actual+1; %Cuando salimos del bucle pasamos a buscar el siguiente objetivo.
end

plot(trayectoria(1,:),trayectoria(2,:));
hold on
Representar_obj(objetivos);

 

 M = [0;0;0]; %Suponemos un estado inicial a priori totalmente desconocido.
 P = diag([0 0 0]); %Suponemos una covarianza también desconocida a priori.
 R = sd_baliza^2 + sd_odometria^2; 
 
 qx = 0.1 ;
 qy = 0.1 ;
 
 A = [1 dt 0;
      0 1 0;
      0 0 1]; %Tanto A como Q pueden ser opcionales ya que si no los ponemos la función utilizaría parámetros por defecto.
  
 Q = [(1/3)*(dt^3)*qx (1/2)*(dt^2)*qx 0;
      (1/2)*(dt^2)*qx (dt)*qx         0;
               0         0       dt*qy];
      

 %Seguimiento y animación de nuestro robot
 
 
 MM = zeros(size(M,1),size(Y,2));
 PP = zeros(size(M,1),size(M,1),size(Y,2));

 
 if preguntar_simular == true 
     disp('<Pulsa una tecla para mostrar la simulación>')
     disp(' ')
     Representar_obj(objetivos);
     plot(X_mod_ruido(1,:),X_mod_ruido(2,:),'b')
     x_min = min(objetivos(1,:))-2;
     x_max = max(objetivos(1,:))*2;
     y_min =  min(objetivos(2,:))-2;
     y_max =  max(objetivos(2,:))*2;
     axis([x_min x_max y_min y_max ]);
     title('PULSA UNA TECLA PARA COMENZAR CON LA SIMULACIÓN')
     pause()
 end
 
 disp(' ')
 disp('¡Comienza la simulación!')

 
 
 for k=1:size(Y,2) %La dimensión del bucle es tan grande como la trayectoria o el número de medidas que hemos tomado durante esta
     % Filtrado con el KF
     if k == 1 
         [M,P] = kf_predict(M,P,A,Q);
         [M,P] = kf_update(M,P,Y(:,k),H,R*eye(3));
         MM(:,k) = M ;
         PP(:,:,k) = P;
     else
         [M,P] = kf_predict(M,P,A,Q,B,U_t(:,k));
         [M,P] = kf_update(M,P,Y(:,k),H,R*eye(3));
         MM(:,k) = M ;
         PP(:,:,k) = P;
     end
     
     
 end

 ekf_rmse = sqrt(mean((X_mod_ruido(1,:)-MM(1,:)).^2+(X_mod_ruido(2,:)-MM(2,:)).^2)); %Calculamos el error cuadrático medio entre la trayectoria estimada y la trayectoria real
 
if mostrar_simulacion == true 
    %Representaciones gráficas de los resultados de la estimación y el
    %seguimiento de nuestro Robot

      M = MM(:,1);  
      P = PP(:,:,1);
      EST = M;
      [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio); %Usamos una función para calcular en la posición estimada el dibujo del Robot.
      
      %Calculamos el intervalo de confianza
      tt = (0:0.01:1)*2*pi;
      cc = repmat([0;0],1,length(tt)) + ...
              Intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)]; %Calculamos el intervalo de confianza
          
      cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
      %Representamos la trayectoria real, la posición del robot, y el resto
      %de parámetros que nos interesan.
      
      h = plot(X_mod_ruido(1,:),X_mod_ruido(2,:),'b-',... %Dibuja la línea  que reprensenta la trayectoria real
               M(1),M(2),'bo',... %representa la posición estimada
               EST(1,:),EST(2,:),'r--',... %Dibujamos la estimación
               cc(1,:),cc(2,:),'m-',... %Intervalo de confianza
               Baliza_1(1),Baliza_1(2),'g--',... %Medida desde la baliza
               Baliza_1(1),Baliza_1(2),'k^',... %Dibuja el triángulo correspondiente a la baliza
               Baliza_2(1),Baliza_2(2),'g--',... %Dibuja la medida desde la baliza
               Baliza_2(1),Baliza_2(2),'k^',...
               Baliza_3(1),Baliza_3(2),'g--',... 
               Baliza_3(1),Baliza_3(2),'k^',... 
               Baliza_4(1),Baliza_4(2),'g--',... %Medida desde la baliza
               Baliza_4(1),Baliza_4(2),'k^'); %Dibuja el triángulo correspondiente a la baliza
      title('Localización usando KF.')
      
      x_min = min(objetivos(1,:))-2;
      x_max = max(objetivos(1,:))*2;
      y_min =  min(objetivos(2,:))-2;
      y_max =  max(objetivos(2,:))*2;
      axis([x_min x_max y_min y_max ]);
      hold on
      
      %Colocamos unas marcas de texto en la representación
      
      text(x_inicial-2,y_inicial-2,'SALIDA')
      texto = 'Baliza ';
      b = 1;
      a = texto;
      for tex_i=1:size(Balizas,2)
        text(Balizas(1,tex_i)-2,Balizas(2,tex_i)-2,a)
        b = num2str(b);
        a = [texto b];
        b = str2num(b);
        b = b+1;
        
      end
      
      
      %Representamos la estimación del robot
      
      Robot = plot(punto(1,:),punto(2,:),'r--',...
                   ruedader1(1,:),ruedader1(2,:),'g',...
                   ruedaizq1(1,:),ruedaizq1(2,:),'g',...
                   Lateral_sup1(1,:),Lateral_sup1(2,:),'r--',...
                   Lateral_inf1(1,:),Lateral_inf1(2,:),'r--',...
                   trasera(1,:),trasera(2,:),'r--',...
                   linea_abajo1(1,:),linea_abajo1(2,:),'r--',...
                   linea_arriba1(1,:),linea_arriba1(2,:),'r--');
       hold on
       
       %Calculamos la representación del robot real y la colocamos en un
       %plot.
       
       [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(X_mod_ruido(1,1),X_mod_ruido(2,1),X_mod_ruido(3,1),Radio);
       
       Robot_real = plot(punto(1,:),punto(2,:),'b',...
                   ruedader1(1,:),ruedader1(2,:),'g',...
                   ruedaizq1(1,:),ruedaizq1(2,:),'g',...
                   Lateral_sup1(1,:),Lateral_sup1(2,:),'b',...
                   Lateral_inf1(1,:),Lateral_inf1(2,:),'b',...
                   trasera(1,:),trasera(2,:),'b',...
                   linea_abajo1(1,:),linea_abajo1(2,:),'b',...
                   linea_arriba1(1,:),linea_arriba1(2,:),'b');
      
       
        EST = [];
        for k=1:steps:size(Y,2)
           
            %Calculamos la longitud de la representación del impacto sobre
            %el robot.
            [ len ] = Dist_medida(Y(:,k),Balizas,dist_max);
            
            %Guardamos la información de la media y la covarianza en el
            %un instante dado (viene dado por cada paso del bucle)
            M = MM(:,k);
            P = PP(:,:,k);
            EST = MM(:,1:k);

            %Calculamos la representación visual del robot para el momento
            %dado.
            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
          
            % Intervalo de confianza

            cc = repmat([0;0],1,length(tt)) + ...
            Intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];

            cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
            % Direcciones de medida de las balizas (dirección de impactos)
            dx_dy = Dir_medida(len,Y_r(:,k));
            
            % Actualizamos el plot. Con esto conseguimos simular el
            % movimiento del robot.
            
            %Actualizamos los impactos de las balizas y los intervalos de
            %confianza
            
            set(h(2),'xdata',M(1)); set(h(2),'ydata',M(2));
            set(h(3),'xdata',EST(1,:)); set(h(3),'ydata',EST(2,:)); 
            set(h(4),'xdata',cc(1,:)); set(h(4),'ydata',cc(2,:)); 
            set(h(5),'xdata',[Baliza_1(1);Baliza_1(1)+dx_dy(1,1)]); set(h(5),'ydata',[Baliza_1(2);Baliza_1(2)+dx_dy(2,1)]); 
            set(h(7),'xdata',[Baliza_2(1);Baliza_2(1)+dx_dy(1,2)]); set(h(7),'ydata',[Baliza_2(2);Baliza_2(2)+dx_dy(2,2)]); 
            set(h(9),'xdata',[Baliza_3(1);Baliza_3(1)+dx_dy(1,3)]); set(h(9),'ydata',[Baliza_3(2);Baliza_3(2)+dx_dy(2,3)]); 
            set(h(11),'xdata',[Baliza_4(1);Baliza_4(1)+dx_dy(1,4)]); set(h(11),'ydata',[Baliza_4(2);Baliza_4(2)+dx_dy(2,4)]); 
            %Actualizamos la representación del robot estimado
            
            set(Robot(1),'xdata',punto(1,:));set(Robot(1),'ydata',punto(2,:));
            set(Robot(2),'xdata',ruedader1(1,:));set(Robot(2),'ydata',ruedader1(2,:));
            set(Robot(3),'xdata',ruedaizq1(1,:));set(Robot(3),'ydata',ruedaizq1(2,:));
            set(Robot(4),'xdata',Lateral_sup1(1,:));set(Robot(4),'ydata',Lateral_sup1(2,:));
            set(Robot(5),'xdata',Lateral_inf1(1,:));set(Robot(5),'ydata',Lateral_inf1(2,:));
            set(Robot(6),'xdata',trasera(1,:));set(Robot(6),'ydata',trasera(2,:));
            set(Robot(7),'xdata',linea_abajo1(1,:));set(Robot(7),'ydata',linea_abajo1(2,:));
            set(Robot(8),'xdata',linea_arriba1(1,:));set(Robot(8),'ydata',linea_arriba1(2,:));
           
            
            %Actualizamos la representación del robot real.
            
            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(X_mod_ruido(1,k),X_mod_ruido(2,k),X_mod_ruido(3,k),Radio);
            
            set(Robot_real(1),'xdata',punto(1,:));set(Robot_real(1),'ydata',punto(2,:));
            set(Robot_real(2),'xdata',ruedader1(1,:));set(Robot_real(2),'ydata',ruedader1(2,:));
            set(Robot_real(3),'xdata',ruedaizq1(1,:));set(Robot_real(3),'ydata',ruedaizq1(2,:));
            set(Robot_real(4),'xdata',Lateral_sup1(1,:));set(Robot_real(4),'ydata',Lateral_sup1(2,:));
            set(Robot_real(5),'xdata',Lateral_inf1(1,:));set(Robot_real(5),'ydata',Lateral_inf1(2,:));
            set(Robot_real(6),'xdata',trasera(1,:));set(Robot_real(6),'ydata',trasera(2,:));
            set(Robot_real(7),'xdata',linea_abajo1(1,:));set(Robot_real(7),'ydata',linea_abajo1(2,:));
            set(Robot_real(8),'xdata',linea_arriba1(1,:));set(Robot_real(8),'ydata',linea_arriba1(2,:));
            
            %Damos una velocidad a la simulación para que el robot vaya más
            %rápido o más lento.
            
            pause(velocidad_simulacion)
         
       end
end
      
disp(' ')
disp('Simulación finalizada')
disp(' ')

fprintf('  El RMS de las coordenadas x e y, es de %f \n',ekf_rmse)
disp(' ')

%Creamos un bloque de depuración para comprobar que las dimensiones de las
%matrices son las correctas y que las variables tienen el valor que
%esperamos obtener

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
 
 

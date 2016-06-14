% Script de test para el generador de trayectorias usando el EKF

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.


rng(1); %Colocamos la misma semilla para que el script siempre se ejecute en las mismas condiciones
clear % Eliminamos todas las variables que guardamos en el workspace
clc;
clf; %Limpiamos el workSpace y las figuras 

%Establecemos la velocidad para la simulación del experimento

velocidad_simulacion = 0.01 ;
mostrar_simulacion = true;
preguntar_simular = true;
depuracion = false ;
steps = 1; %Pasos usados por el bucle para la represetanción gráfica
len = 100 ; %Tamaño asignado para mostrar las mediciones por parte de las balizas
Radio = 0.35 ; %Radio del dibujo
radioruedas = Radio/6 ; %definimos el radio de las ruedas de nuestro robot
intervalo_conf = 50 ; %Definimos el tamaño del intervalo de confianza en la representación.

%Importamos el modelo de medida y sus derivadas (en este caso medimos la
%distancia)

h_func = @bot_dist_h;


%Imprimimos algunos mensajes informativos en el workspace

disp('Seguimiento de un robot móvil usando el UKF')      
disp('  Autor: Iván Rodríguez Méndez')
disp('  Basado en la toolbox de Simo Särkkä')

%Creamos la trayectoria que seguirá el robot y además colocamos las balizas
%para poder calcular su posición.

disp(' ')
disp('PARÁMETROS DE SIMULACIÓN:')

Baliza_1 = [-10;25] ;
Baliza_2 = [-10;-5];

fprintf(' -> La posición de la primera baliza \n    es X= %f  e Y= %f \n',Baliza_1(1,1),Baliza_1(2,1))
fprintf(' -> La posición de la segunda baliza \n    es X= %f  e Y= %f \n',Baliza_2(1,1),Baliza_2(2,1))

sd_baliza = 0.05 ; %Parámetro de afectación de ruido de nuestro robot
sd_odometria = 0.05 ; %Parámetro de ruido que aplicamos a la odometría 
dt = 0.01 ; %Diferencial de tiempo por el que realizamos nuestra simulación

fprintf(' -> La afectación del ruido es %f (varianza) \n',sd_baliza)
fprintf(' -> La afectación del ruido es %f (varianza) \n',sd_odometria)
fprintf(' -> El tiempo de muestreo es %f segundos \n',dt)
fprintf(' -> El radio de las ruedas es %f metros \n',radioruedas)
%Creamos la trayectoria que nuestro robot deberá seguir 

 Y = []; %Inicialización del vector de medidas
 Y_r = []; %Inicializamos el vector que guarda las rotaciones
 
 %Generamos las velocidades lineales y angulares de nuestro robot
V_LINEAL = 1; %Velocidad máxima
V_ANGULAR = 8; %Velocidad de rotación máxima
x_inicial = 0;
y_inicial = 0;
alfa_inicial = 0;
tiempo = 0;
Tiempo = [];
espacio = 0;
pose = [x_inicial;y_inicial;alfa_inicial];
FPS = 15 ;
Epsilon = 0.01; %Umbral de distancia
V = V_LINEAL;
W = V_ANGULAR*pi/(180) ;
trayectoria = [x_inicial;y_inicial;alfa_inicial];

velocidad_lineal = [0];
velocidad_angular = [0];

objetivos = [5 5 5;
             0 10 20 ;
             0 pi/2 pi/2];
obj_actual = 1;
mov = 1;
angulo = 0;

for i=1:size(objetivos,2)
    while norm(trayectoria(:,size(trayectoria,2))-objetivos(:,obj_actual)) > Epsilon && size(trayectoria,2) <= 1000 
        pose = trayectoria(:,size(trayectoria,2));
        
        %Calculamos la diferencia en el ángulo
        b = atan2(sin(pose(3,1)),cos(pose(3,1)));
        a = atan2(sin(objetivos(3,obj_actual)),cos(objetivos(3,obj_actual)));
        
        d1 = a-b;
        d2 = 2*pi - abs(d1);
        
        if d1 > 0
            d2 = d2*(-1);
        end
        if (abs(d1) < abs(d2))
            angulo = d1;
        else 
            angulo = d2;
        end
        
       w = angulo;
        
        if w > W
            w = W;
        end
        if w < -W
            w = -W;
        end 
        v = norm(pose(1:2)-objetivos(1:2,obj_actual));
        
        if (v > V)
            v = V;
        end
        if (v < 0)
            v = 0;
        end
        
        if abs(w) > 0.01
            v = 0;
        end
        
   
        trayectoria(3,mov) = pose(3,1) + w*0.1;
        trayectoria(1,mov) = pose(1,1) + v*cos(trayectoria(3,mov))*0.1;
        trayectoria(2,mov) = pose(2,1) + v*sin(trayectoria(3,mov))*0.1;
        
        tiempo = tiempo + 0.1;
        Tiempo = [Tiempo tiempo]; %Usamos este vector por si quisieramos representar algo con respecto al tiempo.
        espacio = espacio + v;
        mov = mov+1;
    end
    obj_actual = obj_actual+1;
end
        
plot(trayectoria(1,:),trayectoria(2,:))
plot(Tiempo,trayectoria(3,:))

x = [];
x_mod = [];
X_mod = [];
X_mod_ruido = [];
X = [];
U_t = [];
u_t = [];
%Iniciamos el tiempo de simulación
t = 0;
resolucion = dt ;
T_final = (tiempo-0.1)/10 ; %Lo colocamos así para que sea consecuente con la longitud de nuestro vector

%Inicializamos el índice para recorrer el buble

i = 1; %Inicializamos el índice con el que trabajaremos dentro del buble.

 
 for t=0:resolucion:T_final %Creamos la trayectoria que el robot seguirá 
     %Vamos a crear la trayectoria de manera digital para poder
     %entender mejor todo el funcionamiento de nuestro sistema
     
     %Generaremos el vector de estados
     
         x_mod(:,i) = trayectoria(:,i);
         X_mod_ruido = x_mod;
         
    
         if i > 1
         
             x_mod_ruido = X_mod_ruido(:,i-1);
             u_t = odometry_motion(X_mod_ruido(:,i-1),X_mod_ruido(:,i));
         
         else 
             x_mod_ruido = X_mod_ruido(:,i);
         end
         
  
     %Calculamos las medidas usando el vector de estados del modo continuo
     
      y_r1 = atan2(x_mod_ruido(2)-Baliza_1(2), x_mod_ruido(1)-Baliza_1(1)) ; %Creamos estos vectores para simular el track visualmente
      y_r2 = atan2(x_mod_ruido(2)-Baliza_2(2), x_mod_ruido(1)-Baliza_2(1)) ; %Creamos estos vectores para simular el track visualmente
     
      %Medidas para el filtro extendido (usando la distancia).
      y1 = sqrt((x_mod_ruido(1)-Baliza_1(1)).^2 + (x_mod_ruido(2)-Baliza_1(2)).^2)+sd_baliza*randn ;
      y2 = sqrt((x_mod_ruido(1)-Baliza_2(1)).^2 + (x_mod_ruido(2)-Baliza_2(2)).^2)+sd_baliza*randn ;

    %Guardamos la información generada
     
     Y = [Y [y1;y2]];
     U_t = [U_t u_t];
     Y_r = [Y_r [y_r1;y_r2]];
     i = i+1; %Aumentamos el índice del bucle 
 end
 
 M_ukf = [0;0;0];
 P_ukf = diag([0.1 0.1 0.1]); %Inicializamos los vectores que contendrán la información inicial para del filtro clásico.
 
 R = sd_baliza^2 + sd_odometria^2; %Introducimos el ruido que tendrá en cuenta el filtro en las ecuaciones.
 
 %Construimos las matrices A y Q necesarias para el ciclo de predicción.
 qx = 0.9 ;
 qy = 0.9 ;
 
 A = [1 dt 0;
      0 1 dt;
      0 0 1];
  
 Q = [(1/3)*(dt^3)*qx (1/2)*(dt^2)*qx 0;
      (1/2)*(dt^2)*qx (dt)*qx         0;
               0         0       dt*qy];
      
 %Seguimiento y animación de nuestro robot

 MM_ukf = zeros(size(M_ukf,1),size(Y,2)); %Inicialicamos el vector que contendrá toda la información del filtro clásico.
 PP_ukf = zeros(size(M_ukf,1),size(M_ukf,1),size(Y,2));
 
 %Colocamos una condición para preguntar a la hora de realizar la
 %simulación o hacerla directamente.
 
 if preguntar_simular == true 
     
     disp('Pulsa una tecla para mostrar la simulación')
     disp(' ')
     plot(X_mod_ruido(1,:),X_mod_ruido(2,:),'b')
     axis([-30*Radio 80*Radio -30*Radio 80*Radio]);
     title('PULSA UNA TECLA PARA COMENZAR CON LA SIMULACIÓN')
     pause()
     
 end   
 
 disp(' ')
 disp('¡Comienza la simulación!')
 
 for k=1:size(Y,2) %La dimensión del bucle es tan grande como la trayectoria o el número de medidas
      if k == 1
          M_ukf = (trayectoria(:,k));
          [M_ukf,P_ukf] = ckf_predict(M_ukf,P_ukf,A,Q);
          [M_ukf,P_ukf] = ckf_update(M_ukf,P_ukf,Y(:,k),h_func,R*eye(2),[Baliza_1 Baliza_2]);
          MM_ukf(:,k) = M_ukf ;
          MM_ukf(3,k) = 0 ;
          PP_ukf(:,:,k) = P_ukf;
      else 
         M_ukf = (trayectoria(:,k)+M_ukf)/2;
          [M_ukf,P_ukf] = ckf_predict(M_ukf,P_ukf,A,Q);
          [M_ukf,P_ukf] = ckf_update(M_ukf,P_ukf,Y(:,k),h_func,R*eye(2),[Baliza_1 Baliza_2]);
          MM_ukf(:,k) = M_ukf ;
          MM_ukf(3,k) = trayectoria(3,k); %Imponemos que la rotación sea calculada de esta manera
          PP_ukf(:,:,k) = P_ukf;
      end
      
 end
 
 ukf_rmse = sqrt(mean((X_mod_ruido(1,:)-MM_ukf(1,:)).^2+(X_mod_ruido(2,:)-MM_ukf(2,:)).^2)); %Calculamos el error cuadrático medio de la posición estimada usando el filtro clásico de Kalman.
 
%Representamos la estimación obtenida para ambos filtros.
 
if mostrar_simulacion == true 
    %Representaciones gráficas de los resultados de la estimación 
    %Comenzamos con la representación del filtro clásico de Kalman.
    
    %FILTRO UKF.
      M = MM_ukf(:,1);  
      P = PP_ukf(:,:,1);
      EST = M;
      
      %Calculamos la representación para la estimación del primer vector de
      %estados
      [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
      
      tt = (0:0.01:1)*2*pi;
      %Calculamos el intervalo de confianza para conocer cual será la
      %propagración del error en nuestro robot.
      cc = repmat([0;0],1,length(tt)) + ...
              intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)]; %Calculamos el intervalo de confianza
      
      cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
      %Representamos la trayectoria real junto con la estimación del KF y
      %la posición de las balizas en el mapa.
      
      h = plot(X_mod_ruido(1,:),X_mod_ruido(2,:),'b-',... %Dibuja la línea  que reprensenta la trayectoria real
               M(1),M(2),'ro',... %representa la posición estimada
               EST(1,:),EST(2,:),'r--',... %Dibujamos la estimación
               cc(1,:),cc(2,:),'m-',... %Intervalo de confianza
               Baliza_1(1),Baliza_1(2),'g',... %Medida desde la baliza
               Baliza_1(1),Baliza_1(2),'k^',... %Dibuja el triángulo correspondiente a la baliza
               Baliza_2(1),Baliza_2(2),'g',... %Dibuja la medida desde la baliza
               Baliza_2(1),Baliza_2(2),'k^'); %Dibuja la segunda baliza
      legend('Location','NorthEast','Trayectoria real','Posición','Trayectoria estimada',...
             'Intervalo de confianza','Medidas de los sensores','Posicion de los sensores');
      title('Filtrado usando el UKF.')
      axis([-30*Radio 80*Radio -30*Radio 80*Radio]);
      
      hold on
      
      %Marcamos ciertos puntos del mapa con texto para poder identificarlos
      %mejor.
      
      text(x_inicial-1,y_inicial-1,'SALIDA')
      text(Baliza_1(1)-1,Baliza_1(2)-1,'BALIZA 1')
      text(Baliza_2(1)-1,Baliza_2(2)-1,'BALIZA 2')
      
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
      
       hold on
      
       
      EST = []; % Inicializamos un vector para guardar la estimación.
      
      %Usamos un bucle para poder ir actualizando las representaciones de
      %los robots.
          for k=1:steps:size(Y,2)
           
            %Definimos las longitudes de los impactos en el robot.
            len1 = Y(1,k);
            len2 = Y(2,k);
            %Actualizamos la información del FILTRO CLÁSICO.
            M = MM_ukf(:,k);
            Media = MM_ukf(:,k);
            P = PP_ukf(:,:,k);
            EST = MM_ukf(:,1:k);

            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
            
            % Intervalo de confianza.

            cc = repmat([0;0],1,length(tt)) + ...
             intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
         
            cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
            
            % Direcciones de los impactos.
            dx1 = len1*cos(Y_r(1,k));
            dy1 = len1*sin(Y_r(1,k));
            dx2 = len2*cos(Y_r(2,k));
            dy2 = len2*sin(Y_r(2,k));
           
            % Actualizamos las representaciones.
            set(h(2),'xdata',Media(1)); set(h(2),'ydata',Media(2));
            set(h(3),'xdata',EST(1,:)); set(h(3),'ydata',EST(2,:)); 
            set(h(4),'xdata',cc(1,:)); set(h(4),'ydata',cc(2,:)); 
            set(h(5),'xdata',[Baliza_1(1);Baliza_1(1)+dx1]); set(h(5),'ydata',[Baliza_1(2);Baliza_1(2)+dy1]); 
            set(h(7),'xdata',[Baliza_2(1);Baliza_2(1)+dx2]); set(h(7),'ydata',[Baliza_2(2);Baliza_2(2)+dy2]); 
                 
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
           
            %Definimos la velocidad para los pasos en la simulación.
            pause(velocidad_simulacion)
         
          end
end
      
disp(' ')
disp('Simulación finalizada')
disp(' ')

fprintf('  El RMS (UKF) de las coordenadas x e y, \n es de: %f metros \n',ukf_rmse)
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
 
 
 
      
      
 
 
 



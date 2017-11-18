% Copyright (C) 2017 Iván Rodríguez Méndez

function varargout = EKF_GUI(varargin)
% EKF_GUI MATLAB code for EKF_GUI.fig
%      EKF_GUI, by itself, creates a new EKF_GUI or raises the existing
%      singleton*.
%
%      H = EKF_GUI returns the handle to a new EKF_GUI or the handle to
%      the existing singleton*.
%
%      EKF_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in EKF_GUI.M with the given input arguments.
%
%      EKF_GUI('Property','Value',...) creates a new EKF_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before EKF_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to EKF_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help EKF_GUI

% Last Modified by GUIDE v2.5 18-Nov-2017 02:22:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @EKF_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @EKF_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before EKF_GUI is made visible.
function EKF_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to EKF_GUI (see VARARGIN)

% Choose default command line output for EKF_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes EKF_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = EKF_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in beginbutton.
function beginbutton_Callback(hObject, eventdata, handles)
% hObject    handle to beginbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rng(1); %Colocamos la misma semilla para que el script siempre se ejecute en las mismas condiciones

%Establecemos la velocidad para la simulación del experimento
pause on
hold off


velocidad_simulacion = 0.01 ;
mostrar_simulacion = true;
preguntar_simular = false;
depuracion = false ;
steps = 1;
Radio = 0.3 ; %Radio del dibujo
radioruedas = 0.1 ; %definimos el radio de las ruedas de nuestro robot
Intervalo_conf = 10;
cell = get(handles.Modelo_med,'Value');
cell = str2mat(cell);
Modelo_medida = cell;
%Importamos el modelo de medida y sus derivadas

if Modelo_medida == 1 %Modelo de medida de distancia
    h_func = @bot_dist_h;
    dh_dx_func = @bot_dist_dh;
end
if Modelo_medida == 2 %Modelo de medida de rotaciones
    h_func = @bot_h;
    dh_dx_func = @bot_dh_dx;
end


%Imprimimos algunos mensajes informativos en el workspace

disp('Seguimiento de un robot móvil usando el EKF')
disp('  Autor: Iván Rodríguez Méndez')
disp('  Basado en la toolbox de Simo Särkkä')

%Creamos la trayectoria que seguirá el robot y además colocamos las balizas
%para poder triangular su posición.
disp(' ')
disp('PARÁMETROS DE SIMULACIÓN:')

balizas=get(handles.uitable4,'Data');
balizas = cell2mat(balizas);

Baliza_1 = [balizas(1,1);balizas(1,2)] ;
Baliza_2 = [balizas(2,1);balizas(2,2)];
Baliza_3 = [balizas(3,1);balizas(3,2)];
Baliza_4 = [balizas(4,1);balizas(4,2)];
Baliza_5 = [balizas(5,1);balizas(5,2)];

Balizas = [Baliza_1 Baliza_2 Baliza_3 Baliza_4 Baliza_5] %Guardamos toda la informacion de las balizas

fprintf(' -> La posición de la primera baliza \n    es X= %f  e Y= %f \n',Baliza_1(1,1),Baliza_1(2,1))
fprintf(' -> La posición de la segunda baliza \n    es X= %f  e Y= %f \n',Baliza_2(1,1),Baliza_2(2,1))

sd_baliza = str2double(get(handles.ruido_med,'String')); %Parámetro de afectación de ruido de nuestro robot
sd_odometria = str2double(get(handles.ruido_odo,'String')) ; %Parámetro de ruido que aplicamos a la odometría 
dt = 0.1 ; %Diferencial de tiempo por el que realizamos nuestra simulación

fprintf(' -> La afectación del ruido es %f \n',sd_baliza)
fprintf(' -> La afectación del ruido es %f \n',sd_odometria)
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
dist_max = str2double(get(handles.dist_max,'String')) ; %distancia maxima que es capaz de medir el telemetro.
clc
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

datos=get(handles.uitable2,'Data');
datos = cell2mat(datos);
objetivos = zeros(size(datos));
for y=1:size(datos,1)
    objetivos(y,:) = datos(y,:);
end
objetivos = objetivos';
    
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

clc;
disp('Trayectoria generada con éxito');
plot(trayectoria(1,:),trayectoria(2,:));
hold on
Representar_obj(objetivos); %Representamos la localización de los objetivos en la escena.
 
 M = [0;0;0];
 P = diag([0 0 0 ]);
 R = sd_baliza^2 + sd_odometria^2; 
 
 qx = 0.1 ;
 qy = 0.1 ;
 
 A = [1 dt 0;
      0 1 0;
      0 0 1];
  
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
     x_max = max(objetivos(1,:))+2;
     y_min =  min(objetivos(2,:))-2;
     y_max =  max(objetivos(2,:))+2;
     axis([x_min x_max y_min y_max ]);
     title('PULSA UNA TECLA PARA COMENZAR CON LA SIMULACIÓN')
     pause()
 end
 
 disp(' ')
 disp('¡Comienza la simulación!')
 Y_completo = [];
 for k=1:size(Y,2) %La dimensión del bucle es tan grande como la trayectoria o el número de medidas
     % Seguimiento con el EKF
      
     [M,P] = ekf_predict1(M,P,A,Q,Odom(:,k));
     [Y_adap,Balizas_adap,y_completo] = Limitar_medida(Y(:,k),Y_r(:,k),Modelo_medida,Balizas,dist_max);
     [M,P] = ekf_update1(M,P,Y_adap,dh_dx_func,R*eye(size(Balizas_adap,2)),h_func,[],Balizas_adap);
     MM(:,k) = M ;
     PP(:,:,k) = P;
     Y_completo = [Y_completo y_completo];
     
 end

 ekf_rmse = sqrt(mean((X_mod_ruido(1,:)-MM(1,:)).^2+(X_mod_ruido(2,:)-MM(2,:)).^2));
 ekf_rmse_r = sqrt(mean((X_mod_ruido(3,:)-MM(3,:)).^2));
 
if mostrar_simulacion == true 
    %Representaciones gráficas de los resultados de la estimación 

      M = MM(:,1);  
      P = PP(:,:,1);
      EST = M;
      [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
     
      tt = (0:0.01:1)*2*pi;
      cc = repmat([0;0],1,length(tt)) + ...
              Intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)]; %Calculamos el intervalo de confianza
      cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
      
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
               Baliza_4(1),Baliza_4(2),'g--',...
               Baliza_4(1),Baliza_4(2),'k^',...
               Baliza_5(1),Baliza_5(2),'g--',...
               Baliza_5(1),Baliza_5(2),'k^');
     
      if Modelo_medida == 1
        title('Localización usando EKF1 (Distancia)')
      end
      if Modelo_medida == 2
          title('Localización usando EKF1 (Ángulo)')
      end
     
      x_min = min(objetivos(1,:))-2;
      x_max = max(objetivos(1,:))+2;
      y_min =  min(objetivos(2,:))-2;
      y_max =  max(objetivos(2,:))+2;
      axis([x_min x_max y_min y_max ]);
      hold on
      
      
      %Colocamos unas marcas de texto en la representación
      
      text(x_inicial-1,y_inicial-1,'SALIDA')
      texto = 'Baliza ';
      b = 1;
      a = texto;
      for tex_i=1:size(Balizas,2)
        text(Balizas(1,tex_i)-1,Balizas(2,tex_i)-1,a)
        b = num2str(b);
        a = [texto b];
        b = str2num(b);
        b = b+1;
        
      end
      
      Robot = plot(punto(1,:),punto(2,:),'r',...
                   ruedader1(1,:),ruedader1(2,:),'g',...
                   ruedaizq1(1,:),ruedaizq1(2,:),'g',...
                   Lateral_sup1(1,:),Lateral_sup1(2,:),'r',...
                   Lateral_inf1(1,:),Lateral_inf1(2,:),'r',...
                   trasera(1,:),trasera(2,:),'r',...
                   linea_abajo1(1,:),linea_abajo1(2,:),'r',...
                   linea_arriba1(1,:),linea_arriba1(2,:),'r');
       hold on
       
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
            [len] = Dist_medida(X_mod_ruido(:,k),Balizas,dist_max);
            
            M = MM(:,k);
            Media = MM(:,k);
            P = PP(:,:,k);
            EST = MM(:,1:k);

            [punto,ruedader1,ruedaizq1,Lateral_sup1,Lateral_inf1,trasera,linea_abajo1,linea_arriba1] = Pioneer3DX(M(1,1),M(2,1),M(3,1),Radio);
          
            % Confidence ellipse

            cc = repmat([0;0],1,length(tt)) + ...
             Intervalo_conf*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
              
            cc=Inter_Conf(cc,M(1,1),M(2,1),M(3,1));
            % Measurement directions
            dx_dy = Dir_medida(len,Y_rot(:,k));
           
            % Update graphics
            set(h(2),'xdata',Media(1)); set(h(2),'ydata',Media(2));
            set(h(3),'xdata',EST(1,:)); set(h(3),'ydata',EST(2,:)); 
            set(h(4),'xdata',cc(1,:)); set(h(4),'ydata',cc(2,:)); 
            set(h(5),'xdata',[Baliza_1(1);Baliza_1(1)+dx_dy(1,1)]); set(h(5),'ydata',[Baliza_1(2);Baliza_1(2)+dx_dy(2,1)]); 
            set(h(7),'xdata',[Baliza_2(1);Baliza_2(1)+dx_dy(1,2)]); set(h(7),'ydata',[Baliza_2(2);Baliza_2(2)+dx_dy(2,2)]); 
            set(h(9),'xdata',[Baliza_3(1);Baliza_3(1)+dx_dy(1,3)]); set(h(9),'ydata',[Baliza_3(2);Baliza_3(2)+dx_dy(2,3)]); 
            set(h(11),'xdata',[Baliza_4(1);Baliza_4(1)+dx_dy(1,4)]); set(h(11),'ydata',[Baliza_4(2);Baliza_4(2)+dx_dy(2,4)]);
            set(h(13),'xdata',[Baliza_5(1);Baliza_5(1)+dx_dy(1,5)]); set(h(13),'ydata',[Baliza_5(2);Baliza_5(2)+dx_dy(2,5)]);
            
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
            
            drawnow;
         
          end
end
      
disp(' ')
disp('Simulación finalizada')
disp(' ')

fprintf('  El RMS de las coordenadas x e y, es de %f \n',ekf_rmse)
disp(' ')
fprintf('  El RMS de la rotación es de %f \n',ekf_rmse_r)
%Creamos un bloque de depuración para comprobar que las dimensiones de las
%matrices son las correctas y que las variables tienen el valor que
%esperamos obtener

set(handles.rms_xy,'String',num2str(ekf_rmse))
set(handles.rms_rot,'String',num2str(ekf_rmse_r))

clear M P A Q Odom Y_adap y_completo MM PP 

% --- Executes on selection change in mostrar_simulacion_in.
function mostrar_simulacion_in_Callback(hObject, eventdata, handles)
% hObject    handle to mostrar_simulacion_in (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns mostrar_simulacion_in contents as cell array
%        contents{get(hObject,'Value')} returns selected item from mostrar_simulacion_in


% --- Executes during object creation, after setting all properties.
function mostrar_simulacion_in_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mostrar_simulacion_in (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in preguntar_simular_in.
function preguntar_simular_in_Callback(hObject, eventdata, handles)
% hObject    handle to preguntar_simular_in (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns preguntar_simular_in contents as cell array
%        contents{get(hObject,'Value')} returns selected item from preguntar_simular_in


% --- Executes during object creation, after setting all properties.
function preguntar_simular_in_CreateFcn(hObject, eventdata, handles)
% hObject    handle to preguntar_simular_in (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1



function vel_sim_Callback(hObject, eventdata, handles)
% hObject    handle to vel_sim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vel_sim as text
%        str2double(get(hObject,'String')) returns contents of vel_sim as a double


% --- Executes during object creation, after setting all properties.
function vel_sim_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vel_sim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Modelo_med.
function Modelo_med_Callback(hObject, eventdata, handles)
% hObject    handle to Modelo_med (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Modelo_med contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Modelo_med


% --- Executes during object creation, after setting all properties.
function Modelo_med_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Modelo_med (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ruido_med_Callback(hObject, eventdata, handles)
% hObject    handle to ruido_med (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ruido_med as text
%        str2double(get(hObject,'String')) returns contents of ruido_med as a double


% --- Executes during object creation, after setting all properties.
function ruido_med_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ruido_med (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ruido_odo_Callback(hObject, eventdata, handles)
% hObject    handle to ruido_odo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ruido_odo as text
%        str2double(get(hObject,'String')) returns contents of ruido_odo as a double


% --- Executes during object creation, after setting all properties.
function ruido_odo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ruido_odo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function dt_in_Callback(hObject, eventdata, handles)
% hObject    handle to dt_in (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dt_in as text
%        str2double(get(hObject,'String')) returns contents of dt_in as a double


% --- Executes during object creation, after setting all properties.
function dt_in_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dt_in (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function dist_max_Callback(hObject, eventdata, handles)
% hObject    handle to dist_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dist_max as text
%        str2double(get(hObject,'String')) returns contents of dist_max as a double


% --- Executes during object creation, after setting all properties.
function dist_max_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dist_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function uitable2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uitable2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
imshow('logo-vector-universidad-la-laguna.jpg')
% Hint: place code in OpeningFcn to populate axes3

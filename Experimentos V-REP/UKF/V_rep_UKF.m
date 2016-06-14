% Experimentación por medio de una API con V-REP para utilizar el UKF

% Copyright (C) 2016 Iván Rodríguez Méndez, Antonio Luis
% Morell González, Leopoldo Acosta Sánchez
%
% This software is distributed under the GNU General Public 
% Licence (version 3 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [Robot_1,UKF_RMS] = V_rep_UKF(Escena,sd_baliza,Modelo_medida)
    
    UKF_RMS = [];
    semilla = 1;
    
        rng(semilla)
        disp('Pruebas de comuniación con V-REP');
        disp('   Autor: Iván Rodríguez Méndez');
        disp(' ');
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        disp('¡Funciones cargadas!');
        disp(' ');
        
        %Creamos varias opciones para poder elegir que escena nos gustaría
        %cargar
        %% Seleccion del modelo de medida
        % Si el modelo de medida es igual a 1 usaremos la distancia
        % euclidea, en el caso de ser igual a 2 usaremos la información
        % del angulo.
        
        
        
        %% Seleccion de escena
       
        
         if Escena == 1 
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Cuadrado_LF_01.ttt';
        end
        if Escena == 2
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Cuadrado_LF_003.ttt';
        end
         if Escena == 3
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Cuadrado_LF_005.ttt';
         end
        if Escena == 4 
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Cuadrado_LF_007.ttt';
        end
         if Escena == 5 
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Recta_LF_01.ttt';
         end
         if Escena == 6
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Recta_LF_003.ttt';
         end
        if Escena == 7
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Recta_LF_005.ttt';
        end
        if Escena == 8
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Recta_LF_007.ttt';
        end
        if Escena == 9
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Seno_LF_01.ttt';
        end
        if Escena == 10
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Seno_LF_003.ttt';
        end
        if Escena == 11
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Seno_LF_005.ttt';
        end
        if Escena == 12
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Seno_LF_007.ttt';
        end
        if Escena == 13
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Cuadrado_LF_3.ttt';
        end
        if Escena == 14
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Cuadrado_LF_5.ttt';
        end
        if Escena == 15
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Recta_LF_3.ttt';
        end
        if Escena == 16
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Recta_LF_5.ttt';
        end
        if Escena == 17
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Seno_LF_3.ttt';
        end
        if Escena == 18
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Seno_LF_5.ttt';
        end
        if Escena == 19
            link = '/home/ivan/Dropbox/TFG/V-REP/Escenas-Exp/Circuito_completo.ttt';
        end

        %Aquí es donde empieza el código principal
        disp('Nos conectaremos a la siguiente');
        IP_UNI = '10.213.13.255'
        IP_PERSONAL = '192.168.1.38'
        IP_LOCAL = '127.0.0.1'
        IP_UNI2 = '10.213.13.213'

        clientID = Abrir_conexion(IP_LOCAL);

        %Comprobamos si estamos conectados a V-REP
        if clientID > -1 
            conectado = 1;
        else 
            conectado = 0;
        end

        if conectado ==  1
            fprintf('El cliente con el que se ha establecido conexión es el número %d \n',clientID);
            disp(' ');
            disp('Conectado al servidor remoto API y ejecutando el código principal...');
            	

            % Enviamos alguna información al terminal de V-rep:
            Mensaje_terminal(clientID,'Simulacion de localizacion de un robot diferencial');
            %% Preparación de la escena y definicion de parámetros
            Cargar_escena(clientID,link,1); %Cargamos la escena en la que se ejecutará la simulación
            Cargar_modelo(clientID,'/home/ivan/Dropbox/TFG/V-REP/ModeloVREP/pioneer_pen.ttm',1); %Cargamos el modelo del robot que usaremos, este es el robot real.
            
            [Estado,Handle] = Obtener_handle(clientID,'Pioneer_p3dx');
            Robot_1 = Definir_robot(clientID,Handle)
            
            Cargar_modelo(clientID,'/home/ivan/Dropbox/TFG/V-REP/ModeloVREP/pioneer_estimado.ttm',1); %Cargamos el modelo del robot que usaremos, este es el estimado.
            [Estado,Robot_2] = Obtener_handle(clientID,'Pioneer_p3dx#0');
            Definir_posicion(clientID,Robot_2,[0 0 0.1388]); %Definimos la posición inicial para el robot estimado.
            
            disp('Calculando la posición de las balizas...');
            [Pos_bal,Numero_bal] = Balizas(clientID); %Calculamos con ayuda de la función la colocación de las balizas.
            Numero_bal %Mostramos el número de balizas que encontramos en la escena 
            
            disp('posición de las balizas calculada');
            

            %% Caracterización del robot
            %Generamos las velocidades lineales y angulares de nuestro robot
            V_LINEAL = 0.5; %Velocidad lineal máxima
            V_ANGULAR = 7.2; %Velocidad de rotación máxima
            Limite_trayectoria = 3000 ; %Definimos el límite de la trayectoria, es decir la cantidad máxima de puntos que cogemos.
            dist_max = 5 ; %distancia maxima que es capaz de medir el telemetro.

            %Definimos la pose inicial que tendra nuestro robot
            x_inicial = Robot_1.x;
            y_inicial = Robot_1.y;
            alfa_inicial = Robot_1.alfa;
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
            objetivos = Objetivos_loc(clientID,pose);
            disp('posición de los objetivos calculada');

            %Definimos un indice para recorrer el vector de objetivos (para
            %conocer cual es el objetivo que queremos buscar)
            obj_actual = 1;
            

            %Inicializamos los vectores de medida que usaremos en el
            %recorrido para la simulación.
            %% Inicialización de parámetros de vectores
            y = [];
            y_bal = [];
            Y = [];
            Y_real = [];
            x = [];
            X = [];
            odom = [];
            Odom = [];
            %% Inicialización de parámetros del filtro
            
            %% Importamos modelos de medida
            
            if Modelo_medida == 1
                %Importamos los Script que nos modelan la ecuación de medida
                %como la distancia euclídea.
                h_func = @bot_dist_h; %Modelo de medida
                dh_dx_func = @bot_dist_dh; %Derivada del modelo
            end
            if Modelo_medida == 2 
                %Importamos los Script que nos modelan la ecuación de medida
                %como el angulo entre la baliza y la pose del robot.
                h_func = @bot_h; %Modelo de medida
                dh_dx_func = @bot_dh_dx; %Derivada del modelo
            end  
            %% Ruidos del robot
            
            sd_avance = 0.05;
            sd_giro = 0.00001;
            Establecer_ruido(Robot_1,sd_avance,sd_giro,sd_baliza); %Guardamos el ruido en la estructura que creamos para el robot.
            dt = 0.05; %Definimos el diferencial de tiempo para calcular las matrices A y Q.

            M = [0;0;0]; %Media inicial para el filtro
            P = diag([0.2 0.2 0.2]); %Matriz de covarianzas inicial
            R = sd_baliza^2; %Ruido R definido como parámetro del filtro 

            qx = 0.1;
            qy = 0.1;

            A = [1 dt 0;
                 0 1  0;
                 0 0  1]; %Definimos las matrices A y Q tal y como se demuestra en la teoría de la toolbox

            Q = [(1/3)*(dt^3)*qx (1/2)*(dt^2)*qx 0;
                (1/2)*(dt^2)*qx   dt*qx          0;
                0                 0              dt*qy];

            MM = []; %Inicializamos los vectores que guardarán toda la información sacada de los filtros
            PP = [];
            %% Simulación
            Iniciar_simulacion(clientID); %Iniciamos una simulación 

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
                    [Estado,pose] = Obtener_pose(clientID,Robot_1.Handle_Robot);
                    Robot_1.x = pose(1,1);
                    Robot_1.y = pose(2,1);
                    Robot_1.alfa = pose(3,1);
                   
                    x = pose;
                    X = [X x];
                    
                    % Por otra parte medimos el diferencial de rotación que
                    % existe en la ruedas y de esta manera calculamos la
                    % odometría.
                    [Mov_centro ,Robot_1] = Actualizar_odom(clientID,Robot_1);
                    odom = [Robot_1.Odometria(1,1);Robot_1.Odometria(2,1);Robot_1.Odometria(3,1)];
                    Odom = [Odom odom];
     
                    
                   %% Aplicamos el filtro de Kalman extendido
                   % Durante el movimiento del robot aplicaremos el filtro
                   % de Kalman para realizar la extimación. Como
                   % anteriormente hemos tomado la información de la
                   % odometría, se la introduciremos a Kalman.
                    
                    [M,P] = ukf_predict1(M,P,A,Q,[],1.5,2.05,2); 
                    M = (10*odom + 90*M)/100;
                    %Realizamos las medidas de las medidas que captamos
                    %desde nuestro robot.
                    if Modelo_medida == 1
                        [y_bal,y_real,y_adap,Pos_bal_adap,Lecturas] = Tomar_medidas_dist(Numero_bal,Pos_bal,pose,Robot_1.Ruido_medida,dist_max);
                    end
                    if Modelo_medida == 2
                        [y_bal,y_real,y_adap,Pos_bal_adap,Lecturas] = Tomar_medidas_angle(Numero_bal,Pos_bal,pose,Robot_1.Ruido_medida,dist_max);   
                    end
                    
                    %Guardamos los datos
                     Y = [Y y_bal];
                     Y_real = [Y_real y_real];
                    [M,P] = ukf_update1(M,P,y_adap,h_func,R*eye(Lecturas-1),Pos_bal_adap);
                    
                    MM(:,k) = M; 
                     PP(:,:,k) = P; %Guardamos las variables estimadas en los vectores para poder tener un histórico.
                       
                    %Colocamos el robot según lo que hemos estimado.
                    Definir_posicion(clientID,Robot_2,[M(1,1) M(2,1) 0.1388]);
                    Definir_orientacion(clientID,Robot_2,[0 0 M(3,1)]);
                    %% Seguimiento de objetivos
                    %Una vez aplicamos Kalman para la posición en la que
                    %nos encontramos aplicamos el algortimo que permite al
                    %robot seguir los objetivos.
                   
                   Objetivo = objetivos(:,obj_actual); %Guardamos en una variable el objetivo que debemos alcanzar.
                   angulo = Angulo_relativo(pose,Objetivo); % Calculamos el angulo entre la posicion actual y el objetivo que pretendemos alcanzar.
                   v = norm(pose(1:2)-objetivos(1:2,obj_actual)); %Calculamos la distancia entre la posicion actual y el objetivo que queremos alcanzar.

                   w = angulo; %Guardamos el angulo que hemos calculado para pasar a trabajar con el.
                   
                   [w,v] = Seguir_objetivos(w,v,W,V); %Aplicamos nuestro algortimo para que el robot siga los objetivos describiendo una trayectoria curva.

                    %Movemos el robot con los parametros que hemos
                    %calculado anteriormente para cada una de las ruedas.
                    Mover_robot(clientID,Robot_1,v,w);
                    %Obtenemos la pose despues de haber movido el robot
                    [Estado,pose] = Obtener_pose(clientID,Robot_1.Handle_Robot);
                    trayectoria = [trayectoria pose]; %Guardamos la pose del robot en el vector que define la trayectoria.

                    %Imprimimos dos variables de interes para saber en que
                    %paso del bucle nos encontramos y que objetivo
                    %pretendemos alcanzar.
                    obj_actual
                    Obj_real = obj_actual;
                    size(trayectoria,2)
                    k= k +1; %Indices que usamos para Kalman
                    vrep.simxSynchronousTrigger(clientID); %Mandamos un trigger de sincronizacion.
                end
                obj_actual = obj_actual+1; %Cuando salimos del bucle pasamos a buscar el siguiente objetivo.
            end
            clf;
            % Comprobamos que durante la simulacion hemos alcanzado todos
            % los objetivos.
            Parar_simulacion(clientID); %Paramos la simulación en V-REP
            
            if (Obj_real == size(objetivos,2))
                disp('Se han alcanzado todos los objetivos');
            else 
                disp('No se han alcanzado todos los objetivos');
            end

            %Guardamos todos los datos, el primero de ellos el error
            %cuadratico medio.
            ukf_rms = sqrt(mean((X(1,:)-MM(1,:)).^2 + (X(2,:) - MM(2,:)).^2))

            UKF_RMS = [[ukf_rms;Obj_real]];
            
            % Guardamos todas las medidas en la estructura de datos para
            % disponer de todos ellos una vez la funcion termine su
            % ejecucion.
            Robot_1.Medidas = Y;
            Robot_1.Trayectoria_real = X;
            Robot_1.Trayectoria_est = MM;
            Robot_1.Odometria_total = Odom;
            Robot_1.Medidas_reales = Y_real;

            Representar_trayectoria(objetivos,Pos_bal,X,MM);
            
            Cerrar_Escena(clientID);
            % Nos aseguramos de que el último comando a llegado
            % correctamente.
            % Cerramos las conexiones con V-REP:	
            Cerrar_conexion(clientID);
            vrep.delete(); % llamamos al destructor para dejar de usar el metodo que hemos definido al principio de la funcion.

            disp('¡Programa finalizado!');
        else
            disp('La conexión no se ha podido establecer, así que el programa se cerrará');

        end

    
end

%% Definicion de las funciones
%Comenzamos a definir las funciones que utilizaremos.

    function clientID = Abrir_conexion(IP)
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.

        clientID=vrep.simxStart(IP,19999,true,true,5000,5);
        %Activamos el modo síncrono
        
        %Comprobamos si ha sido posible conectarnos o no con V-REP.
        if clientID == -1
            disp('¡Imposible conectar!');

        end
        if clientID > -1 
            disp('¡Conexión establecida!');
            vrep.simxSynchronous(clientID,true);

        end
    end

    function Cerrar_conexion(clientID)
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.

        vrep.simxFinish(clientID);
        fprintf('Conexión del cliente %d cerrada \n',clientID);

    end

    function Iniciar_simulacion(clientID)
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        disp('--------------------------------------------');
        disp('Iniciamos la simulación...');
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    end
    
    function Reanudar_simulacion(clientID) %Usaremos esta funcion en el caso de que previamente pausemos la simulacion.
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    end
    
    function Parar_simulacion(clientID)
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        disp('--------------------------------------------');
        disp('Paramos la simulación.');
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
    end

    function [Estado, Handle] = Crear_dummy(clientID,size) %Creamos una esfera que nos permite hacer un test de las fisicas
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        vrep.simxCreateDummy(clientID,size,[],vrep.simx_opmode_oneshot_wait);
    end

    function Mensaje_terminal(clientID,texto) %Funcion que nos permite mandar un mensaje via terminal.
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        vrep.simxAddStatusbarMessage(clientID,texto,vrep.simx_opmode_oneshot);
    end

    function [Ping] = Mostrar_ping(clientID) %Funcion que nos permite conocer el ping de nuestra conexion con V-REP
       vrep=remApi('remoteApi'); %Importamos el archivo de la API.
       Ping =  vrep.simxGetPingTime(clientID);
    end

    function [Estado,Identificador] = Cargar_modelo(clientID,modelPathAndName,options)
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        disp('--------------------------------------------');
        disp('Vamos a agregar un modelo a nuestra escena...');
        [Estado,Identificador] = vrep.simxLoadModel(clientID,modelPathAndName,options,vrep.simx_opmode_oneshot_wait);
        
        %Comprobamos si el modelo ha sido cargado correctamente o hemos
        %generado algun tipo de error.
        if Estado ~= 32 
            disp('¡Modelo cargado con éxito!');
        else 
            disp('Error al intentar cargar el modelo (Func:Cargar_modelo)');
        end
    end
 
    function Estado = Cargar_escena(clientID,path,opciones)
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        disp('--------------------------------------------');
        disp('Vamos a cargar la escena elegida...');
        Estado = vrep.simxLoadScene(clientID,path,opciones, vrep.simx_opmode_oneshot_wait);
        
        %Comprobamos si la escena se ha podido cargar correctamente.
        if Estado ~= 32 
            disp('¡Escena cargada con éxito!');
        else 
            disp('Error al intentar cargar la escena (Func:Cargar_escena)');
        end
    
    end

    function [Estado,Pose] = Obtener_pose(clientID,identificador_obj) %Esta funcion nos devuelve las posiciones X e Y ademas de la orientacion.
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        
        %Obtenemos la posicion del robot en la escena.
        [Estado_1,pose] = vrep.simxGetObjectPosition(clientID,identificador_obj,-1,vrep.simx_opmode_oneshot_wait);
        %Obtenemos la orientacion de nuestro robot.
        [Estado_2,angulos] = vrep.simxGetObjectOrientation(clientID,identificador_obj,-1,vrep.simx_opmode_oneshot_wait);

        Pose = [pose(1);pose(2);angulos(3)];
        
        %Comprobamos que todo el proceso se ha dado de forma correcta, en
        %caso contrario mandamos un mensaje de error.
        if Estado_1==32 || Estado_2 == 32
            Estado = 32;
            disp('Error al calcular la pose (Func:Obtener_pose)');
        else 
            Estado = Estado_1;
        end
    end
    
    function [Estado,Posicion] = Obtener_posicion(clientID,identificador_obj) %Funcion para obtener las coordenadas cartesianas de un objeto en la escena
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        [Estado,Posicion] = vrep.simxGetObjectPosition(clientID,identificador_obj,-1,vrep.simx_opmode_oneshot_wait);
        Posicion = Posicion';
    end
    
     function [Estado,Rotacion] = Obtener_rotacion(clientID,identificador_obj) %Funcion para obtener las rotaciones de un objeto en la escena.
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        [Estado,Rotacion] = vrep.simxGetObjectOrientation(clientID,identificador_obj,-1,vrep.simx_opmode_oneshot_wait);
        Rotacion = Rotacion';
    end

    function [vl,vr] = Mover_robot(clientID,robot,v,w) %Funcion que nos permite que robot siga unos objetivos prestablecidos
        
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        if abs(v) < 0.0001 && abs(w) < 0.0001
            vl = 0; % Detenemos la rueda izquierda
            vr = 0; % Detenemos la rueda derecha
        else
            v_hat = v ; % Aquí podemos añadirle un término de ruido. Hemos optado por considerar ese ruido como parte de la escena (friccion del suelo)
            w_hat = w ; % Aquí también podemos añadirle un término de ruido.
            vl = (v_hat - robot.Radio_Robot * w_hat) / robot.Radio_Rueda_Izquierda; %calculamos los comandos que tendriamos que mandar a cada una de las ruedas.
            vr = (v_hat + robot.Radio_Robot * w_hat) / robot.Radio_Rueda_Derecha;   
        end 
        
        Enviar_signal(clientID,[vr vl]); %Enviamos la señal de movimiento al robot en V-REP.
        
    end
    
     function Parar_robot(clientID) %Funcion que hace que el robot pare sus motores.
        Enviar_signal(clientID,[0 0]);
     end
        

    function Parametros = Definir_robot(clientID,identificador_obj) %Funcion que inicializa todos los parametros de un robot insertado en la escena.
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        
        %Definimos la pose del robot. Primero consultamos cual es su estado
        %y luego la guardamos.
        
        [Estado,Pose] = Obtener_pose(clientID,identificador_obj);
        if Estado ~= 32
            field1 = 'x';  value1 = Pose(1,1);
            field2 = 'y';  value2 = Pose(2,1);
            field3 = 'alfa';  value3 = Pose(3,1);
        else 
            field1 = 'x';  value1 = -9999;
            field2 = 'y';  value2 = -9999;
            field3 = 'alfa';  value3 = -9999;
            disp('Error calculando la pose del robot (Func:Definir_robot)');
        end
        
        %Calculamos el radio de nuestro robot ya que lo necesitamos para
        %saber que comandos aplicar para tomar curvas.
        
        [Estado_1,Numero_1] = Obtener_handle(clientID,'Pioneer_p3dx_leftWheel');
        [Estado_2,Numero_2] = Obtener_handle(clientID,'Pioneer_p3dx_rightWheel');
        if Estado_1 ~= 0 || Estado_2 ~= 0 
            Radio_robot = -9999;
            disp('Error al obtener el handle de las ruedas del robot (Func:Definir_robot)');
        else
            [Estado_1,Posicion_izq] = Obtener_posicion(clientID,Numero_1);
            [Estado_2,Posicion_der] = Obtener_posicion(clientID,Numero_2);
            if Estado_1 ~= 0 || Estado_2 ~= 0 
                Radio_robot = -7777; 
                disp('Error al obtener el radio de las ruedas del robot (Func:Definir_robot)');
            else
                Radio_robot = sqrt((Posicion_izq(1,1)-Posicion_der(1,1))^2 + (Posicion_izq(2,1)-Posicion_der(2,1))^2 );
                Radio_rueda_der = Posicion_der(3);
                Radio_rueda_izq = Posicion_izq(3);
            end
        end
        
        field4 = 'Radio_Robot';  value4 = {Radio_robot};
        field5 = 'Radio_Rueda_Derecha'; value5 = {Radio_rueda_der};
        field6 = 'Radio_Rueda_Izquierda'; value6 = {Radio_rueda_izq};
        field7 = 'Client_ID'; value7 = {clientID};
        field8 = 'Handle_Robot'; value8 = {identificador_obj};
        field9 = 'Pose_Anterior'; value9 = {[0;0;0]};
        field10 = 'Odometria'; value10 = {[0;0;0]};
        
        [Estado,Numero] = Obtener_handle(clientID,'Pioneer_p3dx_leftMotor');
        if Estado ~= 0 
            field11 = 'Rueda_izq_handle'; value11 = {9999};
            disp('Error al obtener el handle del motor izquierdo (Func:Definir_robot)');    
        else
            field11 = 'Rueda_izq_handle'; value11 = {Numero};
        end
        field12 = 'Ult_ori_izq'; value12 = {0};
        
        [Estado,Numero] = Obtener_handle(clientID,'Pioneer_p3dx_rightMotor');
        if Estado ~= 0 
            field13 = 'Rueda_der_handle'; value13 = {9999};
            disp('Error al obtener el handle del motor derecho (Func:Definir_robot)');   
        else
            field13 = 'Rueda_der_handle'; value13 = {Numero};
        end
        field14 = 'Ult_ori_der'; value14 = {0};
        
        Y = [];
        field15 = 'Medidas'; value15 = {Y};
        T_real = [];
        field16 = 'Trayectoria_real'; value16 = {T_real};
        T_estimada = [];
        field17 = 'Trayectoria_est'; value17 = {T_estimada};
        Odom_total = [];
        field18 = 'Odometria_total'; value18 = {Odom_total};
        Y_real = [];
        field19 = 'Medidas_reales'; value19 = {Y_real};
        %Definimos unos valores bajos para los parametros de ruido ya que
        %si es cero el filtro de Kalman no funcionaria.
        field20 = 'Ruido_avance'; value20 = {0.0001};
        field21 = 'Ruido_giro'; value21 = {0.0001};
        field22 = 'Ruido_medida'; value22 = {0.0001};
        
        %Guardamos todos los parametros de la estrucutra de datos.
        Parametros = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6,field7,value7,field8,value8,field9,value9,field10,value10,field11,value11,field12,value12,field13,value13,field14,value14,field15,value15,field16,value1,field17,value17,field18,value18,field19,value19,field20,value20,field21,value21,field22,value22);
        
    end
      
    function [Estado,Numero] = Obtener_handle(clientID,nombre) %Funcion que usamos para obtener el handle de un objeto que necesitemos 
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        [Estado,Numero] = vrep.simxGetObjectHandle(clientID,nombre,vrep.simx_opmode_oneshot_wait);
    end
    
    function [Estado] = Enviar_signal(clientID,Valor_signal) %Funcion que usamos para enviar los datos de los motores a V-REP.
        vrep=remApi('remoteApi');
        v = [Valor_signal 0];
        packedData = vrep.simxPackFloats(v);
        [Estado] = vrep.simxSetStringSignal(clientID,'PioneerDiffData',packedData,vrep.simx_opmode_oneshot);
    end
    
    function [Pos_bal,Numero_bal] = Balizas(clientID)
        %Lo que haremos en esta función será recorrer los hanble en orden
        %para ir sacando la posición de cada una de las balizas y
        %recogeremos esos datos en un vector. Posteriormente es  lo que
        %usaremos como parametro de entrada al filtro de Kalman.
        
        Pos_bal = [];
        Identificador = '80cmHighPillar25cm';
        Identificador_Obj = Identificador;
        [Estado, Numero] = Obtener_handle(clientID,Identificador);
        a = -1;
        
        while Estado == 0
            [Estado, Numero] = Obtener_handle(clientID,Identificador_Obj);
            [Estado_2, Posicion_bal] = Obtener_posicion(clientID,Numero);
            if Estado == 0 && Estado_2 == 0
                Pos_bal = [Pos_bal Posicion_bal];
            end

            a = a + 1;
            a = num2str(a);
            Identificador_Obj = [Identificador a]; 
            a = str2num(a);

        end

        Numero_bal = size(Pos_bal,2);
        
    end
    
    function [Pos_objetivos,Num_objetivos] = Objetivos_loc(clientID,pose_inicial)
       %Lo que haremos en esta función será recorrer los hanble en orden
        %para ir sacando la posición de cada uno de los objetivos y
        %recogeremos esos datos en un vector.
        Pos_objetivos = [];
        Identificador = 'Cylinder';
        Identificador_Obj = Identificador;
        [Estado, Numero] = Obtener_handle(clientID,Identificador);
        a = -1;
        
        while Estado == 0
            [Estado, Numero] = Obtener_handle(clientID,Identificador_Obj);
            [Estado_2, Posicion_objetivo] = Obtener_posicion(clientID,Numero);
            if Estado == 0 && Estado_2 == 0 
                Pos = Posicion_objetivo ;
                Pos_objetivos = [Pos_objetivos Pos];
            end
           
            a = a + 1;
            a = num2str(a);
            Identificador_Obj = [Identificador a]; 
            a = str2num(a);
        end
        
        Angulos = [];
        angulos = [];
        a = 2;
        %Ahora necesitamos calcular los ángulos con los que afrontamos los
        %objetivos, esto solo se usaria si queremos realizar una
        %interpolacion lineal. En el caso de realizar otro tipo de
        %movimiento no necesitamos saber los angulos relativos entre cada
        %uno de los objetivos.
        for i=1:size(Pos_objetivos,2)
            if i == 1
                angulos = atan2(Pos_objetivos(2,1)-pose_inicial(2,1),Pos_objetivos(1,1)-pose_inicial(1,1));
            else
                angulos = atan2(Pos_objetivos(2,a)-Pos_objetivos(2,a-1),Pos_objetivos(1,a)-Pos_objetivos(1,a-1));
                a = a+1;
            end
            
            Angulos = [Angulos angulos];
            
        end
        
        Pos_objetivos = [Pos_objetivos(1,:);Pos_objetivos(2,:);Angulos(1,:)]
        Num_objetivos = size(Pos_objetivos,2);
    end
   
    
    function [Estado,Handle_consola] = Abrir_consolaAux(clientID,nombre,lines_max,posicion,size,color_texto,color_fondo) %Abrir una consola para poder mostrar informacion
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        [Estado,Handle_consola] = vrep.simxAuxiliaryConsoleOpen(clientID,nombre,lines_max,0,posicion,size,color_texto,color_fondo,vrep.simx_opmode_oneshot_wait);
    end
    
    function [Estado] = Cerrar_consolaAux(clientID,Handle) 
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        [Estado] = vrep.simxAuxiliaryConsoleClose(clientID,Handle,vrep.simx_opmode_oneshot);
    end
    
    function [Estado] = Definir_posicion(clientID,Handle,posicion) %Funcion para definir la posicion de un objeto en la escena.
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        [Estado] = vrep.simxSetObjectPosition(clientID,Handle,-1,posicion,vrep.simx_opmode_oneshot);
    end
    
    function [Estado] = Definir_orientacion(clientID,Handle,orientaciones) %Funcion para definir la orientacion de un objeto en la escena.
        vrep=remApi('remoteApi'); %Importamos el archivo de la API.
        [Estado] = vrep.simxSetObjectOrientation(clientID,Handle,-1,orientaciones,vrep.simx_opmode_oneshot);
    end
    
    function [Estado , Handle_dialogo, Handle_ui] = Mostrar_dialogo(clientID,titulo,texto_principal,modo,texto_inicial)
        vrep=remApi('remoteApi');
        [Estado, Handle_dialogo, Handle_ui] = vrep.simxDisplayDialog(clientID,titulo,texto_principal,modo,texto_inicial,[],[],vrep.simx_opmode_oneshot_wait);
    end

    function [Estado, Numero_resultado] = Obtener_resultados_dialogo(clientID,Handle_dialogo)
        vrep=remApi('remoteApi');
        [Estado,Numero_resultado] = vrep.simxGetDiaglogResult(clientID,Handle_dialogo,vrep.simx_opmode_oneshot);
    end
    
    function [Estado, Texto_entrada] = Obtener_dialogo_entrada(clientID,Handle_dialogo)
        vrep=remApi('remoteApi');
        [Estado, Texto_entrada] = vrep.simxGetDialogInput(clientID,Handle_dialogo,vrep.simx_opmode_oneshot_wait);
    end
    
    function [Estado] = Cerrar_dialogo(clientID,Handle_dialogo)
        vrep=remApi('remoteApi');
        [Estado] = vrep.simxEndDialog(clientID,Handle_dialogo,vrep.simx_opmode_oneshot);
    end
    
    function [inc_izq,inc_der,robot] = Inc_encoder(clientID,robot) %Funcion que nos calcula el incremento de los encoders al mover el robot.
        vrep=remApi('remoteApi');
        [Estado, Rot_izq] = vrep.simxGetJointPosition(clientID,robot.Rueda_izq_handle,vrep.simx_opmode_oneshot_wait);
        angulo = dif_angulo(Rot_izq,robot.Ult_ori_izq);
        inc_izq = angulo * robot.Radio_Rueda_Izquierda;
        robot.Ult_ori_izq = Rot_izq;
        
        [Estado, Rot_der] = vrep.simxGetJointPosition(clientID,robot.Rueda_der_handle,vrep.simx_opmode_oneshot_wait);
        angulo = dif_angulo(Rot_der,robot.Ult_ori_der);
        inc_der = angulo * robot.Radio_Rueda_Derecha;
        robot.Ult_ori_der = Rot_der;
        
        
    end
    
    function [angulo] = dif_angulo(a,b)
        %Funcion para calcular la diferencia angular entre dos posiciones
        a = atan2(sin(a),cos(a));
        b = atan2(sin(b),cos(b));

        d1 = a-b;
        d2 = 2*pi - abs(d1);

        if (d1 > 0)
            d2 = d2*(-1);
        end
        if (abs(d1) < abs(d2))
            angulo = d1;
        else 
            angulo = d2;
        end
    end
    
    function [Mov_centro , robot] = Actualizar_odom(clientID,robot) %Funcion para calcular al odometria de nuestro robot, es decir, sacar la posicion en funcion de lo que nos hemos movido.
        robot.Pose_Anterior = robot.Odometria ;
        [inc_izq,inc_der,robot] = Inc_encoder(clientID,robot);
        Mov_centro = (inc_izq + inc_der)/2;
        inc_yaw = ((inc_der - inc_izq) / robot.Radio_Robot);
        a_norm = norm_angulo(inc_yaw + robot.Pose_Anterior(3,1));
        robot.Odometria(3,1) = a_norm;
        factor = 0;
        if (abs(inc_yaw) < 0.00001)
            robot.Odometria(1,1) = robot.Pose_Anterior(1,1) + Mov_centro*cos(robot.Pose_Anterior(3,1));
            robot.Odometria(2,1) = robot.Pose_Anterior(2,1) + Mov_centro*sin(robot.Pose_Anterior(3,1));
        else
            factor = Mov_centro / norm_angulo(inc_yaw);
            robot.Odometria(1,1) = robot.Pose_Anterior(1,1) + (sin(inc_yaw)*cos(robot.Pose_Anterior(3,1))- sin(robot.Pose_Anterior(3,1))*(1 -cos(inc_yaw)))*factor ;
            robot.Odometria(2,1) = robot.Pose_Anterior(2,1) + (sin(inc_yaw)*sin(robot.Pose_Anterior(3,1))+ cos(robot.Pose_Anterior(3,1))*(1 -cos(inc_yaw)))*factor ;
            
        end
    end
    
    function [a_norm] = norm_angulo(angulo) %Funcion para normalizar el angulo.
        a_norm = mod(mod(angulo,2*pi) + 2*pi, 2*pi);
        if (a_norm > pi)
            a_norm = a_norm - 2*pi;
        end
    end
    
    function [ u_t ] = odometry_motion(pose_ant,pose_act)
        %Modelo de odometría.
        %Este modelo implementa el modelo de odometría de tal
        %manera que convertimos los cambios entre dos poses del robot en una
        %rotación, una traslación y por último otra rotación. La salida de la
        %función será el vector de control que introduciremos al filtro de
        %Kalman. Este metodo se define como vector de control en el filtro
        %de Kalman clasico. Todas las referencias a esta solucion pueden
        %encontrarse en el libro "Probabilistic Robotics" de Sebastian Trhun. 

        Rotacion_1 = atan2(pose_act(2,1) - pose_ant(2,1),pose_act(1,1) - pose_ant(1,1)) - pose_act(3,1);
        Traslacion = sqrt((pose_ant(1,1) - pose_act(1,1))^2 + (pose_ant(2,1) - pose_act(2,1))^2 );
        Rotacion_2 = pose_act(3,1) - pose_ant(3,1) - Rotacion_1;

        u_t = [Traslacion*cos(pose_ant(3,1));Traslacion*sin(pose_ant(3,1));Rotacion_1 + Rotacion_2];

    end
    
    function [w,v] = Seguir_objetivos(w,v,W,V) %Funcion de seguimiento de objetivos, el robot tratara de describir una curva suavizada.
     %El metodo consiste en realizar una interpolacion con el suavizado
     %entre tramos de la trayectoria.
        dist = v;
        if w > W
            w = W;
        end
        if w < -W
            w = -W;
        end 

        if (v > V)
            v = V;
        end

        if (v < 0)
            v = 0;
        end

        if  abs(w) > 0.05 && abs(w) < 0.15 && dist > V
            v = V/4;
        else
            if abs(w) > 0.01 && abs(w) < 0.05 && dist > V
                v = V/2;
            else
                if abs(w) > 0.15
                    v = 0;
                end
            end
        end
    end
    
    
    
    function  [y_bal,y_real,y_adap,Pos_bal_adap,Lecturas] = Tomar_medidas_dist(Numero_bal,Pos_bal,pose,sd_baliza,dist_max)
        %Funcion que realiza las medidas alrededor del robot, conforme a
        %las balizas que tiene dentro del rango circular especificado en la
        %distancia maxima.
        
        for l=1:Numero_bal
            y = sqrt((pose(1,1)-Pos_bal(1,l))^2 + (pose(2,1)-Pos_bal(2,l))^2 ) + sd_baliza*randn;
            y_bal(l,1) = y;
            if (y <= dist_max ) %Establecemos el límite de metros que el telémtro es capaz de medir
                y_real(l,1) = y_bal(l,1);
            else
                y_real(l,1) = 0;
            end
        end
        
        Lecturas = 1;
        y_adap = [];
        Pos_bal_adap = [];
        
        for h=1:size(y_real,1)
            if (y_real(h,1) ~= 0)
                y_adap(Lecturas,1) = y_real(h,1);
                Pos_bal_adap(:,Lecturas) = Pos_bal(:,h);
                Lecturas = Lecturas +1;
            end
        end
            
    end
    
    function  [y_bal,y_real,y_adap,Pos_bal_adap,Lecturas] = Tomar_medidas_angle(Numero_bal,Pos_bal,pose,sd_baliza,dist_max)
        %Funcion que realiza las medidas alrededor del robot, conforme a
        %las balizas que tiene dentro del rango circular especificado en la
        %distancia maxima, aunque para este caso lo que medimos es el ángulo.
        
        for l=1:Numero_bal
            y_dist = sqrt((pose(1,1)-Pos_bal(1,l))^2 + (pose(2,1)-Pos_bal(2,l))^2 );
            y = atan2(pose(2,1)-Pos_bal(2,l),pose(1,1)-Pos_bal(1,l)) + sd_baliza*randn;
            y_bal(l,1) = y;
            if (y_dist <= dist_max ) %Establecemos el límite de metros que el telémtro es capaz de medir
                y_real(l,1) = y_bal(l,1);
            else
                y_real(l,1) = 0;
            end
        end
        
        Lecturas = 1;
        y_adap = [];
        Pos_bal_adap = [];
        
        for h=1:size(y_real,1)
            if (y_real(h,1) ~= 0)
                y_adap(Lecturas,1) = y_real(h,1);
                Pos_bal_adap(:,Lecturas) = Pos_bal(:,h);
                Lecturas = Lecturas +1;
            end
        end
            
    end
    
    function Establecer_ruido(robot,ruido_avance,ruido_giro,ruido_medida) %Funcion para caracterizar los ruidos de nuestro robot. 
        robot.Ruido_avance = ruido_avance;
        robot.Ruido_giro = ruido_giro;
        robot.Ruido_medida = ruido_medida;
    end
    
    function [w]=Angulo_relativo(pose,objetivo)
        %Diferencia entre dos orientaciones (la del robot y el punto que
        %quiere alcanzar) Finalmente usamos esta funcion para calcular como
        %debe darse la trayectoria.
        w = atan2(objetivo(2,1)-pose(2,1),objetivo(1,1)-pose(1,1))-pose(3,1);
        while w > pi
            w = w-2*pi;
        end
        while w < -pi
            w = w+2*pi;
        end
    end

    function [Estado]=Pausar_simulacion(clientID) %Funcion para pausar una simulacion. Esto significa que la simulacion mantendra todas las propiedades al reanudarla.
        vrep=remApi('remoteApi');
        [Estado]=vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    end
    
    
    function [Estado,Handle] = Obtener_handle_coleccion(clientID,nombre) %Funcion que usamos para conocer el handle de una coleccion de objetos
        vrep=remApi('remoteApi');
        [Estado,Handle] = vrep.simxGetCollectionHandle(clientID,nombre,vrep.simx_opmode_oneshot_wait);
    end
    
    function [Estado]= Cerrar_Escena(clientID)
        vrep=remApi('remoteApi');
        [Estado]=vrep.simxCloseScene(clientID,vrep.simx_opmode_oneshot_wait);
    end
    
      function Representar_trayectoria(Objetivos,Balizas,Tray_real,Tray_estimada)
        for i=1:size(Objetivos,2)
            plot(Objetivos(1,i),Objetivos(2,i),'k^');
            hold on
        end
        for j=1:size(Balizas,2)
            plot(Balizas(1,j),Balizas(2,j),'m^');
            hold on
        end
        
        plot(Tray_real(1,:),Tray_real(2,:),'b');
        hold on
        plot(Tray_estimada(1,:),Tray_estimada(2,:),'r');
        
        
        axis square
        title('Recorrido realizado y estimación');
        
    end
   
    



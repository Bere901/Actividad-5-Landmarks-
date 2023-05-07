# Actividad 5

  Daniela Berenice Hernández de Vicente - A01735346 

  ## Landmarks

  En esta actividad se busca que el estudiante pueda implementar el código requerido para generar el seguimiento de los siguientes waypoints (puntos de referencia), ajustando el tiempo de muestreo: “sampleTime”, vector de tiempo: “tVec”, pose inicial: “initPose”, y los waypoints: “waypoints”.
  
  Aunado a este debe poder generar los waypoints (puntos de referencia) necesarios para obtener las siguientes trayectorias, ajustando el tiempo de muestreo: “sampleTime”, vector de tiempo: “tVec”, pose inicial: “initPose”, y los waypoints: “waypoints”
  
  ![image](https://user-images.githubusercontent.com/99983026/236655513-60a9591b-c900-47f1-9359-473ebd6449d9.png)

  ### Explicación del Código
  
Inicialmente se define el vehiculo ingresando los datos del radio de las llantas y la base del mismo.

  ``` matlab
%% Definicion del vehiculo
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);
  ``` 
  
Posteriormente se establecen los parametros para nuestra simulacion, donde se ingresa el tiempo de muestreo, el tiempo total su posicion inicial en x,y,theta, aunado a esto se crea la matriz pose.

  ``` matlab
%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:96;         % Time array

initPose = [5;1;3/4*pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;
  ```
  
Ahora bien se hace la definicion de los wayspoints o puntos especificos los cuales formaran las diferentes figuras.
Es importante destacar que para cada trayectoria los waypoints o landmarks serán diferentes.
A continuación pondremos los waypoints especificos para cada trayectoria.
  ``` matlab
% Define waypoints Cereza
waypoints = [4.8,1.2; 4,2; 3,3; 3,4; 3,5; 4,6; 5,7; 6,7; 7,7; 8,6; 7,5; 6,6; 7,5; 8,5; 7,5; 8,6; 8.5,7; 9.2,8; 10,9; 9,10; 8,11; 7,11; 6,11; 5,10; 4,9; 5,9; 6,9; 7,9; 8,9; 9,9; 10,9; 9.2,8; 8.5,7; 8,6; 9,5; 9,4; 9,3; 8,2; 7,1; 6,1; 5.5,1];

% Define waypoints Flor
waypoints = [4,1; 3,1; 2,1; 1,2; 0,3; 1,3; 2,3; 3,2; 4.1,1; 5,1; 6,1; 7,2; 8,3; 7,3; 6,3; 5,2; 4,1; 4,2; 4,3; 4,4; 4,5; 3,4; 2,3; 2,4; 2,5; 1,5; 0,5; 1,6; 2,7; 3,7; 3,8; 4,8; 5,8; 5,7; 5,6; 4,6; 3,6; 3,7; 2,7; 1,8; 0,9; 1,9; 2,9; 2,10; 2,11; 3,10; 4,9; 5,10; 6,11; 6,10; 6,9; 7,9; 8,9; 7,8; 6,7; 7,6; 8,5;7,5; 6,5; 6,4; 6,3; 5,4; 4,5]

% Define waypoints Perro
waypoints = [7,0; 6,0; 5.5,1; 5,2; 5,3; 5,4; 5,5; 4,6; 3,6; 2,6; 1,6; 0.5,7; 0,8; 1,9; 0,9; 0,8; 1,9; 2,9; 2,10; 3,10; 4,10; 3,9; 3,10; 3,10.3; 4,10.6; 4,12; 5,11; 4,10.6; 5,11; 6,11; 6,10; 6,11; 6,12; 6.3,11; 6.8,10; 7,9; 6,8; 7,9; 6.8,10; 6.3,11; 6,12; 6.5,11; 7,10; 7.5,9; 8,8; 9,7; 8,6.5; 7,6; 6,5.5; 5,5; 5,4; 6,4.4; 7,4.9; 7.5,5.1; 8,5.3; 9,5.7; 10,6; 9,7; 10,6; 11,6; 11,5; 11,4; 11,3; 11,2; 11,1; 11,0; 10,0; 9,0; 9,1; 9,0; 8,0; 7,0; 6,0];
  ```
  
Posteriormente creamos el visualizador para poder definir visualmente los waypoints, asi como definimos nuestro controlador, de igual manera este fue tuneado especificamente dependiendo de la trayectoria definida.

  ``` matlab
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller Perro
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.10;
controller.DesiredLinearVelocity = 0.45;
controller.MaxAngularVelocity = 20.5;

%% Pure Pursuit Controller Flor
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.15;
controller.DesiredLinearVelocity = 0.25;
controller.MaxAngularVelocity = 18.5;

%% Pure Pursuit Controller Cereza
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.45;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 20.5;
  ```
   
Por último pero no menos importante se establece el ciclo con el cual se realizara la simulación.

  ``` matlab
%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
end
  ``
  
  # Resultados
  ## Caso 1 Cereza
  ![image](https://user-images.githubusercontent.com/99983026/236656544-bd5fe2c0-f3d6-4036-96e2-c380dd45cf97.png)
  
  ## Caso 2 Flor
  ![image](https://user-images.githubusercontent.com/99983026/236656501-b4ecc5c3-dab4-4b5b-9e11-ff1eb7c71310.png)
  
  ## Caso 3 Perro
  ![image](https://user-images.githubusercontent.com/99983026/236656289-5c345354-bcd1-4688-8749-ac29c5b98ddc.png)
  

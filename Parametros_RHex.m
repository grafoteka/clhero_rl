%%---PARÁMETROS DEL RHEX---%%

%En este fichero se encuentran las medidas del robot RHex. 
%Antes de iniciar la simulación en Simscape hay que ejecutar este código
%Para cambiar las medidas del robot basta con modificar los parámetros.


%%-parámetros del cuerpo-%%
altura=590;
base=345;
espesor_cuerpo=100;
dist1=275;
dist2=40;
dist3=60;
masa_cuerpo=5.771;
dist4=50;
dist5=altura-dist4;

%%-parámetros de la C-Leg-%%
espesor=10;
radio=100;
anchura=50;
masa_C_Leg=0.057;

%%-perfil el cuerpo-%%

perfil_cuerpo=[0 0;base 0;base dist1;base+dist3 dist1; base+dist3 dist1+dist2; base dist1+dist2; base altura; 0 altura; 0 dist1+dist2; -dist3 dist1+dist2; -dist3 dist1; 0 dist1];

%%-perfil C-Leg-%%

perfil_C_Leg=[radio-espesor 0; radio 0; radio anchura; radio-espesor anchura];

t=0:0.01:10;
tt=2*t;
cont1=20000;
cont2=500;
fric1=0.5;
fric2=0.7;
velocidad=-1;

%% Reinforcement Learning (RL) parameters
Ts = 0.025; % Agent sample time
Tf = 10;    % Simulation end time
        
% Scaling factor for RL action [-1 1]
max_torque = 3.5;

% Initial conditions
h = 4;     % Hip height [cm]
init_height = h;
vx0 = 0;    % Initial X linear velocity [m/s]
vy0 = 0;    % Initial Y linear velocity [m/s]
wx0 = 0;    % Initial X angular velocity [rad/s]
wy0 = 0;    % Initial Y angular velocity [rad/s]

% Posición inicial angular de las patas
leg_initial_pose = -4.52; %[rad]
% Initial foot positions [m]
%leftinit =  [0;0;-h/100];
%rightinit = [0;0;-h/100];

% Calculate initial joint angles
% init_angs_L = zeros(1,2);
% theta = legInvKin(upper_leg_length/100,lower_leg_length/100,-leftinit(1),leftinit(3));
% % Address multiple outputs
% if size(theta,1) == 2
%    if theta(1,2) < 0
%       init_angs_L(1) = theta(2,1);
%       init_angs_L(2) = theta(2,2);
%    else
%       init_angs_L(1) = theta(1,1);
%       init_angs_L(2) = theta(1,2);
%    end
% end
% init_angs_R = zeros(1,2);
% theta = legInvKin(upper_leg_length/100,lower_leg_length/100,-rightinit(1),rightinit(3));
% % Address multiple outputs
% if size(theta,1) == 2
%    if theta(1,2) < 0
%       init_angs_R(1) = theta(2,1);
%       init_angs_R(2) = theta(2,2);
%    else
%       init_angs_R(1) = theta(1,1);
%       init_angs_R(2) = theta(1,2);
%    end
% end
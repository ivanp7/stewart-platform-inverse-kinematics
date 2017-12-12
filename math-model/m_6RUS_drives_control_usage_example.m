% Параметры робота:
robot_constants = struct(...
    'numberOfLegs', 6, ... % количество опор
    'bodyRadius', 0.0708, ... % радиус робота (расстояние от центра до стенки), м
    'legAttachmentDisplacement', [0.015; 0.0055; 0.0425], ... % смещение точки привода, м
    'endEffectorRadius', 0.060, ... % радиус платформы, м
    'sphericalJointDisplacement', [0; 0.0055; 0], ... % смещение точки крепления сферического шарнира, м
    'jointSPositionAsymmetry', 0.5, ... % асимметрия точек креплений опор к подвижной платформе
    'legRUChainLength', 0.050, ... % длина звена R-U, м
    'legUSChainLength', 0.055 ... % длина звена U-S, м
    );
robotWallHeight = 6.45; % высота этажа робота, см

% Положение платформы:
x = 1.72;   % смещение центра, см
y = -1.48;  % смещение центра, см
z = 3.3;    % смещение центра, см
alpha = 15; % угол наклона, град
phi = 45;   % долгота оси наклона, град
theta = 0;  % широта оси наклона, град

% Положения управляемых сочленений (углы в градусах относительно вертикального луча вверх против часовой стрелки)
actuators_values = (180/pi) * ...
    m_6RUS_drives_control(x/100, y/100, (z+robotWallHeight)/100, ...
    alpha*(pi/180), phi*(pi/180), theta*(pi/180), robot_constants);

disp('Actuators values:')
disp(actuators_values)

scene = Scene;
scene.bounds.xmin = -2;
scene.bounds.xmax = 2;
scene.bounds.ymin = -2;
scene.bounds.ymax = 2;

% Walls
scene.obstacle = Item('rect',4,0.1);
% scene.obstacles(2).shape = get_vertices('rect',0.3,0.3);
scene.obstacle(1).pos = [0 -1.95 0];

scene.obstacle(2) = Item('rect',4,0.1);
% scene.obstacles(2).shape = get_vertices('rect',0.3,0.3);
scene.obstacle(2).pos = [0 1.95 0];

scene.obstacle(3) = Item('rect',0.1,4);
% scene.obstacles(2).shape = get_vertices('rect',0.3,0.3);
scene.obstacle(3).pos = [-1.95 0 0];

scene.obstacle(4) = Item('rect',0.1,4);
% scene.obstacles(2).shape = get_vertices('rect',0.3,0.3);
scene.obstacle(4).pos = [1.95 0 0];


% Obstacles
scene.obstacle(5) = Item('rect',0.2,0.3);
% scene.obstacles(1).shape = get_vertices('rect',0.2,0.3);
scene.obstacle(5).pos = [0 1 0];
scene.obstacle(5).rand_vel = [0.1 0 0];

scene.obstacle(6) = Item('rect',0.4,0.2);
% scene.obstacles(2).shape = get_vertices('rect',0.3,0.3);
scene.obstacle(6).pos = [0.6 -0.3 0];
scene.obstacle(6).rand_vel = [-0.1 0.1 0];

scene.obstacle(7) = Item('rect',0.3,0.4);
% scene.obstacles(2).shape = get_vertices('rect',0.3,0.3);
scene.obstacle(7).pos = [-1 -0.5 0];
scene.obstacle(7).rand_vel = [0.1 -0.05 0.1];

% Robot
scene.robot = Robot(0.1);
% scene.robot.shape = get_vertices('rect',0.2,0.1);
scene.robot.pos = [-1.5 -1.5 pi/2];

scene.robot.laser_scanner = LaserScanner(1,360,10);
scene.robot.max_trans_vel = 1;
scene.robot.max_rot_vel = 2;

% Goal
scene.goal = Item('rect',0.1,0.1);
scene.goal.pos  = [1.5 0 0];
scene.goal.color = 'g';


% scene.show()


% Simulating a scene


% Load and display scene
create_scene
scene.show()

% Set simulation parameters
Tsim = 10;
Tstep = 0.05;




getpts(); % Wait for mouse click

for i=1:ceil(Tsim/Tstep)
	% Move obstacles and robot
    scene.obstacle(5).random_move(Tstep)
    scene.obstacle(6).random_move(Tstep)
    scene.obstacle(7).random_move(Tstep)
    scene.robot.random_move(Tstep);
    scene.robot.laser_scanner.scan(scene);
    
    % Stop simulation when collision
    if scene.robot.check_collision(scene)
        disp('collision')
        % break;
    end
    
    % Update figure
    scene.update()
    drawnow
end


classdef LaserScanner < handle
    properties
        % Some configuration parameters of laser scanner
        range = 0.5;
        fov = deg2rad(360);
        res = deg2rad(5);
        
        % Readings of the scanner will be stored here
        angle; 
        dist;
        
        % Display
        handle;
    end
    
    methods
        function obj = LaserScanner(varargin)
            obj = obj@handle;
            % Initialize with default config parameters
            if isempty(varargin)
                obj.angle = linspace(-obj.fov/2,obj.fov/2,...
                    obj.fov/obj.res);
                obj.dist = obj.range*ones(1,length(obj.angle));
                
            % Initialize with input config parameters
            elseif length(varargin)==3
                obj.range = varargin{1};
                obj.fov = deg2rad(varargin{2});
                obj.res = deg2rad(varargin{3});
                obj.angle = linspace(-obj.fov/2,obj.fov/2,...
                    obj.fov/obj.res);
                obj.dist = obj.range*ones(1,length(obj.angle));
            else
                error('invalid input')
            end
        end
        
        function scan(obj, scene)
            % Reset output of the scanner
            obj.dist(:) = obj.range;
            
            % Find the angle of center of obstacles w.r.t robot and also 
            % the angle made by the bounding radius (angular_width) with 
            % the center of the robot
            num_obstacles = length(scene.obstacle);
            angle_center = zeros(1,num_obstacles);
            angular_width = zeros(1,num_obstacles);
            for i = 1:num_obstacles
                delta = scene.obstacle(i).pos(1:2)-scene.robot.pos(1:2);
                angle_center(i) = angle_vec(delta);
                angular_width(i) = asin(scene.obstacle(i).radius/norm(delta));
            end
            
            % For each obstacle find the laser rays that fall within the 
            % angular width. For each of these laser rays find the
            % first intersection of the ray with the obstacle and find the
            % distance. If the distance is less than range or the current 
            % reading, store it. 
            for i = 1:num_obstacles
                % fprintf("Obstacle %d:\n",i);
                for j=1:length(obj.angle)
                    % fprintf("angle %f:\n",obj.angle(j));
                    angular_proximity = abs(mapangle(angle_center(i) - ...
                        (scene.robot.pos(3) + obj.angle(j))));
                    % angular_proximity = 0;
                    % disp(angular_proximity)
                    if angular_proximity <= angular_width(i)
                        dist_tmp = ray_intersect_obstacle(scene.robot.pos(1:2)...
                            ,scene.robot.pos(3)+obj.angle(j),...
                            scene.obstacle(i));
                        % fprintf("dist_tmp: %f\n",dist_tmp);
                        if dist_tmp < obj.dist(j)
                            obj.dist(j) = dist_tmp;
                        end
                    end
                end
            end
        end
        
        function add_to_display(obj,robot_pos)
            X = obj.dist.*cos(obj.angle);
            Y = obj.dist.*sin(obj.angle);
            points = [X;Y];
            R = rot2d(robot_pos(3))';
            points = R*points + robot_pos(1:2)';
            obj.handle =  plot(points(1,:),points(2,:),'r.');
        end
        
        function display_scan(obj)
            figure(2);
            axis equal
            hold on
            xlim(1.2*obj.range*[-1 1]);
            ylim(1.2*obj.range*[-1 1]);
            X = obj.dist.*cos(obj.angle);
            Y = obj.dist.*sin(obj.angle);
            obj.handle =  plot(X,Y,'r--*');
        end
        
        function update_display(obj,robot_pos)
            X = obj.dist.*cos(obj.angle);
            Y = obj.dist.*sin(obj.angle);
            points = [X;Y];
            R = rot2d(robot_pos(3));
            points = R*points + robot_pos(1:2)';
            set(obj.handle,'XData',points(1,:));
            set(obj.handle,'YData',points(2,:));
%             X = obj.dist.*cos(obj.angle);
%             Y = obj.dist.*sin(obj.angle);
%             set(obj.handle,'XData',X);
%             set(obj.handle,'YData',Y);
        end
    end
end





function y = angle_vec(X)
% To find the angle of a vector in 2D
y = atan2(X(2),X(1));
end

function y = ray_intersect_lineseg(source,ang,p1,p2)
% To find the intersection of a laser ray with a line segement
% laser ray given by source and ang
% line segment given by p1 and p2
p1 = p1-source;
p2 = p2-source;
% Inf indicates no itersection
if(abs(mapangle(angle_vec(p2-p1)-ang)) < 10^-6)
    % ray and line segment p1p2 are parallel
    % no intersection
    y = Inf; 
else
    u = -(sin(ang)*p1(1)-p1(2)*cos(ang))/...
        (sin(ang)*(p2(1)-p1(1))- cos(ang)*(p2(2)-p1(2)));
    
    ray = p1 + (p2-p1)*u;
    ang_ray = atan2(ray(2),ray(1));
    
    % u needs to be between 0 and 1 for the intersection to lie in between 
    % p1 and p2
    
    % ang & ang_ray must correspond to the same angle, else it would mean
    % interection in the opposite direction
    if u>=0 && u<=1 && abs(mapangle(ang_ray-ang)) < 10^-6
        y = norm(p1 + (p2-p1)*u);
    else
        y = Inf;
    end
end
end

function y = ray_intersect_obstacle(source,ang,obstacle)
% To find the first the intersection of laser ray with obstacle
y = Inf; % Inf indicates no itersection
n = length(obstacle.shape);
% Loop over all line segments, the intersection closest to the source 
% is the first intersection
for i = 1:n
    i1 = i;
    if i == n
        i2 = 1;
    else
        i2 = i+1;
    end
    % intersection with line segement
    R = rot2d(obstacle.pos(3));
    p1 = (R*obstacle.shape(i1,:)')' + obstacle.pos(1:2);
    p2 = (R*obstacle.shape(i2,:)')' + obstacle.pos(1:2);
    
    y_tmp = ray_intersect_lineseg(source,ang,p1,p2);
    
    % to find the closest intersection
    if y_tmp < y
        y = y_tmp;
    end
end
end
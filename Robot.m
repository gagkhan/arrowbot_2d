classdef Robot < Item
    methods
        function obj = Robot(a,varargin)
            obj = obj@Item('robot',a,varargin);
            obj.color = 'y';
        end
    end
    
    properties
        laser_scanner;
        max_trans_vel;
        max_rot_vel;
    end
    
    methods
        function y = collision_with_obstacle(obj,obstacle)
            center_dist = norm(obj.pos(1:2)-obstacle.pos(1:2));
            y = false;
            if center_dist <= (obj.radius + obstacle.radius)
                n = length(obstacle.shape);
                for i = 1:n
                    i1 = i;
                    if i == n
                        i2 = 1;
                    else
                        i2 = i+1;
                    end
                    dist = dist_to_lineseg(obj.pos(1:2),...
                        obstacle.shape(i1,:)+obstacle.pos(1:2),...
                        obstacle.shape(i2,:)+obstacle.pos(1:2));
                    if dist <= obj.radius
                        y = true;
                        break;
                    end
                end
            end
        end
        
        function y = check_collision(obj,scene)
            num_obstacles = length(scene.obstacle);
            y = false;
            for i = 1:num_obstacles
                if obj.collision_with_obstacle(scene.obstacle(i))
                    y = true;
                    break;
                end
            end
        end
        
        function move(obj,controller,goal,Tstep)
            delta = goal(1:2) - obj.pos(1:2);
            ang = mapangle(atan2(delta(2),delta(1))-obj.pos(3));
            dist = norm(delta);
            y = controller.ff(cat(2,ang,dist,obj.laser_scanner.dist));
            v_trans = y(1)*obj.max_trans_vel;
            v_rot = (y(2)-0.5)*obj.max_rot_vel;
            obj.pos(1:2) = obj.pos(1:2) + ...
                [cos(obj.pos(3)) sin(obj.pos(3))]*v_trans*Tstep;
            obj.pos(3) = obj.pos(3) + v_rot*Tstep;
%             a_trans = (y(1)-0.5)*obj.max_trans_accel;
%             a_rot = (y(2)-0.5)*obj.max_rot_accel;
%             rot = obj.pos(3) + obj.vel(3)*Tstep + 0.5*a_rot*Tstep;
%             rot_mid = 0.5*(rot+obj.pos(3));
%             obj.pos(1:2) = obj.pos(1:2) + obj.vel(1:2)*Tstep +...
%                 [cos(rot_mid) sin(rot_mid)]*a_trans*Tstep^2;
%             obj.pos(3) = rot;
%             obj.vel(3) = obj.vel(3) + a_rot*Tstep;
%             obj.vel(1:2) = obj.vel(1:2) +  ...
%                 [cos(rot_mid) sin(rot_mid)]*a_trans*Tstep;
        end
    end
end

function y = dist_to_lineseg(p,p1,p2)
if norm(p1-p2) <= 10^-6
    error('p1 = p2')
else
    a = p2(2) - p1(2);
    b = - (p2(1)-p1(1));
    c = p1(2)*(p2(1)-p1(1)) - p1(1)*(p2(2)-p1(2));
    y = abs((a*p(1)+b*p(2)+c)/sqrt(a^2+b^2));
end
end
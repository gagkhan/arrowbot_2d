classdef Item < handle
    properties
        shape
        pos
        handle
        color = 'k'
        radius
        rand_vel = [0 0 0]
    end
    methods
        function obj = Item(type,a,varargin)
            switch type
                case 'tri'
                    % equilateral triangle
                    % a = varargin{1}/sqrt(3);
                    obj.shape(1,:) = [a 0];
                    obj.shape(2,:) = [-a/2, a*sqrt(3)/2];
                    obj.shape(3,:) = [-a/2, -a*sqrt(3)/2];
                    obj.radius = a;
                case 'rect'
                    % rectangle length a breadth b
                    % a = varargin{1};
                    b = varargin{1};
                    obj.shape(1,:) = 0.5*[ a  b];
                    obj.shape(2,:) = 0.5*[-a  b];
                    obj.shape(3,:) = 0.5*[-a -b];
                    obj.shape(4,:) = 0.5*[ a -b];
                    obj.radius = sqrt((a^2 + b^2)/4);
                case 'robot'
                    obj.shape(1,:) = [1.5*a 0];
                    obj.shape(2,:) = [-a/2, a*sqrt(3)/2];
                    obj.shape(3,:) = [-a/2, -a*sqrt(3)/2];
                    obj.radius = a;
            end
        end
        function add_to_display(obj,scene_handle)
            ang = obj.pos(3);
            R = [cos(ang) -sin(ang);sin(ang) cos(ang)];
            shape_rotated = R*obj.shape';
            X = shape_rotated(1,:) + obj.pos(1);
            Y = shape_rotated(2,:) + obj.pos(2);
            figure(scene_handle)
            obj.handle = fill(X,Y,obj.color);
        end
        function update_display(obj)
            ang = obj.pos(3);
            R = [cos(ang) -sin(ang);sin(ang) cos(ang)];
            shape_rotated = R*obj.shape';
            X = shape_rotated(1,:) + obj.pos(1);
            Y = shape_rotated(2,:) + obj.pos(2);
            set(obj.handle,'XData',X);
            set(obj.handle,'YData',Y);
        end
        
        function random_move(obj,Tstep)
            obj.pos = obj.pos + Tstep*obj.rand_vel;
            % obj.pos = (rand(1,3)-0.5);
        end
    end
end
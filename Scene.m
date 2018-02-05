classdef Scene < handle
    properties
        bounds
        obstacle
        robot
        goal
        handle
        time
    end
    
    properties
        TSTEP = 0.1;
        TSIM = 10;
    end
    
    methods
        function obj = Scene()
           obj = obj@handle;
           obj.bounds.xmin = -1;
           obj.bounds.xmax = 1;
           obj.bounds.ymin = -1;
           obj.bounds.ymax = 1;
        end
        function show(obj)
            obj.handle = figure(1);
            hold on
            axis equal
            grid on
            xlim([obj.bounds.xmin obj.bounds.xmax])
            ylim([obj.bounds.ymin obj.bounds.ymax])
            for i = 1:length(obj.obstacle)
                obj.obstacle(i).add_to_display(obj.handle);
            end
            obj.robot.add_to_display(obj.handle)
            obj.goal.add_to_display(obj.handle)
            % obj.robot.laser_scanner.scan(obj);
            obj.robot.laser_scanner.add_to_display(obj.robot.pos);
        end
        
        function update(obj)
            for i = 1:length(obj.obstacle)
                obj.obstacle(i).update_display()
            end
            obj.robot.update_display();
            obj.goal.update_display();
            obj.robot.laser_scanner.update_display(obj.robot.pos);
        end
    end
end
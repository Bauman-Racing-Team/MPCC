classdef State < matlab.mixin.Copyable
    %STATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        x
        y
        yaw
        vx
        vy
        r
        s
        throttle
        steeringAngle
        brakes
        vs
    end
    
    methods
        function obj = State(x,y,yaw,vx,vy,r,s,throttle,steeringAngle,brakes,vs)
            if nargin > 0
                obj.x = x;
                obj.y = y;
                obj.yaw = yaw;
                obj.vx = vx;
                obj.vy = vy;
                obj.r = r;
                obj.s = s;
                obj.throttle = throttle;
                obj.steeringAngle = steeringAngle;
                obj.brakes = brakes;
                obj.vs = vs;
            end
        end
        
        function setZero(obj)
            obj.x = 0.0;
            obj.y = 0.0;
            obj.yaw = 0.0;
            obj.vx = 0.0;
            obj.vy = 0.0;
            obj.r = 0.0;
            obj.s = 0.0;
            obj.throttle = 0.0;
            obj.steeringAngle = 0.0;
            obj.brakes = 0.0;
            obj.vs = 0.0;
        end

        function vxNonZero(obj,vxZero)
            if obj.vx < vxZero
              obj.vx = vxZero;
              %obj.vy = 0.0;
              %obj.r = 0.0;
              %obj.steeringAngle = 0.0;
            end
        end
    end
end


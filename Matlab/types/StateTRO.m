classdef StateTRO < matlab.mixin.Copyable
    %STATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        s
        n
        miu
        vx
        vy
        r
        throttle
        steeringAngle
        brakes
    end
    
    methods
        function obj = StateTRO(s,n,miu,vx,vy,r,throttle,steeringAngle,brakes)
            if nargin > 0
                obj.s = s;
                obj.n = n;
                obj.miu =miu;
                obj.vx = vx;
                obj.vy = vy;
                obj.r = r;
                obj.throttle = throttle;
                obj.steeringAngle = steeringAngle;
                obj.brakes = brakes;
            end
        end
        
        function setZero(obj)
            obj.s = 0.0;
            obj.n = 0.0;
            obj.miu = 0.0;
            obj.vx = 0.0;
            obj.vy = 0.0;
            obj.r = 0.0;
            obj.throttle = 0.0;
            obj.steeringAngle = 0.0;
            obj.brakes = 0.0;
        end

        function unwrap(obj,trackLength)
            if obj.miu > pi
              obj.miu = obj.miu - 2.0 * pi;
            end
            if obj.miu < -pi
              obj.miu = obj.miu + 2.0 * pi;
            end
            if obj.s > trackLength
              obj.s = obj.s - trackLength;
            end
            if obj.s < 0.0
              obj.s = obj.s + trackLength;
            end
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


classdef Parameters
    %PARAMETERS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        bounds
        costs
        car
        tire
        config
    end
    
    methods
        function obj = Parameters(vehicle)
            %load bounds
            fname = "../data/params/" + vehicle + "/bounds.json";
            disp(fname);
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.bounds = jsondecode(str);

            %load config
            fname = "../data/params/" + vehicle + "/mpcc.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.config = jsondecode(str);

            %load costs
            fname = "../data/params/" + vehicle + "/cost.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.costs = jsondecode(str);

            %load car parameters
            fname = "../data/cars/" + vehicle + "/car.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.car = jsondecode(str);

            %load tire coefficients
            fname = "../data/cars/" + vehicle + "/tire.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.tire = jsondecode(str);
        end
    end
end


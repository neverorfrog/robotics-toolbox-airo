classdef robot

    properties
        Property1
    end

    methods
        function obj = robot(links)
        %ROBOT Construct an instance of this class
        %   Detailed explanation goes here
        obj.Property1 = inputArg1 + inputArg2;
        end

        function outputArg = method1(obj,inputArg)
        %METHOD1 Summary of this method goes here
        %   Detailed explanation goes here
        outputArg = obj.Property1 + inputArg;
        end
    end
end
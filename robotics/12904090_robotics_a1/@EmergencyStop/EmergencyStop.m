classdef EmergencyStop
    %   Class creates a handle to display EmergencyStop and handles position
    properties
        handle;
        pose;
    end
    
    methods
        function self = EmergencyStop(pose)
            % read ply file for vertex data
            [f,v,data] = plyread('emergencyStop.ply','tri');
            vertexCount = size(v,1);

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            % set current pose and adjust vertices as appropriate
            self.pose = pose;
            updatedPoints = [self.pose * [v,ones(vertexCount,1)]']';
            
            % Then plot Emergency stop
            self.handle = trisurf(f ...
                , updatedPoints(:,1), updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
        end
    end
end


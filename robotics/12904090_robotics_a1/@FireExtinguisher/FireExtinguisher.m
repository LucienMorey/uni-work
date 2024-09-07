classdef FireExtinguisher
    %   Class creates a handle to display fire extinguisher and handles position
    
    properties
        handle;
        pose;
    end
    
    methods
        function self = FireExtinguisher(pose)
            % read ply file for vertex data
            [f,v,data] = plyread('fireExtinguisher.ply','tri');
            vertexCount = size(v,1);

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            %set the current pose and shit the mesh vertices
            self.pose = pose;
            updatedPoints = [self.pose * [v,ones(vertexCount,1)]']';
            
            % plot the fire extinguisher
            self.handle = trisurf(f ...
                , updatedPoints(:,1), updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
        end
    end
end


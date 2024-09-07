classdef Table
    %   Class creates a handle to display table and handles position
    
    properties
        handle;
        TABLE_OFFSET;
    end
    
    methods
        function self = Table(position)
            %set offset for surface of table based on inspected value
            self.TABLE_OFFSET = position(3,4) + 0.12;
            % read ply file for vertex data
            [f,v,data] = plyread('table.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Then plot the table with shifted vertices
            self.handle = trisurf(f ...
                , v(:,1)+position(1,4), v(:,2)+position(2,4), v(:,3)+position(3,4) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
end


classdef RobotLucien < handle
    %ROBOTLUCIEN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %> Robot model
        model;
        %> Flag to indicate if gripper is used
        useGripper = false;

        maxReach;

        placement_handle;

        state=0;
        lastState=0;
        stepCounter=1;
        trajectory;
        brickToTrack = 0;
        finishedPlacement = false;
        
        volumeSurfaceHandle;
        XYReachHandle;
        XZReachHandle;
        YZReachHandle;
        
        workspacePointcloud;
        tri;
        workspaceVolume;

        XYReachPoints;
        YZReachPoints;
        XZReachPoints;
    end
    properties(Constant)
        WAITING = 0;
        TRAVERSE_PICK_UP=1;
        PICK_UP = 2;
        TRAVERSE_PLACE = 3;
        PLACE = 4;
        HOME = 5;

        TRAJ_STEPS = 100;
        LINEAR_RESOLUTION = 0.1;
        ANGULAR_RESOLUTION= deg2rad(10);
    end
    methods
        function self = RobotLucien()
            
        end


% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self,modelPrexix)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([modelPrefix,num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([modelPrexix,num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end
    
    % Display robot
    self.model.plot3d([0,0,0,0,0,0],'noarrow');
    
    t1 = self.model.base * ...
    trotz(self.model.theta(1)) * ...
    transl(self.model.a(1), 0, self.model.d(1)) * ...
    trotx(self.model.alpha(1));

    t2 = self.model.fkine([0, -pi/2, 0, -pi/2, 0, 0]);


    td = t1 \ t2;
    self.maxReach = sqrt(td(1, 4)^2 + td(2, 4)^2 + td(3, 4)^2);
    
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end

function [totalVolume,totalArea] = stlVolume(self, p,t)
% Given a surface triangulation, compute the volume enclosed using
% divergence theorem.
% Assumption:Triangle nodes are ordered correctly, i.e.,computed normal is outwards
% Input: p: (3xnPoints), t: (3xnTriangles)
% Output: total volume enclosed, and total area of surface  
% Author: K. Suresh; suresh@engr.wisc.edu

% Compute the vectors d13 and d12
d13= [(p(1,t(2,:))-p(1,t(3,:))); (p(2,t(2,:))-p(2,t(3,:)));  (p(3,t(2,:))-p(3,t(3,:)))];
d12= [(p(1,t(1,:))-p(1,t(2,:))); (p(2,t(1,:))-p(2,t(2,:))); (p(3,t(1,:))-p(3,t(2,:)))];
cr = cross(d13,d12,1);%cross-product (vectorized)
area = 0.5*sqrt(cr(1,:).^2+cr(2,:).^2+cr(3,:).^2);% Area of each triangle
totalArea = sum(area);
crNorm = sqrt(cr(1,:).^2+cr(2,:).^2+cr(3,:).^2);
zMean = (p(3,t(1,:))+p(3,t(2,:))+p(3,t(3,:)))/3;
nz = -cr(3,:)./crNorm;% z component of normal for each triangle
volume = area.*zMean.*nz; % contribution of each triangle
totalVolume = sum(volume);%divergence theorem
end

function DisplayVolumeWorkSpace(self,displayBool)
    if displayBool == true
        self.volumeSurfaceHandle.Visible = true;
        disp(self.workspaceVolume);
    else
        self.volumeSurfaceHandle.Visible = false;
    end
end 

function DisplayReachNets(self, displayBool)
    self.XYReachHandle.Visible = displayBool;
    self.XZReachHandle.Visible = displayBool;
    self.YZReachHandle.Visible = displayBool;
end
    end
end


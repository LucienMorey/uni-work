classdef RoboticsSimulator < handle
    %ROBOTICSSIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ax;
        bricks = Brick.empty;
        Fence;
        Table;
        FireExtinguisherInternal;
        FireExtinguisherExternal;
        EStop;
        UR3;
        UR5;
        Logger;
        
        wall = {};

        allBricksPlaced = false;
    end
    
    methods
        function self = RoboticsSimulator(UR3Pose, UR5Pose, BrickPositions, wallPose, wallSize)            
            self.ax = axes();
            xlim(self.ax, [-3 3]);
            ylim(self.ax, [-3 3]);
            zlim(self.ax, [0 3]);
            view(self.ax, 3)
            hold(self.ax, 'on')
            
            % Create Table instance
            self.Table = Table(transl(0,0,0.9));
            % Create Fence instance
            self.Fence = Fence(transl(0,0,0));
            % Create Fire Extinguisher Class
            self.FireExtinguisherInternal = FireExtinguisher(transl(2.4,1,self.Table.TABLE_OFFSET));
            self.FireExtinguisherExternal = FireExtinguisher(transl(-2.3,-1.3,self.Table.TABLE_OFFSET));
            % Create EStop Class
            self.EStop = EmergencyStop(transl(-2.5,2.7,self.Table.TABLE_OFFSET)*troty(-pi/2));
            % self.EStop
            self.Logger = log4matlab('assignmentLogFile.log');
            
            % Spawn all bricks at table height
            for i = 1:1:size(BrickPositions,1)
                self.bricks(i) = Brick(BrickPositions(i,:), self.Table.TABLE_OFFSET);
                self.Logger.mlog = {log4matlab.DEBUG, 'Simulator', ['brick ',num2str(i),' spawned at ', self.Logger.MatrixToString(self.bricks(i).pose)]};

            end

            % get dimensions of the brick for future calculations
            bheight = max(self.bricks(1).handle.Vertices(:, 3))-min(self.bricks(1).handle.Vertices(:, 3));
            bwidth  = max( self.bricks(1).handle.Vertices(:, 1))-min( self.bricks(1).handle.Vertices(:, 1));
            
            % determine the transform to the bottom left brick because this will be brick 1
            dleft = wallPose(1, 4) - (0.5 * wallSize(1) - 0.5) * bwidth;
            leftBrick = wallPose * transl(dleft,0,0);

            % for each brick in the wall
            for i = 1:1:wallSize(1)
                for j = 1:1:wallSize(2) 

                    % brick location
                    tfb = leftBrick * transl( (i-1) * bwidth, 0, (j-1) * bheight + self.Table.TABLE_OFFSET);

                    % brick list idx
                    n = wallSize(1)*(j-1)+i;
                    % add to array
                    self.bricks(n).placementPose = tfb;
                    self.Logger.mlog = {log4matlab.DEBUG, 'Simulator', ['brick ',num2str(n),' will be placed at ', self.Logger.MatrixToString(tfb)]};
                end
            end
            
            % Spawn in UR3
            % alter z offset to be at table height
            UR3Pose(3,4)=self.Table.TABLE_OFFSET;
            self.UR3 = UR3(UR3Pose,'l', false);
            
            % spawn in UR5
            % alter z offset to be at table height 
            UR5Pose(3,4) =self.Table.TABLE_OFFSET;
            self.UR5 = LinearUR5Lucien(UR5Pose, 'LinUR5Link', false);
            
        end
        
        % Display the trisurf representing volume of workspace for both arms
        % TODO Volume workspace for UR3
        function DisplayVolumeWorkSpaces(self, displayBool)
            self.UR3.DisplayVolumeWorkSpace(displayBool);
            self.UR5.DisplayVolumeWorkSpace(displayBool);
            self.Logger.mlog = {log4matlab.DEBUG, 'Simulator', 'Toggling display of volume surfaces'};

        end

        % Display the lines representing the reach for both arms
        % TODO fix xz for UR5
        function DisplayReachNets(self, displayBool)
            self.UR3.DisplayReachNets(displayBool);
            self.UR5.DisplayReachNets(displayBool);
            self.Logger.mlog = {log4matlab.DEBUG, 'Simulator', 'Toggling display of reach nets'};
        end
        

        % Agnostic state machine update for robots
        function updateRobotState(self, robot)
            %Check current state
            switch(robot.state)
            % If waiting then check for any remaining bricks to be placed
            % Transition to pickup if there are otherwise indicate finished placement for robot
            case robot.WAITING
                % Check for remaining bricks
                self.CheckForFreeBricks(robot);

                % if still waiting after check and last state was home then done placment
                if (robot.state == robot.WAITING) && (robot.lastState == robot.HOME)
                    robot.trajectory = [];
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' made it home']};
                    robot.finishedPlacement = true;
                end
                
                % Update last state
                robot.lastState = robot.WAITING;
                
                % Reset step counter if changing states
                if robot.lastState ~= robot.state
                    robot.stepCounter = 1;
                end

            % If picking up and just transitioned into this state the n plan trajectory to brick
            % check if at end of trajectory and trnasition top placement if its time
            case robot.PICK_UP
                % if just entered state plan trajectory
                if (robot.lastState == robot.WAITING) || (robot.lastState == robot.PLACE)
                    % determine current joint angles
                    currentQ = robot.model.getpos;

                    % determine waypoint
                    waypointT = self.bricks(robot.brickToTrack).pose * transl(0,0,0.5);
                    waypointQ = robot.model.ikcon(waypointT, currentQ);
                    % plan Trajectory segment
                    waypointSegment = jtraj(currentQ, waypointQ, robot.TRAJ_STEPS);

                    % determine goal
                    goalT = self.bricks(robot.brickToTrack).pose * trotx(pi);
                    goalQ = robot.model.ikcon(goalT, waypointQ);

                    % get Goal segment of trajectory
                    goalSegment = jtraj(waypointQ,goalQ,robot.TRAJ_STEPS);
                    
                    % combine
                    robot.trajectory = [waypointSegment; goalSegment];

                    % log goal joints and pose
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' pickup goal T is at ', self.Logger.MatrixToString(goalT)]};
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' pickup goal Q is at ', self.Logger.MatrixToString(goalQ)]};
                end
                
                % if at final pose then transision
                if robot.model.getpos == robot.trajectory(end,:)
                    % log actual final pose compared to last pose in trajectory
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' pickup goal Q was at ', self.Logger.MatrixToString(robot.trajectory(end,:))]};
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' final Q is ', self.Logger.MatrixToString(robot.model.getpos)]};

                    % empty trajectory  
                    robot.trajectory = [];

                    %transition state
                    robot.state = robot.PLACE;

                    %log state transition
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' transitioning to place state']};

                end

                % update last state
                robot.lastState = robot.PICK_UP;
                
                % reset step couinter if current state and last state arent equal
                if robot.lastState ~= robot.state
                    robot.stepCounter = 1;
                end

            % if placing and just transitioned from pick up then set possession of brick so brick pose is updated along with arm
            % check for remaining bricks after placement and move to home if there are none
            case robot.PLACE
                if robot.lastState == robot.PICK_UP
                    %set possession of brick
                    self.bricks(robot.brickToTrack).SetPossession(str2num(robot.model.name));
                    
                    % determine current joint angles
                    currentQ = robot.model.getpos;
                    goalT = self.bricks(robot.brickToTrack).placementPose*trotx(pi);
                    % determine waypoint
                    waypointQ = robot.GenerateWaypointQ(goalT);
                    % Plan Trajectory segment
                    waypointSegment = jtraj(currentQ, waypointQ, robot.TRAJ_STEPS);

                    % determine goal
                    goalQ = robot.model.ikcon(goalT, waypointQ);
                    goalSegment = jtraj(waypointQ,goalQ,robot.TRAJ_STEPS);
                    
                    % combine
                    robot.trajectory = [waypointSegment; goalSegment];
                    
                    % log goal joints and pose
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' place goal T is at ', self.Logger.MatrixToString(goalT)]};
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' final Q is ', self.Logger.MatrixToString(goalQ)]};
                end

                if robot.model.getpos == robot.trajectory(end,:)  % (fl;oating ppooint comparision)
                    % log actual final pose compared to last pose in trajectory
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' place goal Q was at ', self.Logger.MatrixToString(robot.trajectory(end,:))]};
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' final Q is ', self.Logger.MatrixToString(robot.model.getpos)]};
                    
                    %empty trajectory
                    robot.trajectory = [];
                    
                    % update posessiona dn drop brick
                    self.bricks(robot.brickToTrack).SetPossession(Brick.PLACED);
                    
                    %handle state transisiton
                    % check if bricks need to be picked up and break
                    self.CheckForFreeBricks(robot);
                    % if state still == place then no bricks left so send robot home
                    if robot.state == robot.PLACE
                        robot.state = robot.HOME;
                        self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' transitioning to home state']};
                    end
                end

                % update last state
                robot.lastState = robot.PLACE;

                %reset step counter if changed state
                if robot.lastState ~= robot.state
                    robot.stepCounter = 1;
                end

            %home state is about returning to zero position and awaiting next commands
            case robot.HOME
                % if just entered state then plan traj
                if robot.lastState == robot.PLACE
                    % get current joint angles
                    currentQ = robot.model.getpos;
                    %plan to home pose
                    robot.trajectory = jtraj(currentQ,robot.Q_HOME,robot.TRAJ_STEPS);
                end
                % check if its reached the end and transision
                if robot.model.getpos == robot.trajectory(end,:)
                    %Empty trajectory
                    robot.trajectory = [];

                    %state transition
                    robot.state = robot.WAITING;
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' transitioning to waiting state']};
                end

                % update last state
                robot.lastState = robot.HOME;

                %reset step counter if transitioned state
                if robot.lastState ~= robot.state
                    robot.stepCounter = 1;
                end
             
            otherwise
                % log that system has entered unknown state
                self.Logger.mlog = {log4matlab.ERROR,'Simulator', ['UR',robot.model.name, ' in UNKNOWN STATE']};
            end
        end
        
        % handle state transition and animation of objects
        function updateAssetPose(self)
            % update manipulator states
            self.updateRobotState(self.UR3);
            self.updateRobotState(self.UR5);
            
            % animate trajectory if there are poses to animate
            if size(self.UR3.trajectory,1) > 0
                self.UR3.model.animate(self.UR3.trajectory(self.UR3.stepCounter, :));
                self.UR3.stepCounter=self.UR3.stepCounter+1;
            end

            if size(self.UR5.trajectory,1) > 0
                self.UR5.model.animate(self.UR5.trajectory(self.UR5.stepCounter, :));
                self.UR5.stepCounter=self.UR5.stepCounter+1;
            end
            
            %animate brick motion if held by a robot
            for i = 1:1:size(self.bricks,2)
                if self.bricks(i).possession > 0
                    if self.bricks(i).possession == Brick.UR3_POSSESSION
                        self.bricks(i).updatePose(self.UR3.model.fkine(self.UR3.model.getpos)*troty(pi));
                    elseif self.bricks(i).possession == Brick.UR5_POSSESSION
                        self.bricks(i).updatePose(self.UR5.model.fkine(self.UR5.model.getpos)*troty(pi));
                    end
                end
            end
            
            %check if both robots can no longer place bricks
            if (self.UR3.finishedPlacement == true) && (self.UR5.finishedPlacement == true)
                self.allBricksPlaced = true;
            end
        end

        function CheckForFreeBricks(self, robot)
            % check all bricks to see if a robot can pick any up
            for i=1:1:size(self.bricks,2)
                % if found a brick hasnt yet been interacted with and is within range of robot mark it for pickup
                if (self.bricks(i).possession == Brick.FREE) && (robot.BrickInRange(self.bricks(i).pose) == true)
                    %empty trajectory
                    robot.trajectory = [];
                    
                    %set brick marked so it doesnt get targeted by other arm
                    self.bricks(i).SetPossession(Brick.MARKED);
                    % set brick to track
                    robot.brickToTrack = i;
                    % transistion state
                    robot.state = robot.PICK_UP;
                    %lof state transistion
                    self.Logger.mlog = {log4matlab.DEBUG,'Simulator', ['UR',robot.model.name, ' transistioning to pickup']};
                    break;
                end
            end
        end

        function BuildWall(self)
            self.Logger.mlog = {log4matlab.DEBUG,'Simulator', 'commensing brick build'};
            %build wall until both robots indicate done
            while self.allBricksPlaced == false
                self.updateAssetPose();
                pause(0.01);
            end
        end

    end
end


% TRAJECTORY FOLLOWING
% 
% % uav poses
% p1 = transl(0,0,0);
% p2 = p1* transl(0,0,10);
% p3 = p2 * trotx(10 *pi/180);
% p4 = transl(0,2,0)*p3;
% p5 = p4 * trotx(-10* pi/180);
% p6 = p5 * troty(20 * pi/180);
% p7 = transl(2,0,0) * p6;
% p8 = p7 * troty(-20 * pi/180);
% p9 = p8 * transl(0,0,-10);
% 
% % heard size
% number_of_cows = 10;
% 
% % combining poses into trajectory
% trajectory = cat(3,p1,p2,p3,p4,p5,p6,p7,p8,p9);
% 
% % creating uav handle
% uav_H = trplot(p1, 'frame','UAV', 'color', 'b');
% 
% % creating cowheard
% cowheard = RobotCows(number_of_cows);
% eul = rotm2eul(p1(1:3,1:3));
% quat = rotm2quat(p1(1:3,1:3));
% message = strcat('RPY ', num2str(eul(:).'), ' QUAT ', num2str(quat(:).'));
% rot_text = text(1, 1, 1, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
% % begin uav movement
% for i = 1:size(trajectory,3)-1
%     % increment secondary counter
%     j = i+1;
%     
%     % generate local trajectory between poses
%     trajectory_local = ctraj(trajectory(:, :, i), trajectory(:, :, j),50);
%     
%     % step through local trajectory
%     for pose_idx =1:size(trajectory_local, 3)
%         eul = rotm2eul(trajectory_local(1:3,1:3,pose_idx));
%         quat = rotm2quat(trajectory_local(1:3,1:3,pose_idx));
%         message = strcat('RPY ', num2str(eul(:).'), ' QUAT ', num2str(quat(:).'));
%         set(rot_text, 'str', message);
%         % plot current uav pose
%         trplot(uav_H, trajectory_local(:,:,pose_idx));
%         % cow step
%         cowheard.PlotSingleRandomStep();
%     end
%     
%     for cow_idx = 1:number_of_cows
%         transform = cowheard.cow{cow_idx}.base\trajectory(:,:,j)
%     end
% end

%COW FOLLOWING
% cowHeard = RobotCows(1);
% uav_T = cowHeard.cow{1}.base * transl(0,0,5);
% uav_h = trplot(uav_T, 'frame', 'UAV', 'color', 'b');
% 
% for i = 1:100
%     cowHeard.PlotSingleRandomStep();
%     uav_T = cowHeard.cow{1}.base * transl(0,0,5);
%     trplot(uav_h, uav_T);
% end

%ARM MODELLING
L1 = Link('d',0.0,'a',2.0,'alpha',0.0,'offset',0.0,'qlim', [-pi,pi]);
L2 = Link('d',0.0,'a',2.0,'alpha',0.0,'offset',0.0,'qlim', [-pi,pi]);
L3 = Link('d',0.0,'a',2.0,'alpha',0.0,'offset',0.0,'qlim', [-pi,pi]);
robot = SerialLink([L1, L2, L3],'name','myRobot');
q = zeros(1,3); % This creates a vector of n joint angles at 0.
workspace = [-10 10 -10 10 -10 10];
scale = 1;
robot.plot(q,'workspace',workspace,'scale',scale); 



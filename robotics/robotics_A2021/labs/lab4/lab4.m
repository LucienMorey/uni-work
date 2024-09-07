%% QUESTION 1
close all;
clear;

L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
robot = SerialLink([L1 L2 L3],'name','myRobot');

robot.base = troty(pi);

q = zeros(1,3);
robot.plot(q,'workspace',[-2 2 -2 2 -0.05 2],'scale',0.5);
hold on
robot.teach;
startQ = robot.ikine(transl(-0.75,-0.5,0),q,[1,1,0,0,0,0]);

robot.plot(startQ, 'trail', '-');
rh = findobj('Tag', robot.name); ud = rh.UserData; hold on; ud.trail = plot(0,0,'-'); set(rh,'UserData',ud);
% pause;

% for i = -0.5:0.25:0.5
%     destQ = robot.ikine(transl(-0.75,i,0),startQ,[1,1,0,0,0,0]);
%     T = robot.fkine(destQ);
%     plot(T(1,4), T(2,4), 'o');
%     traj = jtraj(startQ,destQ,50);
    
%     for j=1:1:size(traj,1)
%         robot.animate(traj(j,:))
%         drawnow();
%     end
%     startQ = destQ;
% end

pause;%% Now use the same robot and draw a circle around the robot at a distance of 0.5
hold on;
newQ = robot.ikine(transl(0,0,0),q,[1,1,0,0,0,0]);
plot(-0.5,0,'r.')
for circleHalf = 1:2
    for x = -0.5:0.05:0.5        
        if circleHalf == 1            
            y = sqrt(0.5^2-x^2)        
        else            
            x = -x;            
            y = -sqrt(0.5^2-x^2);        
        end        
        plot(x,y,'r.');        
        newQ = robot.ikine(transl(x,y,0),newQ,[1,1,0,0,0,0]);%,'alpha',0.01);        
        robot.animate(newQ);        
        drawnow();    
    end
end

%% Question 2
close all;
clear;

mdl_puma560;

T1 = transl(0.5,-0.4,0.5);

q1 = p560.ikine(T1);

T2 = transl(0.5,0.4,0.1);

q2 = p560.ikine(T2);

steps = 50;

% 1 for quentic polynomial and anything else for trapezoidal
traj_profile = 2;

if traj_profile == 1
    qMatrix = jtraj(q1,q2,steps);
    figure(1);
    p560.plot(qMatrix,'trail','r-');
    velocity = zeros(steps,6);
    acceleration = zeros(steps,6);
    for i = 2:steps
        velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);
        acceleration(i,:) = velocity(i,:) - velocity(i-1,:);
    end
    for i = 1:6
        figure(2)
        subplot(3,2,i)
        plot(qMatrix(:,i),'k','LineWidth',1)
        title(['Joint ', num2str(i)])
        xlabel('Step')
        ylabel('Joint Angle (rad)')
        refline(0,p560.qlim(i,1)) % Reference line on the lower joint limit for joint i
        refline(0,p560.qlim(i,2)) % Reference line on the upper joint limit for joint i
     
        figure(3)
        subplot(3,2,i)
        plot(velocity(:,i),'k','LineWidth',1)
        title(['Joint ', num2str(i)])
        xlabel('Step')
        ylabel('Joint Velocity')
      
        figure(4)
        subplot(3,2,i)
        plot(acceleration(:,i),'k','LineWidth',1)
        title(['Joint ', num2str(i)])
        xlabel('Step')
        ylabel('Joint Acceleration')
      end

else
    s = lspb(0,1,steps);
    qMatrix = nan(steps,6);
    for i = 1:steps
        qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
    end
    figure(1);
    p560.plot(qMatrix,'trail','r-');
    velocity = zeros(steps,6);
    acceleration = zeros(steps,6);
    for i = 2:steps
        velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);
        acceleration(i,:) = velocity(i,:) - velocity(i-1,:);
    end
    for i = 1:6
        figure(2)
        subplot(3,2,i)
        plot(qMatrix(:,i),'k','LineWidth',1)
        title(['Joint ', num2str(i)])
        xlabel('Step')
        ylabel('Joint Angle (rad)')
        refline(0,p560.qlim(i,1)) % Reference line on the lower joint limit for joint i
        refline(0,p560.qlim(i,2)) % Reference line on the upper joint limit for joint i
     
        figure(3)
        subplot(3,2,i)
        plot(velocity(:,i),'k','LineWidth',1)
        title(['Joint ', num2str(i)])
        xlabel('Step')
        ylabel('Joint Velocity')
      
        figure(4)
        subplot(3,2,i)
        plot(acceleration(:,i),'k','LineWidth',1)
        title(['Joint ', num2str(i)])
        xlabel('Step')
        ylabel('Joint Acceleration')
      end
end
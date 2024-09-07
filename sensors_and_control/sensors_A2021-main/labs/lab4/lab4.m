%% Question 1
bag = rosbag('tutorial4.bag');

laser_bag = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/scan');

laser_scan = readMessages(laser_bag);

laser_scan{1};

cartesian = readCartesian(laser_scan{1,:});

plot(cartesian(:,1), cartesian(:,2));

%% Question 2
laser_data = [1.4142  1.1547  1.0353  1.0000  1.0353  1.1547  1.4142];
sensor_min_angle = deg2rad(-45);
sensor_max_angle = deg2rad(45);
sensor_range = sensor_max_angle - sensor_min_angle;
angle_increment = sensor_range/(size(laser_data,2)-1);
angle_data = sensor_min_angle:angle_increment:sensor_max_angle;
cartesian_data = zeros(size(laser_data,2), 2);

for i = 1:1:size(laser_data,2)
    cartesian_data(i,:) = [laser_data(i) * cos(angle_data(i)), laser_data(i) * sin(angle_data(i))];
end
figure;
axis equal;
plot(cartesian_data(:,1), cartesian_data(:,2), 'o');

%% Question 3
laser_data = readmatrix('intel_LASER_.txt');
laser_1 = laser_data(1,:);
laser_32 = laser_data(32,:);
angle_data = linspace(-90,90,181);
[laser_1_x, laser_1_y] = pol2cart(angle_data, laser_1);
[laser_32_x, laser_32_y] = pol2cart(angle_data, laser_32);
laser_1_pts = [laser_1_x', laser_32_y', zeros(size(laser_data,2),1)];
laser_32_pts = [laser_32_x', laser_32_y', zeros(size(laser_data,2),1)];
pt_cloud_1 = pointCloud(laser_1_pts, 'Color', [zeros(181,1), ones(181,1), zeros(181,1)]);
pt_cloud_32 = pointCloud(laser_32_pts, 'Color', [ones(181,1), zeros(181,1), zeros(181,1)]);

figure;
hold on;
pcshow(pt_cloud_1);
pcshow(pt_cloud_32);

tform = pcregistericp(pt_cloud_1,pt_cloud_32)

% sensor_min_angle = deg2rad(-90);
% sensor_max_angle = deg2rad(90);
% sensor_range = sensor_max_angle - sensor_min_angle;
% angle_increment = sensor_range/(size(laser_data,2)-1);
% angle_data = sensor_min_angle:angle_increment:sensor_max_angle;
% cartesian_data = zeros(size(laser_data,2)*size(laser_data,1), 2);
% point_counter = 1;
% for i = 1:1:size(laser_data,1)
% for i = 1:1:size(laser_data,2)
%     cartesian_data(point_counter,:) = [laser_data(i) * cos(angle_data(i)), laser_data(i) * sin(angle_data(i))];
%     point_counter = point_counter + 1;
% end
% end
% cartesian_data = [cartesian_data zeros(size(cartesian_data,1),1)];
% pt_cloud = pointCloud(cartesian_data);

% figure;
% axis equal;
% plot(cartesian_data(:,1), cartesian_data(:,2), 'o');
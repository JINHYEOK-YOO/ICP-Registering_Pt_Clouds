% Your implementation should run by executing this m-file ("run LW1.m"), 
% but feel free to create additional files for your own functions
% Make sure it runs without errors after unziping
% This file is for guiding the work. You are free to make changes that suit
% you.

% Fill out the information below

% Group members: Jinhyeok Yoo, Romain Husson, Elodie Charitat
% Additional tasks completed (task numbers): 1, 2, 3
%% Task 1: Apply transformation on point and visualize [mandatory]

% Step 1
% create point cloud 
Points = pointCloud([0 0 3; 5 0 5; 2.5 2.5 0]);

% Step 2
% create transformation
R_x = [1 0 0; 0 cos(20) -sin(20); 0 sin(20) cos(20)];
R_y = [cos(50) 0 sin(50); 0 1 0; -sin(50) 0 cos(50)];
R_z = [cos(40) -sin(40) 0; sin(40) cos(40) 0; 0 0 1];
R = R_z * R_y * R_x;
t = [0 2 3];

% Step 3
PointsMoved = pointCloud(rigidTransform(Points.Location,R,t));

% Step 4
% Visualize the point cloud piar
f1=figure('Name','Task 1');
figure(f1);
pcshowpair(Points,PointsMoved, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',200)
offset=.2;
hold on, text(Points.Location(:,1)+offset,Points.Location(:,2)+offset ,Points.Location(:,3)+offset,num2str([1:Points.Count]'))
hold on, text(PointsMoved.Location(:,1)+offset, PointsMoved.Location(:,2)+offset ,PointsMoved.Location(:,3)+offset,num2str([1:PointsMoved.Count]'))
hold off

title('Original and Transformed points')
xlabel('X (unit)')
ylabel('Y (unit)')
zlabel('Z (unit)')
%% Task 2: Estimate homogenous transformation [rotation and translation] between original and transformed point cloud of task 1 [mandatory]
% First run task 1 to get data for this task 2

%data from task 1
pts=Points.Location; %reference points
ptsMoved=PointsMoved.Location; % Points to align to reference

% Estimate the transformation [R,t]
[R,t] = estimateRT_pt2pt(pts,ptsMoved);

% Transform 
ptsAlligned = pointCloud(rigidTransform(ptsMoved,R,t));

% Visualize
f2=figure('Name','Task 2');
figure(f2);
pcshowpair(Points,ptsAlligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',200)
hold on, text(Points.Location(:,1),Points.Location(:,2) ,Points.Location(:,3),num2str([1:Points.Count]'))
hold on, text(ptsAlligned.Location(:,1), ptsAlligned.Location(:,2) ,ptsAlligned.Location(:,3),num2str([1:ptsAlligned.Count]'))
title('tranformed and merged Point clouds')
hold off

% Find the error (RMSE)
err = Points.Location - ptsAlligned.Location;
err = err .* err;
err = sum(err(:));
rmse = sqrt(err/ptsAlligned.Count);
%% Task 3: Create a function to iteratively allign bunny ptsMoved point cloud to the reference [mandatory]

%load dataset
load('bunny.mat')

% downsample
gridStep = 0.01;
bunny = pcdownsample(bunny,'gridAverage',gridStep);
bunnyMoved = pcdownsample(bunnyMoved,'gridAverage',gridStep);

% extract points
pts = bunny.Location; %reference points
ptsMoved = bunnyMoved.Location; %Points to align to reference
bunnyAlligned = pointCloud(ptsMoved);

% Set parameters
tolerance=[0.001, 0.001];  % can be changed
R = eye(3);
t = zeros(1,3);
maxIter = 100;
count = 0;
f3=figure('Name','Task 3');

% Iteration
for iter = 1 : maxIter
    %Perform ICP
    [bunny_estR,bunny_estt]=ICP(pts,bunnyAlligned.Location);

    % Visualize Seperately
    bunnyAlligned=pointCloud(rigidTransform(bunnyAlligned.Location,bunny_estR,bunny_estt));
    pcshowpair(bunny,bunnyAlligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'down','MarkerSize',100)
    title(['ICP Iteration: ',num2str(iter)])
    figure(f3); drawnow;
    
    % Integrate each rotation and translation.
    R = R * bunny_estR;
    t = t + bunny_estt;
    
    % Calculate the change in rotation and translation
    axang_estR = rotm2axang(bunny_estR);
    vec_estR = axang_estR(1,4) * axang_estR(1,1:3); 
    delta_R = norm(vec_estR);
    delta_t = norm(bunny_estt);
    
    % Check stop criterion
    if delta_R < tolerance(1,1) && delta_t < tolerance(1,2)
        count = count + 1;
    else
        count = 0;
    end
    if count >= 3
        R, t
        fprintf('Solution reached in %d iterations\n',iter);
        break;
    end
    if iter == 100
        fprintf('Max iterations reached');
    end
end
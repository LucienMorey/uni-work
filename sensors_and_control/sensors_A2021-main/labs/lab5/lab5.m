%% question 1
close all;
clear;
%1.1
I = imread('checkerboard.jpg');
I = rgb2gray(I);
cornerPoints = detectHarrisFeatures(I);
figure;
imshow(I);
hold on;
plot(cornerPoints);

% 1.2
harris_I = imread('harris_corners_example.jpg');
gs_harris_I = rgb2gray(harris_I);
corner_points_harris = detectHarrisFeatures(gs_harris_I, 'minQuality', 0.2);
figure;
imshow(gs_harris_I);
hold on;
plot(corner_points_harris);
%% Question 2
close all;
clear;
roof_1_I = imread('roofs1.jpg');
roof_2_I = imread('roofs2.jpg');
roof1_gs = rgb2gray(roof_1_I);
roof2_gs = rgb2gray(roof_2_I);
%2.1;
points1 = detectSURFFeatures(roof1_gs);
points2 = detectSURFFeatures(roof1_gs);

[features1, validPoints1] = extractFeatures(roof1_gs, points1);
[features2, validPoints2] = extractFeatures(roof2_gs, points2);
indexPairs = matchFeatures(features1, features2);
matchedPoints1 = validPoints1(indexPairs(:,1));  
matchedPoints2 = validPoints2(indexPairs(:,2));
figure;
showMatchedFeatures(roof1_gs, roof2_gs, matchedPoints1, matchedPoints2, 'montage');

%2.2;
points1 = detectORBFeatures(roof1_gs);
points2 = detectORBFeatures(roof1_gs);

[features1, validPoints1] = extractFeatures(roof1_gs, points1);
[features2, validPoints2] = extractFeatures(roof2_gs, points2);
indexPairs = matchFeatures(features1, features2);
matchedPoints1 = validPoints1(indexPairs(:,1));  
matchedPoints2 = validPoints2(indexPairs(:,2));
figure;
showMatchedFeatures(roof1_gs, roof2_gs, matchedPoints1, matchedPoints2, 'montage');
%% Question 3
close all;
clear;
original = rgb2gray(imread('kfc1.jpg'));
rotated = rgb2gray(imread('kfc2.jpg'));

ptsOriginal = detectSURFFeatures(original);
ptsRotated = detectSURFFeatures(rotated);

[featuresOriginal, validPtsOriginal] = extractFeatures(original, ptsOriginal);
[featuresRotated, validPtsRotated] = extractFeatures(rotated, ptsRotated);
indexPairs = matchFeatures(featuresOriginal, featuresRotated);
matchedPtsOriginal = validPtsOriginal(indexPairs(:,1));  
matchedPtsRotated = validPtsRotated(indexPairs(:,2));

figure;
showMatchedFeatures(original, rotated, matchedPtsOriginal, matchedPtsRotated,'montage');

[tform, inlierRotated, inlierOriginal] = estimateGeometricTransform(matchedPtsRotated,matchedPtsOriginal, 'similarity');

figure;
showMatchedFeatures(original,rotated,inlierOriginal,inlierRotated);
title('Matching Points (inliers only)');
legend('ptsOriginal', 'ptsDistortred');

Tinv = tform.invert.T;
ss = Tinv(2,1);
sc = Tinv(1,1);
scaleRecovered = sqrt(ss^2 +sc+2);
thetaRecovered = atan2(ss, sc) * 180/pi;

outputView = imref2d(size(original));
recoved = imwarp(rotated, tform, 'OutputView', outputView);
figure;
imshowpair(original,recoved, 'montage');
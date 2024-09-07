% This function takes a gray scale image ('gsimg') and a kernek ('k')
% as input and performs the convolution operation to produce the output

% You can visually compare the output of this function with the output of the
% imfilter function
img = imread('SydneyOperaHouse.jpg');
gsimg = rgb2gray(img);
k = [0 0 0; 0 1 0; 0 0 0;];
edgeImg1 = imfilter(gsimg, k);
output = applyConvolution2GrayscaleImgFunc(gsimg, k);

figure
hold on
imshow(edgeImg1)
title('imfilter')

figure
hold on
title('Our function')
imshow(output)

function [output] = applyConvolution2GrayscaleImgFunc(gsimg, k)
% First, evaluate the size of the inputs image and kernel
    [n,m] = size(gsimg);
    [nk,mk] = size(k);

% Create an empty matrix for output.
    output = zeros(n,m);
    
% Iterate over each pixel of the output image and set the value
% Notice that depending on the size of the kernel, we have to ignore pixels
% from top, bottom, left and right
    for i = ceil(nk/2) : n - ceil(nk/2)
        for j = ceil(mk/2) : m - ceil(mk/2)
%           region is the area of the grayscale image that we need to apply
%           convolution to the (i,j)th pixel 
            region = gsimg(i - floor(nk/2) : i + floor(nk/2), j - floor(mk/2) : j+ floor(mk/2));
%           This convertion is needed for the .* operation
            region = double(region);
            temp = sum(sum(region.*k));
            output(i,j) = temp;
        end
    end
%   The output should be a grayscale image. So we need to convert the
%   double matrix into a uint8 matrix
    output = uint8(output);
end

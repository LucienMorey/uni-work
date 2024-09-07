OH = imread('SydneyOperaHouse.jpg');
gsOH = rgb2gray(OH);
sharpenK = [0 -1 0; -1 5 -1; 0 -1 0;];
edgeImg1 = imfilter(gsOH, sharpenK);
output = ConvoluteLucien(gsOH, sharpenK);


figure
hold on
imshow(gsOH)
title('imfilter')

figure
hold on
title('Our function')
imshow(output)

HB = imread('Sydney_Harbour_Bridge_from_Circular_Quay.jpg');
gsHB = rgb2gray(HB);
edgeDetectK = [-1 -1 -1; -1 8 -1; -1 -1 -1;];
edgeImg2 = imfilter(gsHB, edgeDetectK);
output2 = ConvoluteLucien(gsHB, edgeDetectK);


figure
hold on
imshow(gsHB)
title('imilter')

figure
hold on
title('Our')
imshow(output2)
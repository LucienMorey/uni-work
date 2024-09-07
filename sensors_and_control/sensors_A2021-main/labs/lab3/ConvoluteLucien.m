function convImage = ConvoluteLucien(image, kernal)
    % determine size of image
    [imageRows, imageCols] = size(image);
    % determine kernal size
    [kernalRows, kernalCols] = size(kernal);
    % create output image  of the same size as input
    convImage = zeros(imageRows, imageCols);
    % iterate through all convoluteable cells
    for i = ceil(kernalRows/2):1:imageRows-floor(kernalRows/2)
        for j = ceil(kernalCols/2):1:imageCols-floor(kernalCols/2)
            % get local region around pixel
            region = image((i-floor(kernalRows/2)):(i+floor(kernalRows/2)), (j-floor(kernalCols/2)):(j+floor(kernalCols/2)));
            region = double(region);
            newPixelVal = sum(sum(region.*kernal));
            convImage(i,j) = newPixelVal;
        end
    end

    convImage = uint8(convImage);


    % loop through convoluteable image
        % get surrounding temp sample
        % elementwise multiplication
        % obtain the sum and set the element to equal it

    % limit the image values
end
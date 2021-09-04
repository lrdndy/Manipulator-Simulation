function [color, x, y]=image_processing(vid)
    %the code is modified based on the CHANDRAKANT SUPARE's green and red
    %detection matlab example
    data = getsnapshot(vid);
    RI=imref2d(size(data));% image axis reference to 2D world
    RI.XWorldLimits=[1 640];%set x axis from 1 to 640
    RI.YWorldLimits=[1 480];% set y axis from 1 to 48;
    figure(2)
    imshow(data,RI);
    %subtract the green component from the grayscale image.
    diff_im_green = imsubtract(data(:,:,2), rgb2gray(data));% 2 is for green
    %median filter to filter out noise
    diff_im_green = medfilt2(diff_im_green, [3 3]);
    % Convert the resulting grayscale image into a binary image.
    diff_im_green = im2bw(diff_im_green,0.05); % 0.05 is for green pixel threshold  
    % Remove all those pixels less than 300px
    diff_im_green = bwareaopen(diff_im_green,300);    
    % Label all the connected components in the image.
    bw_g = bwlabel(diff_im_green, 8);    
    % image blob analysis.
    % set of properties for each labeled region.
    stats_g = regionprops(bw_g, 'BoundingBox', 'Centroid');
    %RED OBJECT DETECTION
    %subtract the red component from the grayscale image.
    diff_im_red = imsubtract(data(:,:,1), rgb2gray(data));%1 is for red pixel 3 for blue
    %median filter to filter out noise
    diff_im_red = medfilt2(diff_im_red, [3 3]);
    %Convert the resulting grayscale image into a binary image.
    diff_im_red = im2bw(diff_im_red,0.18);    
    %Remove all those pixels less than 300px
    diff_im_red = bwareaopen(diff_im_red,300);    
    %Label all the connected components in the image.
    bw_r = bwlabel(diff_im_red, 8);    
    %image blob analysis.
    %set of properties for each labeled region.
    stats_r = regionprops(bw_r, 'BoundingBox', 'Centroid');
    g1 = length(stats_g);
    r1 = length(stats_r);
    if g1 > r1
        color = 0;
        for object_g = 1:length(stats_g)
        bb_g = stats_g(object_g).BoundingBox;
        bc_g = stats_g(object_g).Centroid;
        %rectangle('Position',bb_g,'EdgeColor','g','LineWidth',2)% plotting rectangle
        %plot(bc_g(1),bc_g(2), '-m+') %plotting green box centroid
        %a_g=text(bc_g(1)+15,bc_g(2), strcat('X: ', num2str(round(bc_g(1))), '    Y: ', num2str(round(bc_g(2)))));
        %set(a_g, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 10, 'Color', 'green');
        x = bc_g(1); y = bc_g(2);
        end
    elseif r1 > g1
        color = 1;
        for object_r = 1:length(stats_r)
        bb_r = stats_r(object_r).BoundingBox;
        bc_r = stats_r(object_r).Centroid;
        %rectangle('Position',bb_r,'EdgeColor','r','LineWidth',2)% plotting rectangle
        %plot(bc_r(1),bc_r(2), '-m+') %plotting red box centroid
        %a_r=text(bc_r(1)+15,bc_r(2), strcat('X: ', num2str(round(bc_r(1))), '    Y: ', num2str(round(bc_r(2)))));
        %set(a_r, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 10, 'Color', 'red');
        x = bc_r(1); y = bc_r(2);
        end
    else
        x = false; y=false; color = 2;
    end
end
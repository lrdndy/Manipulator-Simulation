function []=colorshow(input,objects)
    %this function is used to show objects with colors
    %can be modified to different color by changing line 6 and line 9
    [~,patchObj] = show(objects);
    if input == 1
        patchObj.FaceColor = [1 0 1];
    end
    if input == 0
        patchObj.FaceColor = [0.2 1 0.3];
    end
end
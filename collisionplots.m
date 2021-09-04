function []=collisionplots(rob,robdef,q,colLink,conveybelt,storage,storage2,n)
    %this function is used to plot the collisions plots between arm and
    %obstacles
    collisionPlot(rob,robdef,q,colLink,conveybelt,n);
    hold on;
    collisionPlot(rob,robdef,q,colLink,storage,n);
    hold on;
    collisionPlot(rob,robdef,q,colLink,storage2,n);
    hold on;
    figure(1)
end
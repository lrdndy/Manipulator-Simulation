%This is the main file for this project with the use of camera input
close all;clear all;
%
vid = videoinput('winvideo',1,'YUY2_640x480');
set(vid, 'FramesPerTrigger', Inf);
set(vid, 'ReturnedColorspace', 'rgb');
vid.FrameGrabInterval = 3;
start(vid);

k =1;%the slots for red box
for i = 1:4 
    for j = 1:4
        redx(k) = -1.8+(i-1)*0.2;
        redy(k) = 1-(j)*0.2;
        redz(k) = 1;
        k=k+1;
    end
end

k =1; %the slots for green box
for i = 1:4
    for j = 1:4
        greeny(k) = 1.8-(i-1)*0.2;
        greenx(k) = -1+(j)*0.2;
        greenz(k) = 1;
        k=k+1;
    end
end
ind = 0;
redind = 0;
greenind = 0;
while(vid.FramesAcquired<=600)
    [out, x, y] = image_processing(vid);
    if out == 2 %didn't detect red or green
        continue;
    elseif out == 1% if it is red garbage
        input = 1;
    else
        input = 0;% if it is green garbage 
    end
    xi = 0.7*x/639+0.3-0.7/639;yi = 0.7*y/479-1.4-0.7/479;%inital position of garbage on convey belt
    figure(1)
    ind = ind+1;
    if input == 1
        redind = redind + 1;
        pf = [redx(redind); redy(redind); redz(redind)];%pre-arranged slots on box
        Rp = -[1 0 0; 0 1 0; 0 0 -1];
    else
        greenind = greenind + 1;
        pf = [greenx(greenind); greeny(greenind); greenz(greenind)];
        Rp = [-1 0 0; 0 1 0; 0 0 1];
    end

abb6640def;n=size(abb6640.H,2);robdef=abb6640;radius=.05;
[rob,colLink]=collisionBody(robdef,radius);
Rca = [1 0 0; 0 1 0; 0 0 -1];%
% Create convey belt
conveybelt = collisionBox(3,1,0.5);
conveybelt.Pose = trvec2tform([-0.5 -1 0.15-0.001]);%


objects = collisionBox(0.2,0.2,0.2); %
existobjects = collisionBox(0.2,0.2,0.2); %
objects.Pose = trvec2tform([xi yi 0.5]); %
[~,patchObj] = show(conveybelt);%
patchObj.FaceColor = [0 1 1];
xlim([-3.5 2.5])
ylim([-2 3])
zlim([-3 3])
hold on
%view(90,0)

for t = 1:ind-1 %show the previous dropped garabages
    if ind == 1
        break
    end
    existobjects.Pose = trvec2tform([posecollectorx(t) posecollectory(t) posecollectorz(t)]);
    colorshow(posecollectorcolor(t),existobjects);
    hold on;
end


q0 = [0;0;0;0;0;0];
%show(rob,q0,'Collisions','on','Visuals','off');%
Tinitial = fwdkinrec(1,eye(4),q0,abb6640)%
n = 5;%3*n
targetpose = objects.Pose*[eye(3) [-0.05*3*n;0;0];[0 0 0 1]];%targeted T
ptarget = targetpose(1:3,4)%targed P
R = Rca% define targeted rotation
qf = solveq(R,ptarget)% solve q using inverse kinematics
TF=fwdkinrec(1,eye(4),qf(:,1),abb6640);% move to the target position using forward kinematics
midpoint1 = [0;-0.5;TF(3,4)+1.5]; midpoint2 = [TF(1,4)-0.5;TF(2,4)+0.5;TF(3,4)+1]; %define intermediate points
qm1 = solveq(R,midpoint1);qm2 = solveq(R,midpoint2); 
sph1 = collisionSphere(0.05);sph1.Pose = trvec2tform(transpose(midpoint1));%show intermediate points
sph2 = collisionSphere(0.05);sph2.Pose = trvec2tform(transpose(midpoint2));
[~,sphob] = show(sph1);sphob.FaceColor = [0 1 1];hold on;[~,sphob2] = show(sph2);sphob2.FaceColor = [0 1 1];
hold on;
storage = collisionBox(1,1,0.6);
storage.Pose = trvec2tform([-1.5 0.5 0.4-0.0005]);
[~,patchObj] = show(storage);%
patchObj.FaceColor = [1 0 1];
hold on;
storage2 = collisionBox(1,1,0.6);
storage2.Pose = trvec2tform([-0.5 1.5 0.4-0.0005]);
[~,patchObj] = show(storage2);%
patchObj.FaceColor = [0.2 1 0.3];

for i = 1:3*n %
    objects.Pose = trvec2tform([xi-0.05*i yi 0.5]);%
    colorshow(input,objects);
    hold on;
    if i <= n
        qq = q0+i*(qm1-q0)/n;
    elseif i <= 2*n  
        qq = qm1+(i-n)*(qm2-qm1)/(n);
    else
        qq = qm2+(i-2*n)*(qf-qm2)/(n);
    end
    show(rob,qq,'Collisions','on','Visuals','off');
    %collisionplots(rob,robdef,qq,colLink,conveybelt,storage,storage2,98);
    getframe;%
end
hold on;


midpoint3 = [-1.5;-0.5;pf(3)+0.5];% intermeidiate point 3
qm3 = solveq(Rp,midpoint3);
qpf = solveq(Rp,pf);% solve q using inverse kinematics
m = 3; % 2*m step to drop of the item 
for i = 1:2*m
    if i<= m
        qnow = qq + i*(qm3 - qq)/m;
        T2 = fwdkinrec(1,eye(4,4),qnow,abb6640);  
    else
        qnow = qm3 + (i-m)*(qpf - qm3)/m;
        T2 = fwdkinrec(1,eye(4,4),qnow,abb6640);
    end
    objects.Pose = trvec2tform([T2(1,4) T2(2,4) T2(3,4)]);
    colorshow(input,objects);
    hold on;
    show(rob,qnow,'Collisions','on','Visuals','off');
    drawnow;
end
hold on;


objects.Pose = trvec2tform([T2(1,4) T2(2,4) T2(3,4)-0.2]);
colorshow(input,objects);
hold on;
midpoint4 = [0;1;1.5];% intermediate point 4
sph4 = collisionSphere(0.05);sph4.Pose = trvec2tform(transpose(midpoint4));
[~,sphob4] = show(sph4);sphob4.FaceColor = [0 0 1];hold on;%show points
qm4 = solveq(-R,midpoint4);
qhf = q0;
m = 3; %2*m step to go back to inital pose 
for i = 1:2*m
    if i<= m
        qh = qnow + i*(qm4 - qnow)/m;
        T2 = fwdkinrec(1,eye(4,4),qh,abb6640);  
    else
        qh = qm3 + (i-m)*(qhf - qm3)/m;
        T2 = fwdkinrec(1,eye(4,4),qh,abb6640);
    end
    show(rob,qh,'Collisions','on','Visuals','off');
    drawnow;
end
hold off;


%record previous garbage 
posecollectorx(ind) = objects.Pose(1,4);
posecollectory(ind) = objects.Pose(2,4);
posecollectorz(ind) = objects.Pose(3,4);
posecollectorcolor(ind) = input;
end
delete(vid);
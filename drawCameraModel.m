function [opticalCenter,imagePlanePoint] = drawCameraModel(showAxis,scale,transformationMatrix,h_ax)
%绘制camera的3D模型
scale = scale/100; %外面单位是m，这个函数单位是cm

%喇叭口的camera窗口
hold on;
windowSize = [720;1080]/1080*scale;
cameraWindow_p1 = [-1;1;0].*[windowSize;0]; %top left
cameraWindow_p2 = [1;1;0].*[windowSize;0]; %top right
cameraWindow_p3 = [1;-1;0].*[windowSize;0]; %bottom right
cameraWindow_p4 = [-1;-1;0].*[windowSize;0]; %bottom left

cameraWindow = [cameraWindow_p1,cameraWindow_p2,cameraWindow_p3,cameraWindow_p4,cameraWindow_p1];
cameraWindow = transformationMatrix(1:3,1:3)*cameraWindow + transformationMatrix(1:3,4)* ones(1,size(cameraWindow,2));
%plot3(h_ax, cameraWindow(1,:),cameraWindow(2,:),cameraWindow(3,:), 'b-')
%fill3(cameraWindow(1,:),cameraWindow(2,:),cameraWindow(3,:),[1,1,1],'FaceAlpha',0.2,'FaceColor',[0.6350 0.0780 0.1840])
fill3(cameraWindow(1,:),cameraWindow(2,:),cameraWindow(3,:),[1,1,1],'FaceAlpha',0.2,'FaceColor',[0.5 0.5 0.5],'LineStyle','none')

%长方体基座
depth1=0.5*scale; %相机窗口到基座近端边距离
depth2=1.5*scale; %基座长度
cubeSize=0.35*scale; %基座面边长
cube_proximal_p1 = [-1;1;1].*[cubeSize;cubeSize;-depth1];
cube_proximal_p2 = [1;1;1].*[cubeSize;cubeSize;-depth1];
cube_proximal_p3 = [1;-1;1].*[cubeSize;cubeSize;-depth1];
cube_proximal_p4 = [-1;-1;1].*[cubeSize;cubeSize;-depth1];

proximalEdge = [cube_proximal_p1,cube_proximal_p2,cube_proximal_p3,cube_proximal_p4,cube_proximal_p1];
proximalEdge = transformationMatrix(1:3,1:3)*proximalEdge + transformationMatrix(1:3,4)* ones(1,size(proximalEdge,2));
%plot3(h_ax, proximalEdge(1,:),proximalEdge(2,:),proximalEdge(3,:), 'b-')


cube_distal_p1 = [-1;1;1].*[cubeSize;cubeSize;-depth2];
cube_distal_p2 = [1;1;1].*[cubeSize;cubeSize;-depth2];
cube_distal_p3 = [1;-1;1].*[cubeSize;cubeSize;-depth2];
cube_distal_p4 = [-1;-1;1].*[cubeSize;cubeSize;-depth2];

distalEdge = [cube_distal_p1,cube_distal_p2,cube_distal_p3,cube_distal_p4,cube_distal_p1];
distalEdge = transformationMatrix(1:3,1:3)*distalEdge + transformationMatrix(1:3,4)* ones(1,size(distalEdge,2));
%plot3(h_ax, distalEdge(1,:),distalEdge(2,:),distalEdge(3,:), 'b-')

%画侧边
sideEdge1 = [cameraWindow_p1,cube_proximal_p1,cube_distal_p1];
sideEdge1 = transformationMatrix(1:3,1:3)*sideEdge1 + transformationMatrix(1:3,4)* ones(1,size(sideEdge1,2));
%plot3(h_ax, sideEdge1(1,:),sideEdge1(2,:),sideEdge1(3,:), 'b-')

sideEdge2 = [cameraWindow_p2,cube_proximal_p2,cube_distal_p2];
sideEdge2 = transformationMatrix(1:3,1:3)*sideEdge2 + transformationMatrix(1:3,4)* ones(1,size(sideEdge2,2));
%plot3(h_ax, sideEdge2(1,:),sideEdge2(2,:),sideEdge2(3,:), 'b-')

sideEdge3 = [cameraWindow_p3,cube_proximal_p3,cube_distal_p3];
sideEdge3 = transformationMatrix(1:3,1:3)*sideEdge3 + transformationMatrix(1:3,4)* ones(1,size(sideEdge3,2));
%plot3(h_ax, sideEdge3(1,:),sideEdge3(2,:),sideEdge3(3,:), 'b-')

sideEdge4 = [cameraWindow_p4,cube_proximal_p4,cube_distal_p4];
sideEdge4 = transformationMatrix(1:3,1:3)*sideEdge4 + transformationMatrix(1:3,4)* ones(1,size(sideEdge4,2));
%plot3(h_ax, sideEdge4(1,:),sideEdge4(2,:),sideEdge4(3,:), 'b-')

%画坐标系
axisLength = scale*0.5;
opticalCenter=[0;0;cube_proximal_p1(3)*1/(1-(cube_proximal_p1(2)/cameraWindow_p1(2)))];
xEnd = [axisLength;0;0];
yEnd = [0;axisLength;0];
zEnd = [0;0;axisLength];
frameArrowPoint = [opticalCenter,xEnd,yEnd,zEnd];
frameArrowPoint = transformationMatrix(1:3,1:3)*frameArrowPoint + transformationMatrix(1:3,4)* [1,zeros(1,size(frameArrowPoint,2)-1)];
if showAxis>0
    
    %plot3(h_ax,frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),'ko');
    
    %quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,2),frameArrowPoint(2,2),frameArrowPoint(3,2),'r-','LineWidth',2);
    %quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,3),frameArrowPoint(2,3),frameArrowPoint(3,3),'g-','LineWidth',2);
    %quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,4),frameArrowPoint(2,4),frameArrowPoint(3,4),'b-','LineWidth',2);
    
    quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,2),frameArrowPoint(2,2),frameArrowPoint(3,2),'r-');
    quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,3),frameArrowPoint(2,3),frameArrowPoint(3,3),'g-');
    quiver3(h_ax, frameArrowPoint(1,1),frameArrowPoint(2,1),frameArrowPoint(3,1),frameArrowPoint(1,4),frameArrowPoint(2,4),frameArrowPoint(3,4),'b-');
    
end
opticalCenter = frameArrowPoint(:,1);
imagePlanePoint = cameraWindow(:,1:4);

axis equal;
hold off;
end


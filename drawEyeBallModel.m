function [eyeCenter,pupilRadius, pupilCenter, transformedPupilPoints, transformedIrisPoints, transformedCorneaCenter, corneaSphereRadius] = drawEyeBallModel(scale,pupilEdgeSampleNum, pupilDilationRatio ,transformationMatrix,h_ax)
% sphere: corneal, 折射率1.376，外半径7.7，内半径6.8，角膜在瞳孔法矢方向厚度0.5
% 瞳孔中心沿瞳孔法矢方向，离corneal内侧距离3.3
% 瞳孔内部玻璃体 vitreous humour折射率1.336
% 眼球直球直接24，沿着瞳孔法矢，4个线段依次为0.5,3.3,3.5,16.
% 虹膜离角膜远端距离3.6， “optics of the human eye， p22”

scale = scale/1000; %外部以m为单位，这个函数是以mm为单位。

hold on;
%将眼球旋转中心设置在原点。默认模型的光轴为z轴。
eyeBallRotationCenter=[0;0;0];  
eyeBallRadius = 10.9*scale;   %24.4-13.5 https://application.wiley-vch.de/books/sample/3527403809_c01.pdf page5

% sphere: corneal, 折射率1.376，外半径7.7，内半径6.8，角膜在瞳孔法矢方向厚度0.5
%第一个元素是角膜anterior球面外表面，中心在原点； 第二个元素是角膜posterior球形内表面，中心往z轴正向偏移
corneaSphereCenter = [0,0;0,0;5.8,6.2]*scale;
corneaSphereRadius = [7.7,6.8]*scale;
sphereSelectRatio=[3.85/7.7+0.2, 3.25/6.5+0.1];

pupilRadius = 2.5*scale*pupilDilationRatio;   %直径2.0-8.0 “optics of the human eye， p24”, pupilDilationRatio为瞳孔放缩比
irisRadius = 5*scale; 
pupilCenter=[0;0;(corneaSphereCenter(3,1)+corneaSphereRadius(1)-3.6*scale)];  %入口距离3.05，中间虹膜距离3.6，出口距离3.67

color=['r','g'];
alpha=[0.1,0.2];
sphereSamples=100;
[x,y,z]=sphere(sphereSamples);  %单位圆上101个点，z轴方向101层

eyeCenter = zeros([3,3]);

% 画角膜内外表面
transformedCorneaCenter = zeros(size(corneaSphereCenter));
for i=1:2
    selectNum = round(sphereSelectRatio(i)*sphereSamples/2); %取小半个球
    tempX=corneaSphereRadius(i)*x((sphereSamples - selectNum):(sphereSamples+1),:) +corneaSphereCenter(1,i);
    tempY=corneaSphereRadius(i)*y((sphereSamples - selectNum):(sphereSamples+1),:) +corneaSphereCenter(2,i);
    tempZ=corneaSphereRadius(i)*z((sphereSamples - selectNum):(sphereSamples+1),:) +corneaSphereCenter(3,i);
    for j=1:size(tempX,1)
        for k=1:size(tempX,2)
            transformedVector = transformationMatrix(1:3,1:3)*[tempX(j,k);tempY(j,k);tempZ(j,k)] + transformationMatrix(1:3,4);
            tempX(j,k) = transformedVector(1);
            tempY(j,k) = transformedVector(2);
            tempZ(j,k) = transformedVector(3);
        end
    end
    surf(tempX, tempY, tempZ, 'Facecolor', color (i), 'Linestyle', 'none', 'FaceAlpha', alpha (i));
    tempCenter = transformationMatrix(1:3,1:3)*corneaSphereCenter(:,i) + transformationMatrix(1:3,4);
    transformedCorneaCenter(:,i)=tempCenter;
    plot3(tempCenter(1),tempCenter(2),tempCenter(3),'.','Color', color (i));
    eyeCenter(:,1+i)= tempCenter;
end

% 眼球
selectNum = round((1+sphereSelectRatio(1)-0.1)*sphereSamples/2); %取大半个球
tempX = eyeBallRadius*x(1:selectNum,:) +eyeBallRotationCenter(1);
tempY = eyeBallRadius*y(1:selectNum,:) +eyeBallRotationCenter(2);
tempZ = eyeBallRadius*z(1:selectNum,:) +eyeBallRotationCenter(3);
for j=1:size(tempX,1)
    for k=1:size(tempX,2)
        transformedVector = transformationMatrix(1:3,1:3)*[tempX(j,k);tempY(j,k);tempZ(j,k)] + transformationMatrix(1:3,4);
        tempX(j,k) = transformedVector(1);
        tempY(j,k) = transformedVector(2);
        tempZ(j,k) = transformedVector(3);
    end
end
%surf(h_ax,tempX, tempY, tempZ, 'Facecolor', [0 0.4470 0.7410], 'Linestyle', 'none', 'FaceAlpha', alpha (1));
surf(h_ax,tempX, tempY, tempZ, 'Facecolor', [0.5 0.5 0.5], 'Linestyle', 'none', 'FaceAlpha', alpha (1));
eyeCenter(:,1) = transformationMatrix(1:3,1:3)*eyeBallRotationCenter + transformationMatrix(1:3,4);
plot3(h_ax,eyeCenter(1,1),eyeCenter(2,1),eyeCenter(3,1),'.','Color', 'k')


% 虹膜内外圆盘
t = linspace(0,2*pi,pupilEdgeSampleNum);
x = cos(t);
y = sin(t);
pupilPoints = [x*pupilRadius;y*pupilRadius;pupilCenter(3)*ones(size(t))];
irisPoints = [x*irisRadius;y*irisRadius;pupilCenter(3)*ones(size(t))];
transformedPupilPoints = transformationMatrix(1:3,1:3)*pupilPoints + transformationMatrix(1:3,4)* ones(1,size(pupilPoints,2));
transformedIrisPoints = transformationMatrix(1:3,1:3)*irisPoints + transformationMatrix(1:3,4)* ones(1,size(irisPoints,2));

plot3(h_ax,transformedPupilPoints(1,:),transformedPupilPoints(2,:),transformedPupilPoints(3,:),'k-','LineWidth',1)
%plot3(h_ax,transformedIrisPoints(1,:),transformedIrisPoints(2,:),transformedIrisPoints(3,:),'k-','LineWidth',0.5)
pupilCenter = transformationMatrix(1:3,1:3)*pupilCenter + transformationMatrix(1:3,4);
plot3(h_ax,pupilCenter(1),pupilCenter(2),pupilCenter(3),'k.')
hold off;
end


%% clear the env
clc;
clear;
figure(1);
ax =axes();

xlabel('x');
ylabel('y');
zlabel('z');

%% test
eyeTrans_t = [0;0;0];
pupilEdgeSampleNum=100;

% draw camera
drawCameraFrameAxis = 1;
cameraTrans_R = eul2rotm([0,0,-pi/2],'ZYX')*eul2rotm([-pi/2,0,0],'ZYX');
cameraTrans_t = [0;-0.045;0];
cameraTrans = [cameraTrans_R,cameraTrans_t;0,0,0,1];
[opticalCenter,imagePlanePoint]= drawCameraModel(drawCameraFrameAxis,1,cameraTrans,ax);

% draw eye ball
eyeTrans_R_init = zeros([3,3]);
eyeTrans_R_init(:,3) = (opticalCenter - eyeTrans_t)/norm(opticalCenter - eyeTrans_t);
eyeTrans_R_init(:,2) = cross(eyeTrans_R_init(:,3),cameraTrans_R(:,1))/norm(cross(eyeTrans_R_init(:,3),cameraTrans_R(:,1)));
eyeTrans_R_init(:,1) = cross(eyeTrans_R_init(:,2),eyeTrans_R_init(:,3))/norm(cross(eyeTrans_R_init(:,2),eyeTrans_R_init(:,3)));

currentEyeAlpha = -30*pi/180; 
currentEyeBeta = 45*pi/180;  
eyeTrans_R = eyeTrans_R_init*eul2rotm([currentEyeAlpha, 0,0],'ZYX')*eul2rotm([0,currentEyeBeta,0],'ZYX');
eyeTrans = [eyeTrans_R,eyeTrans_t;0,0,0,1];

[eyeBallCenter, pupilRadius, pupilCenter, transformedPupilPoints, transformedIrisPoints, transformedCorneaCenter, corneaSphereRadius]= drawEyeBallModel(1,pupilEdgeSampleNum,1,eyeTrans,ax);

% draw eye frame.
axisLength = 0.0075;
eyeFrame= eyeTrans_R*axisLength;
globalInitFrame = eyeTrans_R_init*axisLength*2;
hold on;

quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),globalInitFrame(1,1),globalInitFrame(2,1),globalInitFrame(3,1),'r-');
quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),globalInitFrame(1,2),globalInitFrame(2,2),globalInitFrame(3,2),'g-');
quiver3(ax, eyeTrans_t(1),eyeTrans_t(2),eyeTrans_t(3),globalInitFrame(1,3),globalInitFrame(2,3),globalInitFrame(3,3),'b-');


% optical axis of the eye
hold on;
gazeArrowLength = 0.02;
gazeArrowEndPoint = pupilCenter + gazeArrowLength*eyeFrame(:,3)/norm(eyeFrame(:,3));
eyeGazeDirection = [pupilCenter,gazeArrowEndPoint];
plot3(eyeGazeDirection(1,:),eyeGazeDirection(2,:),eyeGazeDirection(3,:),'k');
eyeGazeDirection_innerEye = [pupilCenter,eyeTrans_t];
plot3(eyeGazeDirection_innerEye(1,:),eyeGazeDirection_innerEye(2,:),eyeGazeDirection_innerEye(3,:),'k:','LineWidth',1);

% light raytracing of the pupil contour
[intersectionPointsOnInnerCorneaSphere, intersectionPointsOnOuterCorneaSphere, directIntersectionPointsOnOuterCorneaSphere] = ...
    refractedPupilEdgeRayTracing(opticalCenter,transformedPupilPoints,corneaSphereRadius(2),transformedCorneaCenter(:,2),...
    corneaSphereRadius(1),transformedCorneaCenter(:,1),1.376,1.4,1);

plot3(ax,intersectionPointsOnInnerCorneaSphere(1,:),intersectionPointsOnInnerCorneaSphere(2,:),intersectionPointsOnInnerCorneaSphere(3,:),'b-','LineWidth',1);
plot3(ax,intersectionPointsOnOuterCorneaSphere(1,:),intersectionPointsOnOuterCorneaSphere(2,:),intersectionPointsOnOuterCorneaSphere(3,:),'r-','LineWidth',1);

% light raytracing of on ECP plane spanned by the eye rotation center, the camera center, and the pupil center
eyeBallCenter2PupilCenter = (pupilCenter - eyeBallCenter(:,1))/norm(pupilCenter - eyeBallCenter(:,1));
eyeBallCenter2CameraOpticalCenter = (opticalCenter - eyeBallCenter(:,1)) / norm(opticalCenter - eyeBallCenter(:,1));
if norm(cross(eyeBallCenter2CameraOpticalCenter,eyeBallCenter2PupilCenter))<1e-10
    ECPplaneNormalDirection = eyeTrans_R(:,2);
else
    ECPplaneNormalDirection = cross(eyeBallCenter2CameraOpticalCenter,eyeBallCenter2PupilCenter)/norm(cross(eyeBallCenter2CameraOpticalCenter,eyeBallCenter2PupilCenter)); % eye center, camera center, pupil center 张成的平面ECP
end
pupilRadiusDirectionOnECP = cross(ECPplaneNormalDirection,eyeBallCenter2PupilCenter) / norm(cross(ECPplaneNormalDirection,eyeBallCenter2PupilCenter));

pupilEdgePointOnECPPlane_distalToCamera = pupilCenter + pupilRadiusDirectionOnECP * pupilRadius;
pupilEdgePointOnECPPlane_proximalToCamera = pupilCenter - pupilRadiusDirectionOnECP * pupilRadius;
pupilEdgePointOnECPAndPupilCenter = [pupilEdgePointOnECPPlane_distalToCamera,pupilEdgePointOnECPPlane_proximalToCamera, pupilCenter];

[ECPIntersectionPointsOnInnerCorneaSphere, ECPIntersectionPointsOnOuterCorneaSphere, ECPDirectIntersectionPointsOnOuterCorneaSphere] = ...
    refractedPupilEdgeRayTracing(opticalCenter,pupilEdgePointOnECPAndPupilCenter,corneaSphereRadius(2),transformedCorneaCenter(:,2),...
    corneaSphereRadius(1),transformedCorneaCenter(:,1),1.376,1.4,1);

refractedECPProjectionsOnCamera = linesPlaneIntersection(opticalCenter,ECPIntersectionPointsOnOuterCorneaSphere,imagePlanePoint,ax,0);

x = [pupilEdgePointOnECPAndPupilCenter(1,:);ECPIntersectionPointsOnInnerCorneaSphere(1,:);ECPIntersectionPointsOnOuterCorneaSphere(1,:);refractedECPProjectionsOnCamera(1,:)];
y = [pupilEdgePointOnECPAndPupilCenter(2,:);ECPIntersectionPointsOnInnerCorneaSphere(2,:);ECPIntersectionPointsOnOuterCorneaSphere(2,:);refractedECPProjectionsOnCamera(2,:)];
z = [pupilEdgePointOnECPAndPupilCenter(3,:);ECPIntersectionPointsOnInnerCorneaSphere(3,:);ECPIntersectionPointsOnOuterCorneaSphere(3,:);refractedECPProjectionsOnCamera(3,:)];
plot3(ax,x(:,3),y(:,3),z(:,3),'Color',[0 0 0],'LineWidth',1,'LineStyle','--');
plot3(ax,x(:,1:2), y(:,1:2), z(:,1:2),'k-');
x = [refractedECPProjectionsOnCamera(1,:);ones(size(refractedECPProjectionsOnCamera,2))*opticalCenter(1)];
y = [refractedECPProjectionsOnCamera(2,:);ones(size(refractedECPProjectionsOnCamera,2))*opticalCenter(2)];
z = [refractedECPProjectionsOnCamera(3,:);ones(size(refractedECPProjectionsOnCamera,2))*opticalCenter(3)];
plot3(ax,x,y,z,'Color',[0 0 0],'LineWidth',0.5,'LineStyle',':');

% draw perspective projection image of the pupil contour considering refraction
refractedPupilEdgeProjectionsOnCamera = linesPlaneIntersection(opticalCenter,intersectionPointsOnOuterCorneaSphere,imagePlanePoint,ax,0);
plot3(ax,refractedPupilEdgeProjectionsOnCamera(1,:),refractedPupilEdgeProjectionsOnCamera(2,:),refractedPupilEdgeProjectionsOnCamera(3,:),'Color',[0.8000 0.6000 0],'LineWidth',1);


% mesh the light raytracing
meshX1 = [refractedPupilEdgeProjectionsOnCamera(1,:);intersectionPointsOnOuterCorneaSphere(1,:)];
meshY1 = [refractedPupilEdgeProjectionsOnCamera(2,:);intersectionPointsOnOuterCorneaSphere(2,:)];
meshZ1 = [refractedPupilEdgeProjectionsOnCamera(3,:);intersectionPointsOnOuterCorneaSphere(3,:)];
surf(meshX1,meshY1,meshZ1,'FaceColor',[0.3010 0.7450 0.9330],'FaceAlpha',0.5,'EdgeColor','none');

meshX2 = [intersectionPointsOnInnerCorneaSphere(1,:);intersectionPointsOnOuterCorneaSphere(1,:)];
meshY2 = [intersectionPointsOnInnerCorneaSphere(2,:);intersectionPointsOnOuterCorneaSphere(2,:)];
meshZ2 = [intersectionPointsOnInnerCorneaSphere(3,:);intersectionPointsOnOuterCorneaSphere(3,:)];
surf(meshX2,meshY2,meshZ2,'FaceColor',[1 1 0],'FaceAlpha',0.5,'EdgeColor','none');

meshX3 = [intersectionPointsOnInnerCorneaSphere(1,:);transformedPupilPoints(1,:)];
meshY3 = [intersectionPointsOnInnerCorneaSphere(2,:);transformedPupilPoints(2,:)];
meshZ3 = [intersectionPointsOnInnerCorneaSphere(3,:);transformedPupilPoints(3,:)];
surf(meshX3,meshY3,meshZ3,'FaceColor',[0.5490 0.2235 0.8118],'FaceAlpha',0.5,'EdgeColor','none');

% set view of the axis
view([-59,28]);
xlabel('x');
ylabel('y');
zlabel('z');
grid off;
axis equal
xlim([-0.03 0.04]);
ylim([-0.06 0.02]);
zlim([-0.05 0.02]);

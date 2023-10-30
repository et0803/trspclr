function [intersectionPointsOnInnerCorneaSphere, intersectionPointsOnOuterCorneaSphere, directIntersectionPointsOuterCorneaSphere,refractedErrorOfAllPoint] = refractedPupilEdgeRayTracing(cameraOpticalCenterPoint,pupilEdgePoints,innerCorneaSphereRadius,innerCorneaSphereCenter,outerCorneaSphereRadius,outerCorneaSphereCenter,eyeHumorRefractionIndex, corneaRefractionIndex, airRefractionIndex)
% pupilEdgePoints在角膜内部的anterior chamber aqueous humor内部
% 求解所有pupilEdgePoints经过内、外角膜界面两次折射后，能够经过cameraOpticalCenterPoint的射线方向。
% 这些射线与角膜外侧的交点，为摄像头成像的ray tracing的光线追踪，用来分析虚像和通过真实位姿的几何关系。

% 角膜厚度均匀，内外表面近似平行的情况下。那么角膜层的厚度，会轻微影响光线追踪与角膜外表面的交点的位置偏置，不会影响视线从眼睛前房液到空气的视线偏转角度。
% 交点位置偏置和角膜厚度、角膜折射率、眼睛前房液折射率有关。

intersectionPointsOnInnerCorneaSphere = zeros(size(pupilEdgePoints));
intersectionPointsOnOuterCorneaSphere = zeros(size(pupilEdgePoints));
directIntersectionPointsOuterCorneaSphere = zeros(size(pupilEdgePoints));
errorCompensateRatio = 0.5;

refractedErrorOfAllPoint = zeros([1,size(pupilEdgePoints,2)]);

for i = 1:size(pupilEdgePoints,2)
    % 先求cameraOpticalCenterPoint和某一个pupilEdgePoints的连线，如角膜内球面的交点
    lineDirection = (cameraOpticalCenterPoint - pupilEdgePoints(:,i))/norm(cameraOpticalCenterPoint - pupilEdgePoints(:,i));
    directIntersectionPointsOuterCorneaSphere(:,i) = lineSphereIntersection(outerCorneaSphereCenter,outerCorneaSphereRadius,lineDirection,pupilEdgePoints(:,i));
    
    incidentDirection_innerCorneaSphere = lineDirection; %角膜内表面折射方向初始化
    intersectionPointOnInnerCorneaSphere = lineSphereIntersection(innerCorneaSphereCenter,innerCorneaSphereRadius,incidentDirection_innerCorneaSphere,pupilEdgePoints(:,i)); %角膜内表面折射点初始化
    
    attemptIndex=0;
    while 1
        %角膜内表面第一次折射
        normalDirectionToInnerCorneaSphereAtInnerIntersectionPoint  = (intersectionPointOnInnerCorneaSphere - innerCorneaSphereCenter)/norm(intersectionPointOnInnerCorneaSphere - innerCorneaSphereCenter);
        if norm(cross(incidentDirection_innerCorneaSphere,normalDirectionToInnerCorneaSphereAtInnerIntersectionPoint)) < 1e-10 %入射角为0
            incidentDirection_outerCorneaSphere = incidentDirection_innerCorneaSphere;
        else
            directionForRotateIncidentDirection_1 = cross(incidentDirection_innerCorneaSphere,normalDirectionToInnerCorneaSphereAtInnerIntersectionPoint)/norm(cross(incidentDirection_innerCorneaSphere,normalDirectionToInnerCorneaSphereAtInnerIntersectionPoint));
            incidentAngle_1 = acos(dot(incidentDirection_innerCorneaSphere,normalDirectionToInnerCorneaSphereAtInnerIntersectionPoint));
            refractedAngle_1 = asin(eyeHumorRefractionIndex*sin(incidentAngle_1)/corneaRefractionIndex);
            incidentDirection_outerCorneaSphere = axang2rotm([directionForRotateIncidentDirection_1',-refractedAngle_1])*normalDirectionToInnerCorneaSphereAtInnerIntersectionPoint;
        end
        
        %角膜外表面第二次折射
        intersectionPointOnOuterCorneaSphere = lineSphereIntersection(outerCorneaSphereCenter,outerCorneaSphereRadius,incidentDirection_outerCorneaSphere,intersectionPointOnInnerCorneaSphere);
        normalDirectionToOuterCorneaSphereAtOuterIntersectionPoint = (intersectionPointOnOuterCorneaSphere - outerCorneaSphereCenter)/norm(intersectionPointOnOuterCorneaSphere - outerCorneaSphereCenter);
        if norm(cross(incidentDirection_outerCorneaSphere,normalDirectionToOuterCorneaSphereAtOuterIntersectionPoint)) < 1e-10  %入射角为0
            refractedDirection_outerCorneaSphere = incidentDirection_outerCorneaSphere;
        else
            directionForRotateIncidentDirection_2 = cross(incidentDirection_outerCorneaSphere,normalDirectionToOuterCorneaSphereAtOuterIntersectionPoint)/norm(cross(incidentDirection_outerCorneaSphere,normalDirectionToOuterCorneaSphereAtOuterIntersectionPoint));
            incidentAngle_2 = acos(dot(incidentDirection_outerCorneaSphere,normalDirectionToOuterCorneaSphereAtOuterIntersectionPoint));
            refractedAngle_2 = asin(corneaRefractionIndex*sin(incidentAngle_2)/airRefractionIndex);
            refractedDirection_outerCorneaSphere = axang2rotm([directionForRotateIncidentDirection_2',-refractedAngle_2])*normalDirectionToOuterCorneaSphereAtOuterIntersectionPoint;
        end
        currentOutputDirection = (cameraOpticalCenterPoint-intersectionPointOnOuterCorneaSphere)/norm(cameraOpticalCenterPoint-intersectionPointOnOuterCorneaSphere);
        refractedError = acos(dot(refractedDirection_outerCorneaSphere,currentOutputDirection));
        
        intersectionPointsOnInnerCorneaSphere(:,i) = intersectionPointOnInnerCorneaSphere; %保存要输出的数据
        intersectionPointsOnOuterCorneaSphere(:,i) = intersectionPointOnOuterCorneaSphere;
        
        %fprintf("%dth attempt for the %dth point. Current error is %f\n",attemptIndex,i,refractedError);

        if abs(refractedError)>1e-5
            angleToCompensate = errorCompensateRatio*refractedError;
            rotationDirectionForCompensate = cross(refractedDirection_outerCorneaSphere,currentOutputDirection)/norm(cross(refractedDirection_outerCorneaSphere,currentOutputDirection));
            incidentDirection_innerCorneaSphere = axang2rotm([rotationDirectionForCompensate',angleToCompensate])*incidentDirection_innerCorneaSphere; %角膜内表面折射方向迭代修正
            intersectionPointOnInnerCorneaSphere = lineSphereIntersection(innerCorneaSphereCenter,innerCorneaSphereRadius,incidentDirection_innerCorneaSphere,pupilEdgePoints(:,i)); %角膜内表面折射点迭代修正
            
            attemptIndex=attemptIndex+1;
        else
            refractedErrorOfAllPoint(1,i)=refractedError;
            break;
        end
    end
end
end


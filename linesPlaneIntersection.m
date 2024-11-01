function [intersectionPoints] = linesPlaneIntersection(onePointOnOneEnd, multiplePointsOnTheOtherEnd, imagePlanePoints, h_ax, drawFlag)
% function [I,rc] = line_plane_intersection(u, N, n, M, verbose)
% - u : real row or column vector double. numel(u) = 3. One director vector of the parametric line.
% - N : real row or column vector double. numel(N) = 3. One point belonging to the line.
% - n : real row or column vector double. numel(n) = 3. One normal vector to the plane.
% - M : real row or column vector double. numel(M) = 3. One point belonging to the plane.

intersectionPoints = zeros(size(multiplePointsOnTheOtherEnd));
for i=1:size(multiplePointsOnTheOtherEnd,2)
    normalDirectionOfPlane = cross((imagePlanePoints(:,1)-imagePlanePoints(:,2)),(imagePlanePoints(:,1)-imagePlanePoints(:,3)));
    [I,rc] = linePlaneIntersection(multiplePointsOnTheOtherEnd(:,i)-onePointOnOneEnd, onePointOnOneEnd, normalDirectionOfPlane, imagePlanePoints(:,1));
    intersectionPoints(:,i) = I;
end

if drawFlag>0
    hold on;
    if size(multiplePointsOnTheOtherEnd,2)>1
        plot3(h_ax,intersectionPoints(1,:),intersectionPoints(2,:),intersectionPoints(3,:),'k-','LineWidth',0.5);
    else
        plot3(h_ax,intersectionPoints(1,:),intersectionPoints(2,:),intersectionPoints(3,:),'k.','LineWidth',0.5);
        x = [onePointOnOneEnd(1);multiplePointsOnTheOtherEnd(1)];
        y = [onePointOnOneEnd(2);multiplePointsOnTheOtherEnd(2)];
        z = [onePointOnOneEnd(3);multiplePointsOnTheOtherEnd(3)];
        plot3(h_ax,x,y,z,'k--','LineWidth',0.5);
    end
    hold off;
    
end




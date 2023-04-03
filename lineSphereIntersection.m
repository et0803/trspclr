function intersectionPoint = lineSphereIntersection(sphereCenter, sphereRadius, lineDirection, linePoint)
% 射线与球面的交点。
% 本函数只返回一个交点，即从linePoint出发，沿着lineDirection，能够与球面相交的点。
% 所以linePoint在球面内时，只返回1个交点；linePoint 在球面外时，可能返回0个、1个或者2个交点。
% 对直线用t和方向[dx, dy, dz]进行参数化
% 直线方程为  x = linePoint_x + t*dx, y = linePoint_y + t*dy, z = linePoint_z + t*dz
% 交点的t的代数方程为 (x - sphereCenter_x)^2 + (y - sphereCenter_y)^2 + (z - sphereCenter_z)^2 = sphereRadius^2,求解t
% 令 cx = linePoint_x - sphereCenter_x， cy = linePoint_y - sphereCenter_y， cz = linePoint_z - sphereCenter_z
% 则交点t的代数方程为 (cx + t*dx)^2 + (cy + t*dy)^2 + (cz + t*dz)^2 = sphereRadius^2

lineDirection = lineDirection/norm(lineDirection);
cx = linePoint(1) - sphereCenter(1);
cy = linePoint(2) - sphereCenter(2);
cz = linePoint(3) - sphereCenter(3);
dx = lineDirection(1);
dy = lineDirection(2);
dz = lineDirection(3);
r = sphereRadius;
a = dx*dx + dy*dy + dz*dz;
b = 2*(dx*cx + dy*cy + dz*cz);
c = cx*cx + cy*cy + cz*cz - r*r;
delta = b*b - 4*a*c;

if abs(imag(sqrt(delta)))<1e-10 %有实根
    t1=(-b+real(sqrt(delta)))/(2*a);
    t2=(-b-real(sqrt(delta)))/(2*a);
    
    if ~(t1<0) 
        intersectionPoint=linePoint + t1*lineDirection;
    end
    
    if ~(t2<0) 
        intersectionPoint=linePoint + t2*lineDirection;
    end
    
    if (~(t1<0)) && (~(t2<0))
        intersectionPoint = zeros(3,2);
        intersectionPoint(:,1)=linePoint + t1*lineDirection;
        intersectionPoint(:,2)=linePoint + t2*lineDirection;
    end
    
    if (t1<0) && (t2<0)
        intersectionPoint=[];
    end
else
    fprintf("no real intersection\n");
    intersectionPoint=[];
end
end


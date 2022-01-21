% rotation matrix of roll (X), pitch (Y) and yaw (Z)
% 
% inputs:
%   - euler: roll, pitch, yaw angles (r, p, y), in rad, [3x1]
% 
% outputs: 
%   - R: rotation matrix
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function R = rotXYZ(euler)

    r = euler(1);
    p = euler(2);
    y = euler(3);
    
    R = [cos(y)*cos(p), cos(y)*sin(p)*sin(r) - sin(y)*cos(r), cos(y)*sin(p)*cos(r) + sin(y)*sin(r); ...
         sin(y)*cos(p), sin(y)*sin(p)*sin(r) + cos(y)*cos(r), sin(y)*sin(p)*cos(r) - cos(y)*sin(r); ...
        -sin(p), cos(p)*sin(r), cos(p)*cos(r)];

end


function [T, Eps] = estimateRigidTransform(x, y)
% ESTIMATERIGIDTRANSFORM
%   [T, EPS] = ESTIMATERIGIDTRANSFORM(X, Y) estimates the rigid transformation
%   that best aligns x with y (in the least-squares sense).
%  
%   Reference: "Estimating Rigid Transformations" in 
%   "Computer Vision, a modern approach" by Forsyth and Ponce (1993), page 480
%   (page 717(?) of the newer edition)
%
%   Input:
%       X: 3xN, N 3-D points (N>=3)
%       Y: 3xN, N 3-D points (N>=3)
%
%   Output
%       T: the rigid transformation that aligns x and y as:  xh = T * yh
%          (h denotes homogenous coordinates)  
%          (corrspondence between points x(:,i) and y(:,i) is assumed)
%       
%       EPS: the smallest singular value. The closer this value it is 
%          to 0, the better the estimate is. (large values mean that the 
%          transform between the two data sets cannot be approximated
%          well with a rigid transform.
%
%   Babak Taati, 2003
%   (revised 2009)

if nargin ~= 2
    error('Requires two input arguments.')
end

if size(x,1)~=3 || size(y,1)~=3
    error('Input point clouds must be a 3xN matrix.');
end

if size(x, 2) ~= size(y,2)
    error('Input point clouds must be of the same size');
end                            

if size(x,2)<3 || size(y,2)<3
    error('At least 3 point matches are needed');
end                            
    
pointCount = length(x); % since x has N=3+ points, length shows the number of points
                    
x_centroid = sum(x,2) / pointCount;
y_centroid = sum(y,2) / pointCount; 

x_centrized = [x(1,:)-x_centroid(1) ; x(2,:)-x_centroid(2); x(3,:)-x_centroid(3)];
y_centrized = [y(1,:)-y_centroid(1) ; y(2,:)-y_centroid(2); y(3,:)-y_centroid(3)];

R12 = y_centrized' - x_centrized';
R21 = x_centrized - y_centrized;
R22_1 = y_centrized  + x_centrized;
R22 = crossTimesMatrix(R22_1(1:3,:));

B = zeros(4, 4);
A = zeros(4, 4, pointCount);
for ii=1:pointCount
    A(1:4,1:4,ii) = [0, R12(ii,1:3); R21(1:3,ii), R22(1:3,1:3,ii)];
    B = B + A(:,:,ii)' * A(:,:,ii);
end

[~, S, V] = svd(B);
quat = V(:,4);
rot = quat2rot(quat);

T1 = [eye(3,3), -y_centroid ; 0 0 0 1];
T2 = [rot, [0; 0; 0]; 0 0 0 1];
T3 = [eye(3,3), x_centroid ;  0 0 0 1];

T = T3 * T2 * T1;
Eps = S(4,4);

end

function V_times = crossTimesMatrix(V)
% CROSSTIMESMATRIX
%   V_TIMES = CROSSTIMESMATRIX(V) returns a 3x3 (or a series of 3x3) cross times matrices of input vector(s) V
% 
%   Input:
%       V a 3xN matrix, rpresenting a series of 3x1 vectors
% 
%   Output:   
%       V_TIMES (Vx) a series of 3x3 matrices where V_times(:,:,i) is the Vx matrix for the vector V(:,i)
% 
% 	Babak Taati, 2003
%   (revised 2009)

[a,b] = size(V);
V_times = zeros(a, 3, b);

% V_times(1,1,:) = 0;
V_times(1,2,:) = - V(3,:);
V_times(1,3,:) = V(2,:);

V_times(2,1,:) = V(3,:);
% V_times(2,2,:) = 0;
V_times(2,3,:) = - V(1,:);

V_times(3,1,:) = - V(2,:);
V_times(3,2,:) = V(1,:);
% V_times(3,3,:) = 0;
end

function R = quat2rot(Q)
% QUAT2ROT
%   R = QUAT2ROT(Q) converts a quaternion (4x1 or 1x4) into a 3x3 rotation mattrix
%
%   reference: google!
%
%   Babak Taati, 2003
%   (revised 2009)

q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);

R(1,1)  = q0*q0  +  q1*q1  -  q2*q2  -  q3*q3;
R(1,2)  = 2 * (q1*q2  -  q0*q3);
R(1,3)  = 2 * (q1*q3  +  q0*q2);

R(2,1)  = 2 * (q1*q2  +  q0*q3);
R(2,2)  = q0*q0  -  q1*q1  +  q2*q2  -  q3*q3;
R(2,3)  = 2 * (q2*q3  -  q0*q1);

R(3,1)  = 2 * (q1*q3  -  q0*q2);
R(3,2)  = 2 * (q2*q3  +  q0*q1);
R(3,3)  = q0*q0  -  q1*q1  -  q2*q2  +  q3*q3;
end
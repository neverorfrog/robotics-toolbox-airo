function triangle = plotTriangle(center,theta,scale,varargin)

theta = pi/2 - atan2(sin(theta),cos(theta));
center = center(:); %column vector

%Coordinate del triangolo
coordinates = [cos(theta) sin(theta) ; -sin(theta) cos(theta)] * [0 1 -1 ; 1 -1 -1]*scale + center;

% axis("equal");
if nargin == 4
    triangle = plot(polyshape(coordinates(1,:),coordinates(2,:)),"LineWidth",2,"FaceColor",varargin{1});
else
    triangle = plot(polyshape(coordinates(1,:),coordinates(2,:)),"LineWidth",2,"FaceColor","r");
end

end

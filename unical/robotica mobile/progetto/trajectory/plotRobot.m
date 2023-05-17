function yOut = plotRobot(yOut,varargin)

pauseT = 0.08;

if size(varargin) == 1
    scale = varargin{1};
else 
    scale = 1;
end

plot(yOut(1,1),yOut(1,2),"or");

for i = 1 : size(yOut) - 1
    plot(yOut(i,1),yOut(i,2),"or");
    triangle = plotTriangle([yOut(i,1),yOut(i,2)],yOut(i,3),scale);
    pause(pauseT);
    delete(triangle);
    plot([yOut(i,1) yOut(i+1,1)] , [yOut(i,2) yOut(i+1,2)],"r");
end
    




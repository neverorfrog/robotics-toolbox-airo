function r = visibility1(xline,yline,obstacles)

r = 1;

for i = 1 : size(obstacles,1)
    xobsti = obstacles(i,1); xobstf = obstacles(i,2);
    yobsti = obstacles(i,3); yobstf = obstacles(i,4);
    
    if obstacleIntersect2(xline,yline,[xobsti xobstf],[yobsti yobstf])
        r = 0; break;
    end
    
end


end

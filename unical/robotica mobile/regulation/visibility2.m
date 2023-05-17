function r = visibility2(xline,yline,obstacles,edges)

r = 1;

for i = 1 : size(obstacles,1)
    xobsti = obstacles(i,1); xobstf = obstacles(i,2);
    yobsti = obstacles(i,3); yobstf = obstacles(i,4);
    
    if obstacleIntersect2(xline,yline,[xobsti xobstf],[yobsti yobstf])
        r = 0; break;
    elseif obstacleIntersect2(xline,yline,[edges(i,1)-1 edges(i,2)+1],[edges(i,3) edges(i,4)])
        r = 0; break;
    end
    
end


end

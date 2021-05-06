function x = CoxdeBoor(t, controlPoints, knots, k)


% Find the index j such as knots(j+1) <= t < knots(j+2).
% If it can't find it, return the last point of the control points vector
% (assuming endpoint interpolation)
j = -1;
for i = 0 : (length(knots)-1)
    if t < knots(i+1)
        j = i - 1;
        break
    end
end
if j == -1
    x = controlPoints(end,:);
    return
end
 
for r = 1 : (k-1)
    for i = j : -1 : j-k+r+1
        w = (t-knots(i+1)) / (knots(i+k-r+1)-knots(i+1));
        controlPoints(i+1,:) = (1 - w)*controlPoints(i,:) + w*controlPoints(i+1,:);
    end
end
x = controlPoints(j+1,:);
return
 
 
 
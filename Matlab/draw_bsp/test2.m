clear;
%close all;

hold on;
grid on;

%% init control point
control_point =[1,2;1,2;2.61479049825092,2.97862817676773;2.80837016044980,0.815985025115673;5.37716725370785,2.44795007987969;4,-1;4,-1];
plot(control_point(:,1), control_point(:,2));

k = 3;
n = 5;

knots = [0,0,0,0,0.197901546205980,0.407352087233569,0.602861986117260,1,1,1,1];

x=0;y=0;mm=0;
m = 101;
X = ones(m, 2);

for j=1:(n-1)
    uu=(knots(j+3)):0.01:knots(j+4);
    for kk=1:length(uu)
        mm=mm+1;
        X(mm,1)=control_point(j,1)*RCoxdeBoor(j,3,knots,uu(kk))+control_point(j+1,1)*RCoxdeBoor(j+1,3,knots,uu(kk))+control_point(j+2,1)*RCoxdeBoor(j+2,3,knots,uu(kk))+control_point(j+3,1)*RCoxdeBoor(j+3,3,knots,uu(kk));
        X(mm,2)=control_point(j,2)*RCoxdeBoor(j,3,knots,uu(kk))+control_point(j+1,2)*RCoxdeBoor(j+1,3,knots,uu(kk))+control_point(j+2,2)*RCoxdeBoor(j+2,3,knots,uu(kk))+control_point(j+3,2)*RCoxdeBoor(j+3,3,knots,uu(kk));
    end
end

plot(X(:,1),X(:,2),'*');
 
xlabel('x');ylabel('y');
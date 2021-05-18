m=50;
control_point =[ 1,2;1,2;1,2; 1,2; 1.2,3; 1.5,1;  2,3; 2,3; 2,3; 2,3];

plot(control_point(:,1), control_point(:,2));
hold on;
grid on;


n=size(control_point);
n=n(1);
matrix = 1/6*[-1 3 -3 1; 3 -6 3 0; -3 0 3 0;1 4 1 0];
x=zeros(500,1);
y=zeros(500,1);
c=1;
for i=1:n-3
    P=zeros(4,2);
    P(1,1) = control_point(i,1);
    P(2,1) = control_point(i+1,1);
    P(3,1) = control_point(i+2,1);
    P(4,1) = control_point(i+3,1);
    P(1,2) = control_point(i,2);
    P(2,2) = control_point(i+1,2);
    P(3,2) = control_point(i+2,2);
    P(4,2) = control_point(i+3,2);
    res = matrix*P;
    
    for j=0:m-1
        t = j/m;
        x(c) = ((res(1,1)*t+res(2,1))*t+res(3,1))*t+res(4,1);
        y(c)= ((res(1,2)*t+res(2,2))*t+res(3,2))*t+res(4,2);
        c=c+1;        
    end
    
end

plot(x(1:c-1), y(1:c-1), '*');


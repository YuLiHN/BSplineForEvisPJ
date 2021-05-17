clear
k=3;
n=4;

load('interp.mat') 
data = zeros(n,2);

%% get 5 points to represent the spline
for i=1:25:101    
    index = (i-1)/25+1;
    data(index,1) = X(i,1);
    data(index,2) = X(i,2);
end
%plot data
plot(data(:,1),data(:,2),'o');
hold on

[n,m]=size(data);
% compute knots book p.236
% according to definition, the knots should be 
% U=[0,0,0,0, (l1+l2)/L, (l1+l2+l3)/L, ...(l1+...ln-2)/L,1,1,1,]; 
% In this case, n=5
knots(k+n)=0;
for i=1:n-1
    knots(k+i+1)=knots(k+i)+sqrt((data(i+1,1)-data(i,1))^2+(data(i+1,2)-data(i,2))^2);
end
    L=knots(n+k);
disp(L);
for i=1:n
    knots(k+i)=knots(k+i)/L;
end
for i=1:3
    knots(k+i+n)=1;
end
disp(knots);

%% calculating n+2 control points book p.257
%首位重节点v1=v2
%首位与控制多边形相切
%[a1 b1 c1 ... ][d1]  [e1]   
% this line representing the entry point boundary condition, a1=1,b1=c1=0,
%[a2 b2 c2 ... ][d2]  [e2]
%[..    
%[...      an-2 bn-2 cn-2]  [dn-2]  [en-2] 
%[...      an-1 bn-1 cn-1]  [dn-1]  [en-1]
% this line represent the final boundary condition, an-1 = bn-1 =0, c1 = 1

A=zeros(n+2);
%boundary condition
A(1,1)=1;A(1,2)=-1;
A(2,2)=1;
A(n+2,n+1)=-1;A(n+2,n+2)=1;
A(n+1,n+1)=1;
for i=3:n
  for j=0:2
    A(i,i+j-1)=RCoxdeBoor(i+j-1,k,knots,knots(i+2)); %p240
   end
end
disp(A)
%e:方程右边.
e=0;
for i=1:m
    e(n+2,i)=0;
end
for i=1:n
    e(i+1,:)=data(i,:);
end
%求出控制点d
control_point=inv(A)*e;
plot(control_point(:,1),control_point(:,2),'b');
plot(control_point(:,1),control_point(:,2),'b*'); 
hold on

mm = 0;
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

plot(X(:,1),X(:,2),'red');
 
xlabel('x');ylabel('y');

clear;
close all;

hold on;
grid on;

%% init control point

control_point =[1,2;2,3;3,1;5,2;4,-1;];
plot(control_point(:,1), control_point(:,2));

k = 3;
n = 4;
knots = [zeros(1,k-1) linspace(0, 1, n-k+3) ones(1,k-1)];

m = 101; 
t = linspace(0, 1, m);
X = ones(m, 2);
for i = 1 : m  
    X(i,:) = CoxdeBoor(t(i), control_point, knots, k);
end

plot(X(:,1), X(:,2), '*');
hold on;
grid on;
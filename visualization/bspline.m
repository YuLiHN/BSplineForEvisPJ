clear, clc, close ALL

%% read data
sizeA = [17 Inf];
formatSpec = '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
fileID = fopen('interpolation.txt','r');
data = fscanf(fileID,formatSpec,sizeA);
fclose(fileID);
num_poses = size(data);
num_poses = num_poses(2);
%% prepare data
T = zeros(4,4,num_poses);
Matrix_T = @(d) [d(2), d(3), d(4), d(5); 
                 d(6), d(7), d(8), d(9);
                 d(10), d(11), d(12), d(13);
                 d(14), d(15), d(16), d(17);];
for i =1:num_poses
     d = data(:,i);
     T(:,:,i) = Matrix_T(d);
end


%% Visualize how the coordinate axes transform using the interpolated poses
%xyz = [0 0 0; 1 0 0; 0 1 0; 0 0 1]';
xyz = [1 0 0; 0 1 0; 0 0 1; 0 0 0 ]';
xyz_homog = [xyz; ones(1,4)];

figure(1),
for k = 1:200%num_poses
    p = T(:,:,k)*xyz_homog;
    p(1,4) = p(1,4)*0.1;
    hold on,
    plot3(p(1,1:2),p(2,1:2),p(3,1:2),'r')  % plot x axis in red
    plot3(p(1,[1,3]),p(2,[1,3]),p(3,[1,3]),'g') % plot y axis in green
    plot3(p(1,[1,4]),p(2,[1,4]),p(3,[1,4]),'b') % plot z axis in blue
    axis vis3d, axis equal, grid on
    drawnow
end
hold off
xlabel('X'),ylabel('Y'),zlabel('Z')
title('Transformation of the coordinate axes')
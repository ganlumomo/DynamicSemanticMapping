flow=flow_read('000040_10_flow.png');
disp0=disp_read('000040_d0.png');
disp1=disp_read('000040_d1.png');
I1=imread('000040_10_L.png');
I2=imread('000040_10_R.png');
label=imread('000040_lab.png');
P_2=[7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01;...
    0.000000e+00 7.215377e+02 1.728540e+02 2.163791e-01;...
    0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03];

P_3= [7.215377e+02 0.000000e+00 6.095593e+02 -3.395242e+02;...
    0.000000e+00 7.215377e+02 1.728540e+02 2.199936e+00;...
    0.000000e+00 0.000000e+00 1.000000e+00 2.729905e-03];

C1=cameraParameters('IntrinsicMatrix',P_2(1:3,1:3)');
C2=cameraParameters('IntrinsicMatrix',P_3(1:3,1:3)');
m=P_3(1:3,4)-P_2(1:3,4);
st_param=stereoParameters(C1,C2,eye(3),m);
[J1, J2]=rectifyStereoImages(I1,I2,st_param);
xyz1=reconstructScene(disp0(9:365,7:1234),st_param);
xyz2=reconstructScene(disp1(9:365,7:1234),st_param);
[a b c]=size(xyz1);
xyz=reconstructScene(disp1(9:365,7:1234)-disp0(9:365,7:1234),st_param);

new_disp=zeros(size(disp0));
[a b c]=size(disp0);
for i=1:a
    for j=1:b
        x_new=floor(0*flow(i,j,1)+i);
        y_new=floor(1*flow(i,j,2)+j);
%         y_new=j;
        if x_new> 0 && y_new >0 && x_new<= a && y_new <=b
            if flow(i,j,3)
                new_disp(i,j)=disp1(x_new,y_new);
            end
        end
    end
end
xyz_new=reconstructScene(new_disp(9:365,7:1234),st_param);

[a b c]=size(xyz1);
delta=zeros(size(xyz1));
delta1=zeros(size(xyz1));
for i=1:a
    for j=1:b
        x_new=floor(1*flow(i,j,1)+i);
        y_new=floor(1*flow(i,j,2)+j);
        if x_new> 0 && y_new >0 && x_new<= a && y_new <=b
            if flow(i,j,3)
                delta(i,j,:)=xyz1(i,j,:)-xyz_new(x_new,y_new,:);
            end
            if label(i,j)
                delta1(i,j,:)=xyz1(i,j,:)-xyz_new(x_new,y_new,:);
            end
        end
    end
end


% x = xyz_new(:, :, 1);
% y = xyz_new(:, :, 2);
% z = xyz_new(:, :, 3);
% x = x(:);
% y = y(:);
% z = z(:);

output=truncate_xyz(xyz1,delta1,label);
points= delta1;
% points=(xyz_new);
x = points(:, :, 1);
y = points(:,:, 2);
z = points(:,:, 3);
x = x(:);
y = y(:);
z = z(:);
scatter3(x, y, z, '.')
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');

% 
% 
% x = xyz(:, :, 1);
% y = xyz(:, :, 2);
% z = xyz(:, :, 3);
% x = x(:);
% y = y(:);
% z = z(:);
% scatter3(x, y, z, '.')



% sceneflow=zeros(size(xyz1));
% for i=1:a
%     for j=1:b
%         if flow(i,j,3)
% %             x_new=floor(-flow(i,j,1)+i);
% %             y_new=floor(-flow(i,j,2)+j);
% % %             x_new=i;
% % %             y_new=j;
%             if x_new> 0 & y_new >0 &x_new<= a & y_new <=b
%                 sceneflow(i,j,:)=xyz2(x_new,y_new,:)-xyz1(i,j,:);
%             end
%         end
%     end
% end
% subplot(3,1,1), imagesc(sceneflow(:,:,1))
% subplot(3,1,2), imagesc(sceneflow(:,:,2))
% subplot(3,1,3), imagesc(sceneflow(:,:,3))

% x = xyz2(:, :, 1);
% y = xyz2(:, :, 2);
% z = xyz2(:, :, 3);
% x = x(:);
% y = y(:);
% z = z(:);
% scatter3(x, y, z, '.')
% 


x1 = xyz1(:, :, 1);
y1 = xyz1(:, :, 2);
z1 = xyz1(:, :, 3);

x1 = x1(:);
y1 = y1(:);
z1 =z1(:);
% hold on 
% scatter3(x1, y1, z1, '.r')
% hold off
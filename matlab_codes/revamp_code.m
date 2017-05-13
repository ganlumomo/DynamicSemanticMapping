flow=flow_read('000040_10_flow.png');
disp0=disp_read('000040_d0.png');
disp1=disp_read('000040_d1.png');

I1=imread('000040_10_L.png');
I2=imread('000040_10_R.png');
% label1=read_label('000040_10_seg.png');
% label2=read_label('000040_11_seg.png');
label=imread('000040_lab.png');


calibname = 'calib.txt';
T = readtable(calibname, 'Delimiter', 'space', 'ReadRowNames', true, 'ReadVariableNames', false);
A = table2array(T);

P_2 = vertcat(A(1,1:4), A(1,5:8), A(1,9:12));
P_3 = vertcat(A(2,1:4), A(2,5:8), A(2,9:12));

% load Tr2.mat

C1=cameraParameters('IntrinsicMatrix',P_2(1:3,1:3)');
C2=cameraParameters('IntrinsicMatrix',P_3(1:3,1:3)');
m=P_3(1:3,4)-P_2(1:3,4);
st_param=stereoParameters(C1,C2,eye(3),m);
[J1, J2]=rectifyStereoImages(I1,I2,st_param);
xyz1=reconstructScene(disp0(6:368,6:1235),st_param);

[a b c]=size(xyz1);
delta=zeros(size(xyz1));
xyz_trun=zeros(size(xyz1));
xyz_new=zeros(size(xyz1));
count=0;
points=zeros(a*b,2*c+1);
new_points=zeros(a*b,2*c+1);
for i=1:a
    for j=1:b
        if abs(xyz1(i,j,3))<100000
            xyz_trun(i,j,:)=xyz1(i,j,:);
            if disp1(i+6,j+6)>0 
                new_z=xyz1(i,j,3)*(disp0(i+6,j+6)/disp1(i+6,j+6));
                delta(i,j,3)=new_z-xyz1(i,j,3);
                if flow(i+6,j+6,3)
                    delta(i,j,1)=(new_z*(j+6-609.5593 + flow(i+6,j+6,1))-(j+6-609.5593)*xyz1(i,j,3))*(j+6-609.5593)/xyz1(i,j,1);
                    delta(i,j,2)=(new_z*(i+6-172.854+ flow(i+6,j+6,2))-(i+6-172.854)*xyz1(i,j,3))*(i+6-172.854)/xyz1(i,j,2);


    %                 delta(i,j,2)=xyz1(i,j,2)*(flow(i+6,j+6,2)*(disp0(i+6,j+6)/disp1(i+6,j+6))-1);
    %                 delta(i,j,1)=xyz1(i,j,1)*(flow(i+6,j+6,1)*(disp0(i+6,j+6)/disp1(i+6,j+6))-1);
                end
                count=count+1;
                points(count,1:3)=xyz_trun(i,j,:)/1000;
                points(count,4:6)=delta(i,j,:)/1000;
                points(count,7)=label1(i+6,j+6);
                
                %for the next frame
                xyz_new(i,j,:)=xyz_trun(i,j,:)+delta(i,j,:);
                pt=(Tr)*[permute(xyz_new(i,j,:),[3 2 1]);1];
                pt_xyz=pt(1:3)/pt(4);
                xyz_new(i,j,:)=permute(pt_xyz, [3 2 1]);
                new_points(count,1:3)=xyz_new(i,j,:)/1000;
                new_points(count,7)=label2(i+6,j+6);
            end
        end
    end
end

points=points(1:count,:);
new_points=new_points(1:count,:);

plot_pt=xyz_new;
x1 = plot_pt(:, :, 1);
y1 = plot_pt(:, :, 2);
z1 = plot_pt(:, :, 3);

x1 = x1(:);
y1 = y1(:);
z1 =z1(:);
% hold on 
scatter3(x1, y1, z1, '.r')
% hold offplot_pt=xyz_new;

plot_pt=xyz_trun;
x1 = plot_pt(:, :, 1);
y1 = plot_pt(:, :, 2);
z1 = plot_pt(:, :, 3);

x1 = x1(:);
y1 = y1(:);
z1 =z1(:);
hold on 
scatter3(x1, y1, z1, '.b')
hold off



xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
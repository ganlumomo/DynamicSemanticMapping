function [ P0,P1,K,Pd,m,b_m ] = loadStereoCalib( filename )
%LOADSTEREOCALIB Reads projection matrices of left and right camera in
% KITTI stereo benchmark format. 
%
% Returns:
% P0,P1 projection matrices of left and right cam,
% K     the camera matrix of the reference view,
% Pd    an extended projection matrix to handle disparity,
% m     the translation vector between left and right camera,
% b_m   the stereo baseline in meters

  data = dlmread(filename,' ',0,1);

  P0   = reshape(data(1,:),4,3)';
  P1   = reshape(data(2,:),4,3)';

  m    = P1(1:3,4);
  b_m  = -m(1,1)/P0(1,1);

  Pd   = [ P1(1,1:3), 0; ...
           P1(2,1:3), 0; ...
           0, 0, 0, -m(1,1);... 
           P1(3,1:3),0 ];

  K    = P0(1:3,1:3);         
         
end


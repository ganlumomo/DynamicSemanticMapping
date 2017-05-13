function camMats = loadCamMats(filename)

% open file
fid = fopen(filename,'r');

if fid<0
  camMats = [];
  return;
end

camMats = zeros(3,4,4);
for cam = 1:4
    rowP = readVariable(fid,['P' num2str(cam-1)], 1, 12);
    P = reshape(rowP, [4 3])';
    K = P; K(:,end) = 0;
    
    % K*[I|t] = P 
    I_t = K \ P;
    P(:,end) = I_t(1:end-1,end);
    camMats(:,:,cam)= P;
end

function A = readVariable(fid,name,M,N)

% rewind
fseek(fid,0,'bof');

% search for variable identifier
success = 1;
while success>0
  [str,success] = fscanf(fid,'%s',1);
  if strcmp(str,[name ':'])
    break;
  end
end

% return if variable identifier not found
if ~success
  A = [];
  return;
end

% fill matrix
A = zeros(M,N);
for m=1:M
  for n=1:N
    [val,success] = fscanf(fid,'%f',1);
    if success
      A(m,n) = val;
    else
      A = [];
      return;
    end
  end
end
function label=read_label(file)
img=imread(file);

[a b c]=size(img);
label=uint8(zeros(a,b));
for i=1:a
    for j=1:b
        pt=permute(img(i,j,:),[2 3 1]);
        if min(pt==[128 128 128])
            label(i,j)=1;
        elseif min(pt==[128 0 0])
            label(i,j)=2;
        elseif min(pt==[192 192 128])
            label(i,j)=3;
        elseif min(pt==[255 69 0])
            label(i,j)=4;
        elseif min(pt==[128 64 128])
            label(i,j)=5;
        elseif min(pt==[60 40 222])
            label(i,j)=6;
        elseif min(pt==[128 128 0])
            label(i,j)=7;
        elseif min(pt==[192 128 128])
            label(i,j)=8;
        elseif min(pt==[64 64 128])
            label(i,j)=9;
        elseif min(pt==[64 0 128])
            label(i,j)=10;
        elseif min(pt==[0 128 192])
            label(i,j)=12;
        else
            label(i,j)=11;
        end
    end
end
end
        
            



function arr_points=truncate_data(points,delta,label)
[a b c]= size(points);
lin_points=zeros(a*b,2*c+1);
count=0;
for i=1:a
    for j=1:b
        if points(i,j,3)^2 <=10^10 && points(i,j,1)^2 <=20000^2&& points(i,j,3)^2 >= 00^2 
            if ~(max(delta(i,j,:)) == Inf || min(delta(i,j,:)) == -Inf)
                count=count+1;
                lin_points(count,1:3)=points(i,j,:)/1000;
                lin_points(count,4:6)=delta(i,j,:)/1000;
                lin_points(count,7)=label(i,j);
            end
        end
    end
end
arr_points=lin_points(1:count,:);
end

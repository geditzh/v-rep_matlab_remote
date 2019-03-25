clc
clear all
format short

[x,y,z] = sphere(50);
z(z<0) = nan;
mesh(50*x,50*y,50*z);
hold on;
 
[x1,y1] = meshgrid(linspace(-50,50));
z1 = 50*cos(30/180*pi)*ones(size(x1));
mesh(x1,y1,z1)
axis equal
view(135,20)

figure(2)
[x2,y2,z2]=meshgrid(linspace(-50,50));
figure(2)
fv=x2.^2+y2.^2+z2.^2-2500;
contourslice(x2,y2,z2,fv,[],[],50*cos(30/180*pi),[0 0]);
view(135,20)

figure(3)
%Ï¸·Ö100¸öµã
R = 50*sin(30/180*pi);
theta = 0:pi/50:2*pi;
X = R*cos(theta);
Y = R*sin(theta);
Z = 50*cos(30/180*pi)*ones(1,length(theta));
plot(X,Y)


origin_point = [0;0;50];
results = [];
for i=1:length(theta)
    final_point = [X(i);Y(i);Z(i)];
    result = final_point*pinv(origin_point);
    F = @(x)([cos(x(3))*sin(x(2))*cos(x(1))+sin(x(3))*sin(x(1))-result(1,3);sin(x(3))*sin(x(2))*cos(x(1))-cos(x(3))*sin(x(1))-result(2,3);cos(x(2))*cos(x(1))-result(3,3)]);
    result = fsolve(F,[0,0,0])/pi*180; 
    results(i,:) = result(:,:);
end
file_name = 'data1.dat';
save(file_name,'results','-ascii');
% file_name = fopen('data.dat','w');
% [n,m] = size(results);
% for i=1:n
%     for j=1:m
%         if j==m
%             fprintf(file_name,'%d\r\n',results(i,j));
%         else
%             fprintf(file_name,'%d\r\t',results(i,j));
%         end
%     end
% end
% fclose(file_name);

figure(4)
check_points = [];
for i=1:length(results)
    R_t = myfun(results(i,1)/180*pi,results(i,2)/180*pi,results(i,3)/180*pi);
    check_points(i,:) = R_t*[0;0;105];
end
check_X = check_points(:,1);
check_Y = check_points(:,2);
check_Z = check_points(:,3);

finalpoint = [check_X./1000 check_Y./1000 check_Z./1000];
file=fopen('circle.txt','wt+');
[m,n]=size(finalpoint);
for i=1:1:m
    for j=1:1:n
        if j==n 
            fprintf(file,'%g\n',finalpoint(i,j));
        else
            fprintf(file,'%g\t',finalpoint(i,j));
        end
    end
end
fclose(file); 


plot3(check_X,check_Y,check_Z,'MarkerSize',0.5)



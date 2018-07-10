clear
m= 1:5 ;
n=1:5;
o= 1:5;
m =m';
n =n';
o =o';
[X, Y, Z] = meshgrid(m, n, o);
% s= ones(5,5,5) * 5
s= rand(5,5,5) 
figure(3)
hold off;
 scatter3( X(:), Y(:), Z(:), [], s(:), 'filled' )
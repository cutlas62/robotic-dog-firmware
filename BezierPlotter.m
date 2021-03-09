clf
clear

N = 6;
n = N - 1;
pointsX = [0 100 100 -100 -100 0];
pointsY = [-30 -30 -100 -100 -30 -30];

hold on
plot(pointsX, pointsY, 'h')
axis([-110 110 -110 0])
pbaspect([2 1 1])

for T = 0:0.01:1
   xi = 0;
   yi = 0;
    for i = 1:1:N
       k = i - 1;
       xi = xi + (factorial(n)/(factorial(k)*factorial(n-k))) * (1-T)^(n-k) * (T^k) * pointsX(i);
       yi = yi + (factorial(n)/(factorial(k)*factorial(n-k))) * (1-T)^(n-k) * (T^k) * pointsY(i);
    end
    plot(xi,yi,'m.')
end
hold off

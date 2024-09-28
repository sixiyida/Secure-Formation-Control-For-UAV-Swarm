epi = 0.5;
k = 0.5;
ra = 0.3;
r = 1;
i = 0;
j = 0;
for x = -2:0.1:2
    j = 0;
    i = i + 1;
    for y = -2:0.1:2
        j = j + 1;
        z(i, j) = epi * k^((ra - sqrt(x^2 + y^2))/(ra - r));

    end
end

x = -2:0.1:2
y = -2:0.1:2

mesh(x,y,z)
xlabel('$dx_{ij}$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
ylabel('$dy_{ij}$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
zlabel('$E$','interpreter','latex','FontName','Times NewRoman','FontSize',16);

figure(2)
L = 1;
l = 0.1;


i = 0;
j = 0;

for x = [-2:0.01:-0.2 0.2:0.01:2]
    j = 0;
    i = i + 1;
    for y = [-2:0.01:-0.2 0.2:0.01:2]
        j = j + 1;
        o(i, j) = -4 * (sqrt(x^2 + y^2)^2 - L^2) * (L^2 - l^2) / (sqrt(x^2 + y^2)^2 - l^2)^3;
    end
end
 x = [-2:0.01:-0.2 0.2:0.01:2]
y = [-2:0.01:-0.2 0.2:0.01:2]
mesh(x,y,o)
xlabel('$dx_{i0k}$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
ylabel('$dy_{i0k}$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
zlabel('$E$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
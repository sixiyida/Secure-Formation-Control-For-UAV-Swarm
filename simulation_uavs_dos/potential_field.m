epi = 0.5;
k = 0.5;
ra = 0.2;
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

L = 1;
l = 0.1;


i = 0;
for x = 0.2:0.1:2
    i = i + 1;
    o(i) = 4 * (x^2 - L^2) * (L^2 - l^2) / (x^2 - l^2)^3
end

x = 0.2:0.1:2

plot (x, o)
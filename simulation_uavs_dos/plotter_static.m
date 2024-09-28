%% 绘图

%% 图1 X-Y编队坐标 
figure(1);

len = length(dout);

color = ['k', 'b', 'g', 'r'];

marker = ['^', 's', 's', 's'];

for i = 1:N
    plot(dout(1, 4*i - 3), dout(1, 4*i - 1), 'Color', color(i), 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 4);
    hold on;
    plot(dout(len, 4*i - 3), dout(len, 4*i - 1), 'Color', color(i), 'Marker', marker(i), 'MarkerSize', 6, 'LineWidth', 4);
    hold on;
    plot(dout(:, 4*i - 3), dout(:, 4*i - 1), 'Color', color(i), 'LineWidth', 2);
    hold on;
end



grid on
xlabel('$x_{iX}(t)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
ylabel('$x_{iY}(t)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
axis equal;

%% 图2 X轴速度-时间
figure(2);

for i = 1:N
    plot(tout, dout(:, 4*i - 2), 'Color', color(i), 'LineWidth', 2);
    hold on;
end

grid on
xlabel('$t(s)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
ylabel('$v_{iX}(t)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);

figure(2);

for i = 1:N
    plot(tout, dout(:, 4*i - 2), 'Color', color(i), 'LineWidth', 2);
    hold on;
end

grid on
xlabel('$t(s)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
ylabel('$v_{iX}(t)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);

%% 图3 σ(t)
figure(3);


plot(tout, sigma_t - 1,'Color', 'b', 'LineWidth', 2)


grid on
xlabel('$t(s)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
ylabel('$\sigma(t)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
axis([0 50 -1 4])

%% 图4 跟踪误差
figure(4);
miss_vec = zeros(5001,12);
miss = zeros(5001,1);
for i = 1:5000
    h = [];
    for j = 1:3
        t = i/100;
        h = [h get_h(j, t)'];
    end
    miss_vec(i,:) = dout(i, 5:16) - h - [dout(i, 1:4), dout(i, 1:4), dout(i, 1:4)];

    miss(i,1) = miss_vec(i,:) * miss_vec(i,:)';
end

plot(tout, miss,'Color', 'b', 'LineWidth', 2)
xlabel('$t(s)$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
ylabel('$\zeta^T\zeta$','interpreter','latex','FontName','Times NewRoman','FontSize',16);
grid on
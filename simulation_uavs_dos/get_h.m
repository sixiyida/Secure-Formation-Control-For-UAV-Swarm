function h = get_h(i, t)
    w = 0.2;
    % para1 = [-9.14, -3.26, 15.91];
    % para2 = [-15.6, -14.26, -13.3];
    %if t < 30
    % if t < 0
    %     h = [h_getfx(para1(i), i, t), h_getfv(para1(i), i, t), h_getfx(para2(i), i, t), h_getfv(para2(i), i, t)]';
    % else 
    %     h = [10 * cos(0.1 * t + 2 * pi * (i - 1) / 3), -sin(0.1 * t + 2 * pi * (i - 1) / 3),... 
    %     10 * sin(0.1 * t + 2 * pi * (i - 1) / 3), cos(0.1 * t + 2 * pi * (i - 1) / 3)]';
    % end
    h = [5 * cos(w * t + 2 * pi * (i - 1) / 3), -sin(w * t + 2 * pi * (i - 1) / 3),... 
    5 * sin(w * t + 2 * pi * (i - 1) / 3), cos(w * t + 2 * pi * (i - 1) / 3)]';
end

% function sat = h_sat(theta_t)
%     if abs(theta_t) >= 1
%         sat = sign(theta_t);
%     else
%         sat = theta_t;
%     end
% end
% 
% function fx = h_getfx(h_dx, h_k, t)
%     %fx = h_dx + (10 * cos(h_k / 3) + h_dx) * h_sat(t / abs(10 * cos(h_k / 3) + h_dx));
%     fx = h_dx + h_sat(10 * cos(h_k / 3) + h_dx) * (t / abs(10 * cos(h_k / 3) + h_dx));
% end
% 
% function fv = h_getfv(h_dv, h_k, t)
%     %fv = sign(1 / abs(10 * cos(h_k / 3) + h_dv)) * abs(1 - sign(h_sat(t / abs(10 * cos(h_k / 3) + h_dv))));
% 
% end
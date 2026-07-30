% function plotMetricMap(l_drives_vec, l_driven_vec, metric_map, fig_title, colorbar_label)
% 
% [L_drives_grid, L_driven_grid] = meshgrid(l_drives_vec, l_driven_vec);
% 
% [best_value, best_l_drives, best_l_driven] = ...
%     findBestLength(metric_map, l_drives_vec, l_driven_vec);
% 
% figure;
% hold on;
% 
% contourf(L_drives_grid, L_driven_grid, metric_map, 30, 'LineColor', 'none');
% colorbar;
% 
% colormap jet;
% 
% plot(best_l_drives, best_l_driven, 'rp', ...
%     'MarkerSize', 14, ...
%     'MarkerFaceColor', 'r', ...
%     'LineWidth', 1.5);
% 
% xlabel('l\_drives / mm');
% ylabel('l\_driven / mm');
% 
% title(sprintf('%s\n最优：l\\_drives = %.2f mm, l\\_driven = %.2f mm, value = %.4f', ...
%     fig_title, best_l_drives, best_l_driven, best_value));
% 
% cb = colorbar;
% ylabel(cb, colorbar_label);
% 
% grid on;
% box on;
% 
% end


% 三维曲面图
function plotMetricMap(l_drives_vec, l_driven_vec, metric_map, fig_title, colorbar_label)

[L_drives_grid, L_driven_grid] = meshgrid(l_drives_vec, l_driven_vec);

[best_value, best_l_drives, best_l_driven] = ...
    findBestLength(metric_map, l_drives_vec, l_driven_vec);

figure;
hold on;

surf(L_drives_grid, L_driven_grid, metric_map, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.95);

plot3(best_l_drives, best_l_driven, best_value, 'rp', ...
    'MarkerSize', 16, ...
    'MarkerFaceColor', 'r', ...
    'LineWidth', 1.5);

xlabel('l\_drives / mm');
ylabel('l\_driven / mm');
zlabel(colorbar_label);

title(sprintf('%s\n最优：l\\_drives = %.2f mm, l\\_driven = %.2f mm, value = %.4f', ...
    fig_title, best_l_drives, best_l_driven, best_value));

colorbar;
colormap jet;
grid on;
box on;
view(45, 30);

end
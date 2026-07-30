function saveFigIEEE(filename)


    % figure; % 打开一个新图窗口
    % clf;    % 清空当前figure内容
    % hold on; % 允许在一个坐标轴上继续绘制新的曲线，而不覆盖旧曲线（不然每次plot会清掉以前的内容）


    % 全局设置Arial
    set(0, 'DefaultAxesFontName', 'Arial');     % 坐标轴字体
    set(0, 'DefaultTextFontName',  'Arial');    % 所有文本字体

    % 设置图窗口gcf
    set(gcf, 'Units', 'inches');                % gcf 的位置单位设置为英寸
    set(gcf, 'Position', [1, 1, 5, 4]);     % [左，下，宽，高] = 3.5x2.5 英寸     3.5*3.5:8a
    set(gcf, 'Color', 'w');                     % 背景设为白色
    set(gcf, 'PaperUnits', 'inches');           % 输出纸张单位设为英寸
    set(gcf, 'PaperPosition', [0 0 5 4]);   % 输出图尺寸（与 Position 匹配）
    set(gcf, 'PaperSize', [5 4]);           % PDF/PNG 的最终纸张大小

    % 设置坐标轴gca
    set(gca, 'FontName', 'Arial');              % 坐标轴字体 Arial
    set(gca, 'FontSize', 10);                    % 坐标轴字体大小   8:6a
    set(gca, 'FontWeight','normal');            % 坐标轴字体粗体bold
    set(gca, 'FontAngle','normal');             % 斜体 normal   italic/oblique
    set(gca, 'LineWidth', 1.0);                 % 坐标轴线宽（边框）
    set(gca, 'TickDir', 'in');                  % 刻度向内
    set(gca, 'Box', 'on');                      % 外边框开启
    set(gca,'XGrid','on');                     % 网格X轴
    set(gca,'YGrid','on');                     % 网格Y轴 
    set(gca,'ZGrid','on');                     % 网格Z轴
    xlim([-50 10]);                          % 设置坐标范围
    ylim([140 200]);
    % axis equal;% 坐标轴横纵等比例
    % view(3) % 3D视角
    % set(gca,'View',[45 30]) % 视角

    % 图例legend设置
    legend('off');
    % set(legend,'AutoUpdate','off');
    % set(legend,'Box','off');                     % 图例边框
    % set(legend,'Color','none');                  % 图例背景色
    % set(legend,'FontName','Arial');              % 图例字体
    % set(legend,'FontSize',10);                    % 图例字体大小
    % set(legend,'TextColor','black');             % 图例字体颜色
    % legend('Location','best');
    % 设置坐标轴公式字体方式
    % xlabel('$l_{in}$ (mm)', 'Interpreter', 'latex', 'FontName', 'Cambria');  


    % 高分辨率（600DPI）导出 PNG
    print(gcf, [filename '.png'], '-dpng', '-r600');  

    % 颜色
    % ppt_orange = [255/255, 85/255, 0/255];  % PPT橙色 RGB: [1, 0.33, 0]
    % word_blue = [0/255, 112/255, 192/255];  % Word蓝色 RGB: [0, 0.44, 0.75]
    % orange_light = [1.0, 0.65, 0.40];
    % orange_dark  = [1.0, 0.33, 0.00];
    % blue_light   = [0.70, 0.85, 1.00];
    % blue_dark    = [0.00, 0.30, 0.65];
end

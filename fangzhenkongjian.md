% UVW对位平台工作范围仿真（论文级可视化修正版）
% 目标：不改动正解/采样/统计计算，仅修改显示（绘图、图例、坐标轴、交互、导出）

clear; clc; close all;

% 平台参数设定
platform_length = 650; % mm
platform_width = 268;  % mm
l1 = 295.5; % U滑轨中心到平台中心的Y方向距离
l2 = 295.5; % V滑轨中心到平台中心的Y方向距离
l3 = 0;     % W滑轨中心在平台中心
max_travel = 30; % 最大行程 ±30mm

% 设定特定的UVW输入值来控制绿色轮廓位置
u_input = 15;  % U轴输入值
v_input = -10; % V轴输入值
w_input = 20;  % W轴输入值

fprintf('设定的UVW输入值: U=%.1fmm, V=%.1fmm, W=%.1fmm\n', u_input, v_input, w_input);

% 正解函数：已知输入 (u, v, w) 求位姿 (x, y, theta)
function [x, y, theta] = forward_kinematics(u, v, w, l1, l2, l3)
    % 计算转角
    theta = atan2(v - u, l1 + l2);

    % 避免奇异点
    if abs(theta) > pi/6
        theta = sign(theta) * pi/6;
    end

    % 简化计算（当l3=0时）
    cos_theta = cos(theta);
    sin_theta = sin(theta);
    sin2theta = sin(2*theta);

    % 位置计算（l3=0时的简化公式）
    x = -w * cos_theta^2 - (sin2theta/4) * (u + v);
    y = -(sin2theta/2) * w + (cos_theta^2/2) * (u + v);
end

% 计算上平台所有点的全局坐标
function global_points = get_platform_surface_points(center_x, center_y, theta, length, width, grid_density)
    % 在整个上平台表面创建密集网格点
    x_points = linspace(-length/2, length/2, grid_density);
    y_points = linspace(-width/2, width/2, grid_density);
    [X_local, Y_local] = meshgrid(x_points, y_points);

    % 将局部坐标点展平
    local_points = [X_local(:), Y_local(:)];

    % 旋转矩阵
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % 转换为全局坐标
    global_points = (R * local_points')' + [center_x, center_y];
end

% 计算上平台轮廓点
function contour_points = get_platform_contour(center_x, center_y, theta, length, width)
    % 定义矩形平台的四个角点（局部坐标系）
    half_length = length / 2;
    half_width = width / 2;
    local_corners = [-half_length, -half_width;  % 左下
                     half_length, -half_width;   % 右下
                     half_length, half_width;    % 右上
                     -half_length, half_width;   % 左上
                     -half_length, -half_width]; % 闭合回到左下

    % 旋转矩阵
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % 转换为全局坐标
    contour_points = (R * local_corners')' + [center_x, center_y];
end

% 工作空间分析 - 蒙特卡洛方法
num_samples_2d = 200;    % 二维平面图采样点数
grid_density_2d = 15;    % 平台表面离散化网格密度
num_samples_3d = 8000;   % 三维点云图采样点数

% 存储二维平面图数据
all_surface_points = [];

% 存储三维点云图数据
all_centers_3d = [];
all_thetas_3d = [];

fprintf('开始计算上平台表面工作范围，平台尺寸: %.0f×%.0fmm\n', platform_length, platform_width);

% 计算二维平面图数据（整个平台表面点）
fprintf('计算二维平面图数据（%d个样本，%d×%d网格）...\n', num_samples_2d, grid_density_2d, grid_density_2d);
for i = 1:num_samples_2d
    u = (2*rand() - 1) * max_travel;
    v = (2*rand() - 1) * max_travel;
    w = (2*rand() - 1) * max_travel;

    try
        [x, y, theta] = forward_kinematics(u, v, w, l1, l2, l3);
        surface_points = get_platform_surface_points(x, y, theta, platform_length, platform_width, grid_density_2d);
        all_surface_points = [all_surface_points; surface_points];
    catch
        continue;
    end

    if mod(i, 50) == 0
        fprintf('已处理 %d/%d 个样本\n', i, num_samples_2d);
    end
end

% 计算三维点云图数据
fprintf('计算三维点云图数据（%d个样本）...\n', num_samples_3d);
for i = 1:num_samples_3d
    u = (2*rand() - 1) * max_travel;
    v = (2*rand() - 1) * max_travel;
    w = (2*rand() - 1) * max_travel;

    try
        [x, y, theta] = forward_kinematics(u, v, w, l1, l2, l3);
        all_centers_3d = [all_centers_3d; x, y];
        all_thetas_3d  = [all_thetas_3d; theta];
    catch
        continue;
    end
end

fprintf('计算完成！\n');
fprintf('二维平面图: %d 个平台表面点\n', size(all_surface_points, 1));
fprintf('三维点云图: %d 个中心点\n', size(all_centers_3d, 1));

% 计算由指定UVW输入值确定的平台位置
[exact_x, exact_y, exact_theta] = forward_kinematics(u_input, v_input, w_input, l1, l2, l3);
fprintf('由UVW输入值确定的平台位置: X=%.2fmm, Y=%.2fmm, θ=%.2f°\n', exact_x, exact_y, rad2deg(exact_theta));

% =========================【显示改动】字体选择（避免中文变方块，Times可选）=========================
font_en = pick_first_font({'Times New Roman','Arial','Helvetica'});
font_cn = pick_first_font({'Microsoft YaHei','PingFang SC','SimHei','STHeiti','Arial Unicode MS','SansSerif'});
font_mono = pick_first_font({'Consolas','Menlo','Courier New','FixedWidth'});
% =================================================================================================

% =========================【显示改动】图窗 + 布局（tiledlayout更利于论文排版）=========================
fig = figure('Position', [80, 60, 1600, 900], 'Color', 'white', 'Renderer', 'opengl');

t = tiledlayout(fig, 2, 2);
t.Padding = 'compact';
t.TileSpacing = 'compact';
% =================================================================================================

% =========================【显示改动】主图：二维工作范围============================================
ax_main = nexttile(t, 1, [2 1]);

h1 = scatter(ax_main, all_surface_points(:,1), all_surface_points(:,2), 2, [0.2, 0.4, 0.8], 'filled', ...
        'MarkerEdgeAlpha', 0.05, 'MarkerFaceAlpha', 0.6);
hold(ax_main, 'on');
grid(ax_main, 'on');

% 绘制初始平台表面（零位，红色）
initial_surface = get_platform_surface_points(0, 0, 0, platform_length, platform_width, grid_density_2d);
h2 = scatter(ax_main, initial_surface(:,1), initial_surface(:,2), 4, 'r', 'filled', ...
        'MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.4);

% 初始平台轮廓（红色）
initial_contour = get_platform_contour(0, 0, 0, platform_length, platform_width);
plot(ax_main, initial_contour(:,1), initial_contour(:,2), 'r-', 'LineWidth', 3);

% UVW 指定位置轮廓（绿色）
moved_contour = get_platform_contour(exact_x, exact_y, exact_theta, platform_length, platform_width);
plot(ax_main, moved_contour(:,1), moved_contour(:,2), 'g-', 'LineWidth', 3);

% 工作范围轮廓（凸包）
K = convhull(all_surface_points(:,1), all_surface_points(:,2));
plot(ax_main, all_surface_points(K,1), all_surface_points(K,2), 'k-', 'LineWidth', 2);

% ----【显示改动】主图范围：只由数据决定 + 合理padding（避免上下大空白/竖向长条）----
all_x = [all_surface_points(:,1); initial_surface(:,1); initial_contour(:,1); moved_contour(:,1)];
all_y = [all_surface_points(:,2); initial_surface(:,2); initial_contour(:,2); moved_contour(:,2)];
x_range_data = [min(all_x), max(all_x)];
y_range_data = [min(all_y), max(all_y)];

pad_x = 0.05 * (x_range_data(2) - x_range_data(1));
pad_y = 0.08 * (y_range_data(2) - y_range_data(1));

xlim(ax_main, [x_range_data(1) - pad_x, x_range_data(2) + pad_x]);
ylim(ax_main, [y_range_data(1) - pad_y, y_range_data(2) + pad_y]);

% ----【显示改动】保持等比例（单位一致），但不让MATLAB通过扩展ylim来满足外框比例----
ax_main.DataAspectRatioMode = 'manual';
ax_main.DataAspectRatio = [1 1 1];
ax_main.PlotBoxAspectRatioMode = 'manual';
ax_main.PlotBoxAspectRatio = [diff(ax_main.XLim), diff(ax_main.YLim), 1];

% 轴样式（论文友好）
ax_main.Box = 'on';
ax_main.TickDir = 'out';
ax_main.TickLength = [0.012 0.012];
ax_main.LineWidth = 1.0;
ax_main.FontName = font_en;
ax_main.FontSize = 11;

xlabel(ax_main, 'X 方向 (mm)', 'FontName', font_cn, 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax_main, 'Y 方向 (mm)', 'FontName', font_cn, 'FontSize', 12, 'FontWeight', 'bold');
title(ax_main, 'UVW对位平台上平台表面工作范围', 'FontName', font_cn, 'FontSize', 14, 'FontWeight', 'bold');

% ----【显示改动】独立inset图例：不参与数据范围、不影响缩放平移、文字自适应不裁切----
refresh_inset_legend(fig, ax_main, font_cn, font_en);
fig.SizeChangedFcn = @(~,~) refresh_inset_legend(fig, ax_main, font_cn, font_en);

% 强制刷新，减少“白框/需点击才显示”等渲染时序问题
drawnow;
% =================================================================================================

% =========================【显示改动】右上：XY-θ 三维点云===========================================
ax3d = nexttile(t, 2);

theta_deg_3d = rad2deg(all_thetas_3d);

scatter3(ax3d, all_centers_3d(:,1), all_centers_3d(:,2), theta_deg_3d, ...
    6, theta_deg_3d, 'filled', 'MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.7);
grid(ax3d, 'on');
colormap(ax3d, jet);

xlabel(ax3d, 'X (mm)', 'FontName', font_en, 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax3d, 'Y (mm)', 'FontName', font_en, 'FontSize', 10, 'FontWeight', 'bold');
zlabel(ax3d, '转角 θ (°)', 'FontName', font_cn, 'FontSize', 10, 'FontWeight', 'bold');
title(ax3d, 'XY-θ三维工作空间', 'FontName', font_cn, 'FontSize', 11, 'FontWeight', 'bold');
view(ax3d, 45, 30);

ax3d.Box = 'on';
ax3d.TickDir = 'out';
ax3d.TickLength = [0.012 0.012];
ax3d.LineWidth = 1.0;
ax3d.FontName = font_en;
ax3d.FontSize = 10;

% ----【显示改动】颜色说明：使用ax3d自身范围定位，避免出界导致看不见----
xlim3 = xlim(ax3d); ylim3 = ylim(ax3d); zlim3 = zlim(ax3d);
tx = xlim3(1) + 0.03*diff(xlim3);
ty = ylim3(1) + 0.03*diff(ylim3);
tz = zlim3(1) + 0.03*diff(zlim3);

text(ax3d, tx, ty, tz, sprintf('颜色表示转角 θ\n范围: %.1f° ~ %.1f°', min(theta_deg_3d), max(theta_deg_3d)), ...
    'FontName', font_cn, 'FontSize', 9, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
% =================================================================================================

% =========================【显示改动】右下：角度分布直方图===========================================
axh = nexttile(t, 4);

histogram(axh, theta_deg_3d, 40, 'FaceColor', [0.2, 0.6, 0.8], 'FaceAlpha', 0.8, ...
    'EdgeColor', 'black', 'LineWidth', 0.5);
grid(axh, 'on');

xlabel(axh, '转角 θ (°)', 'FontName', font_cn, 'FontSize', 10, 'FontWeight', 'bold');
ylabel(axh, '点数', 'FontName', font_cn, 'FontSize', 10, 'FontWeight', 'bold');
title(axh, '转角分布直方图', 'FontName', font_cn, 'FontSize', 11, 'FontWeight', 'bold');

axh.Box = 'on';
axh.TickDir = 'out';
axh.TickLength = [0.012 0.012];
axh.LineWidth = 1.0;
axh.FontName = font_en;
axh.FontSize = 10;
% =================================================================================================

% =========================统计计算（保持不变）======================================================
x_range = [min(all_surface_points(:,1)), max(all_surface_points(:,1))];
y_range = [min(all_surface_points(:,2)), max(all_surface_points(:,2))];
coverage_width  = x_range(2) - x_range(1);
coverage_height = y_range(2) - y_range(1);

coverage_area = coverage_width * coverage_height;
platform_area = platform_length * platform_width;
coverage_ratio = coverage_area / platform_area;

stats_text = sprintf(['上平台表面工作范围统计:\n', ...
                     '平台尺寸: %.0f×%.0fmm (面积: %.0f mm²)\n', ...
                     '行程参数: U/V/W±%.1fmm\n', ...
                     '工作范围: %.1f×%.1fmm (面积: %.0f mm²)\n', ...
                     '覆盖率: %.1f%%\n', ...
                     'X扩展: +%.1f/%.1fmm\n', ...
                     'Y扩展: +%.1f/%.1fmm\n', ...
                     '转角范围: %.1f° 至 %.1f°\n', ...
                     '表面点数: %d, 中心点数: %d\n', ...
                     '指定UVW: U=%.1fmm, V=%.1fmm, W=%.1fmm\n', ...
                     '对应位置: X=%.2fmm, Y=%.2fmm, θ=%.2f°'], ...
                     platform_length, platform_width, platform_area, ...
                     max_travel, ...
                     coverage_width, coverage_height, coverage_area, ...
                     coverage_ratio * 100, ...
                     abs(x_range(1)), x_range(2), ...
                     abs(y_range(1)), y_range(2), ...
                     min(theta_deg_3d), max(theta_deg_3d), ...
                     size(all_surface_points, 1), size(all_centers_3d, 1), ...
                     u_input, v_input, w_input, ...
                     exact_x, exact_y, rad2deg(exact_theta));
% =================================================================================================

% =========================【显示改动】统计信息：不进论文图，单独窗口（可选）=========================
show_stats_window = true;  % 需要纯粹论文图：可以改成 false
if show_stats_window
    stats_fig = figure('Name','UVW工作范围统计（参考用，不进论文图）', ...
        'Color','white','Position',[120 120 650 450]);

    uicontrol(stats_fig, 'Style','edit', 'String', stats_text, ...
        'Units','normalized', 'Position',[0.03 0.03 0.94 0.94], ...
        'Max', 10, 'Min', 0, 'HorizontalAlignment','left', ...
        'FontName', font_mono, 'FontSize', 10, 'BackgroundColor','white');
end
% =================================================================================================

% 显示详细统计信息（保持不变）
fprintf('\n=== UVW对位平台上平台表面工作范围统计 ===\n');
fprintf('平台尺寸: %.0f×%.0fmm (面积: %.0f mm²)\n', platform_length, platform_width, platform_area);
fprintf('行程参数: U/V/W±%.1fmm\n', max_travel);
fprintf('工作范围宽度: %.1fmm\n', coverage_width);
fprintf('工作范围高度: %.1fmm\n', coverage_height);
fprintf('工作面积: %.0f mm²\n', coverage_area);
fprintf('覆盖率: %.1f%%\n', coverage_ratio * 100);
fprintf('X方向扩展: 向左%.1fmm, 向右%.1fmm\n', abs(x_range(1)), x_range(2));
fprintf('Y方向扩展: 向下%.1fmm, 向上%.1fmm\n', abs(y_range(1)), y_range(2));
fprintf('转角范围: %.1f° 至 %.1f°\n', min(theta_deg_3d), max(theta_deg_3d));
fprintf('上平台表面点数: %d\n', size(all_surface_points, 1));
fprintf('平台中心点数: %d\n', size(all_centers_3d, 1));
fprintf('指定UVW输入值: U=%.1fmm, V=%.1fmm, W=%.1fmm\n', u_input, v_input, w_input);
fprintf('对应平台位置: X=%.2fmm, Y=%.2fmm, θ=%.2f°\n', exact_x, exact_y, rad2deg(exact_theta));

% =========================【显示改动】论文级导出（可选）============================================
do_export = false;            % 需要导出就改 true
out_dir   = fullfile(pwd, 'export_figs');
base_name = 'UVW_workspace';

if do_export
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    % PDF（矢量优先；若出现伪影，可改ContentType="auto"或"image"）
    exportgraphics(fig, fullfile(out_dir, base_name + ".pdf"), ...
        'ContentType','vector', 'BackgroundColor','white');

    % EPS（同样支持；若期刊要求EPS可用）
    exportgraphics(fig, fullfile(out_dir, base_name + ".eps"), ...
        'ContentType','vector', 'BackgroundColor','white');

    % PNG（高分辨率位图，建议 600 DPI）
    exportgraphics(fig, fullfile(out_dir, base_name + ".png"), ...
        'Resolution',600, 'BackgroundColor','white');

    fprintf('\n已导出到目录: %s\n', out_dir);
end
% =================================================================================================

% ================================== 本脚本新增/修改的辅助函数（仅显示用途） =========================
function fontname = pick_first_font(candidates)
    % 选择系统中存在的第一个字体；若都不存在，退回SansSerif
    fontname = 'SansSerif';
    try
        available = listfonts;
        for i = 1:numel(candidates)
            if any(strcmpi(available, candidates{i}))
                fontname = candidates{i};
                return;
            end
        end
    catch
        % listfonts在某些环境不可用时，保持默认
    end
end

function refresh_inset_legend(fig, ax_main, font_cn, font_en)
    % 删除旧图例
    old = findall(fig, 'Tag', 'insetLegendAxes');
    if ~isempty(old), delete(old); end

    % 用 tightPosition 获取绘图区（plot box）位置，避免覆盖标题/标签
    try
        pos = tightPosition(ax_main); % Units一般为normalized
    catch
        pos = ax_main.Position;       % 兜底：使用轴Position
    end

    pad = 0.01;                 % 贴边留白（normalized）
    w = pos(3) * 0.36;          % 图例宽（相对plot box）
    h = pos(4) * 0.32;          % 图例高
    x = pos(1) + pos(3) - w - pad;
    y = pos(2) + pos(4) - h - pad;

    ax_leg = axes('Parent', fig, 'Units','normalized', 'Position',[x y w h], 'Tag','insetLegendAxes');
    axis(ax_leg, [0 1 0 1]);
    ax_leg.XTick = [];
    ax_leg.YTick = [];
    ax_leg.Box = 'on';
    ax_leg.LineWidth = 0.9;
    ax_leg.Color = 'white';

    % 关键：图例不捕获鼠标事件，避免影响zoom/pan
    ax_leg.PickableParts = 'none';
    ax_leg.HitTest = 'off';

    % 固定显示层，不要被“当前坐标轴”污染
    hold(ax_leg, 'on');

    % 行位置
    ys = [0.84 0.66 0.48 0.30 0.12];
    xm = 0.12;    % 标记x
    xt = 0.20;    % 文字x
    seg = 0.05;   % 线段半长

    % 先用一个偏大的字号，然后通过extent迭代缩小，保证不裁切
    font_try = 12;

    % 画符号 + 文本
    p1 = plot(ax_leg, xm, ys(1), 's', 'MarkerSize',7, 'MarkerFaceColor',[0.2 0.4 0.8], 'MarkerEdgeColor',[0.2 0.4 0.8]);
    t1 = text(ax_leg, xt, ys(1), '上平台表面工作范围', 'Units','normalized', 'FontName',font_cn, 'FontSize',font_try, ...
        'VerticalAlignment','middle', 'Interpreter','none');

    p2 = plot(ax_leg, xm, ys(2), 's', 'MarkerSize',7, 'MarkerFaceColor','r', 'MarkerEdgeColor','r');
    t2 = text(ax_leg, xt, ys(2), '初始平台位置', 'Units','normalized', 'FontName',font_cn, 'FontSize',font_try, ...
        'VerticalAlignment','middle', 'Interpreter','none');

    l3 = plot(ax_leg, [xm-seg xm+seg], [ys(3) ys(3)], 'r-', 'LineWidth',2.5);
    t3 = text(ax_leg, xt, ys(3), '初始平台轮廓', 'Units','normalized', 'FontName',font_cn, 'FontSize',font_try, ...
        'VerticalAlignment','middle', 'Interpreter','none');

    l4 = plot(ax_leg, [xm-seg xm+seg], [ys(4) ys(4)], 'g-', 'LineWidth',2.5);
    t4 = text(ax_leg, xt, ys(4), 'UVW指定位置轮廓', 'Units','normalized', 'FontName',font_cn, 'FontSize',font_try, ...
        'VerticalAlignment','middle', 'Interpreter','none');

    l5 = plot(ax_leg, [xm-seg xm+seg], [ys(5) ys(5)], 'k-', 'LineWidth',2.0);
    t5 = text(ax_leg, xt, ys(5), '工作范围轮廓', 'Units','normalized', 'FontName',font_cn, 'FontSize',font_try, ...
        'VerticalAlignment','middle', 'Interpreter','none');

    % 禁止子对象捕获事件（双保险）
    objs = [p1 p2 l3 l4 l5 t1 t2 t3 t4 t5];
    for k = 1:numel(objs)
        try
            objs(k).PickableParts = 'none';
            objs(k).HitTest = 'off';
        catch
        end
    end

    % 迭代缩小字体直到所有文字extent落在[0,1]内（避免裁切）
    texts = [t1 t2 t3 t4 t5];
    drawnow;
    for fs = font_try:-0.5:8
        for ti = 1:numel(texts)
            texts(ti).FontSize = fs;
        end
        drawnow;

        ok = true;
        for ti = 1:numel(texts)
            ext = get(texts(ti), 'Extent'); % normalized: [x y w h]
            if (ext(1) + ext(3)) > 0.98 || ext(1) < 0.02
                ok = false;
                break;
            end
        end
        if ok
            break;
        end
    end

    % 图例轴刻度字体用英文（可选；此处主要影响边框不多）
    ax_leg.FontName = font_en;

    hold(ax_leg, 'off');
end
% =================================================================================================

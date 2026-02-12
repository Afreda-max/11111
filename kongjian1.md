% UVW对位平台工作范围仿真（大图例版）
% 显示整个上平台面（650mm×268mm）在运动过程中覆盖的工作范围
% 完全修复MATLAB R2024b图例和colorbar bug
% 使用精确的UVW输入值控制绿色轮廓位置
% 大图例框，文字跟随自适应

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
% 二维平面图使用较少的样本但密集的点
num_samples_2d = 200;    % 二维平面图采样点数
grid_density_2d = 15;    % 平台表面离散化网格密度

% 三维点云图使用较多的点
num_samples_3d = 8000;   % 三维点云图采样点数

% 存储二维平面图数据
all_surface_points = []; % 存储整个平台表面点

% 存储三维点云图数据
all_centers_3d = [];     % 存储平台中心点
all_thetas_3d = [];      % 存储平台转角

fprintf('开始计算上平台表面工作范围，平台尺寸: %.0f×%.0fmm\n', platform_length, platform_width);

% 计算二维平面图数据（整个平台表面点）
fprintf('计算二维平面图数据（%d个样本，%d×%d网格）...\n', num_samples_2d, grid_density_2d, grid_density_2d);
for i = 1:num_samples_2d
    % 随机生成输入 (u, v, w) 在行程范围内
    u = (2*rand() - 1) * max_travel;
    v = (2*rand() - 1) * max_travel;
    w = (2*rand() - 1) * max_travel;
    
    % 计算正解（平台中心位置）
    try
        [x, y, theta] = forward_kinematics(u, v, w, l1, l2, l3);
        
        % 计算整个平台表面的全局坐标
        surface_points = get_platform_surface_points(x, y, theta, platform_length, platform_width, grid_density_2d);
        
        % 存储所有表面点
        all_surface_points = [all_surface_points; surface_points];
        
    catch
        continue;
    end
    
    % 显示进度
    if mod(i, 50) == 0
        fprintf('已处理 %d/%d 个样本\n', i, num_samples_2d);
    end
end

% 计算三维点云图数据
fprintf('计算三维点云图数据（%d个样本）...\n', num_samples_3d);
for i = 1:num_samples_3d
    % 随机生成输入 (u, v, w) 在行程范围内
    u = (2*rand() - 1) * max_travel;
    v = (2*rand() - 1) * max_travel;
    w = (2*rand() - 1) * max_travel;
    
    % 计算正解（平台中心位置）
    try
        [x, y, theta] = forward_kinematics(u, v, w, l1, l2, l3);
        
        % 存储中心点数据
        all_centers_3d = [all_centers_3d; x, y];
        all_thetas_3d = [all_thetas_3d; theta];
        
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

% 创建科研论文级别可视化
fig = figure('Position', [100, 100, 1400, 1000], 'Color', 'white');

% 主图：上平台表面工作范围（单一颜色）
subplot(2,2,[1,3]);
% 使用较小的点大小（从3减小到2）
h1 = scatter(all_surface_points(:,1), all_surface_points(:,2), 2, [0.2, 0.4, 0.8], 'filled', ...
        'MarkerEdgeAlpha', 0.05, 'MarkerFaceAlpha', 0.6);

hold on;
grid on;

% 绘制初始平台表面（零位，使用红色）
initial_surface = get_platform_surface_points(0, 0, 0, platform_length, platform_width, grid_density_2d);
h2 = scatter(initial_surface(:,1), initial_surface(:,2), 4, 'r', 'filled', ...  % 从5减小到4
        'MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.4);

% 添加初始平台轮廓（红色粗线）
initial_contour = get_platform_contour(0, 0, 0, platform_length, platform_width);
plot(initial_contour(:,1), initial_contour(:,2), 'r-', 'LineWidth', 3, 'DisplayName', '初始平台轮廓');

% 添加由UVW输入值确定的平台轮廓（绿色粗线）
moved_contour = get_platform_contour(exact_x, exact_y, exact_theta, platform_length, platform_width);
plot(moved_contour(:,1), moved_contour(:,2), 'g-', 'LineWidth', 3, 'DisplayName', 'UVW指定位置轮廓');

% 添加工作范围轮廓线
% 计算凸包来获取轮廓
K = convhull(all_surface_points(:,1), all_surface_points(:,2));
plot(all_surface_points(K,1), all_surface_points(K,2), 'k-', 'LineWidth', 2, 'DisplayName', '工作范围轮廓');

% 设置图形属性
axis equal;
xlabel('X 方向 (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y 方向 (mm)', 'FontSize', 12, 'FontWeight', 'bold');
title('UVW对位平台上平台表面工作范围', 'FontSize', 14, 'FontWeight', 'bold');

% 大图例框 - 根据坐标轴范围动态调整，更靠近主图
% 获取当前坐标轴范围
ax = gca;
xlim_vals = get(ax, 'XLim');
ylim_vals = get(ax, 'YLim');

% 计算大图例框大小（基于图形范围的较大比例）
range_x = xlim_vals(2) - xlim_vals(1);
range_y = ylim_vals(2) - ylim_vals(1);

% 设置更大的图例框尺寸
legend_width = range_x * 0.25;  % 增加到25%
legend_height = range_y * 0.28; % 增加到28%

% 计算图例框位置，更靠近主图（减少边距）
legend_x = xlim_vals(2) - legend_width * 1.02;  % 减少边距至1.02倍
legend_y = ylim_vals(2) - legend_height * 1.02; % 减少边距至1.02倍

% 绘制大图例背景（白色填充，黑色边框）
rectangle('Position', [legend_x, legend_y, legend_width, legend_height], ...
    'FaceColor', [1, 1, 1], 'EdgeColor', 'k', 'LineWidth', 0.8);

% 计算文字大小和行间距，确保文字适合大框
text_size = 10;  % 使用更大字体
line_spacing = legend_height / 6;  % 六行内容分配高度

% 计算文字水平偏移量，确保文字不会超出框外
margin_left = legend_width * 0.08;  % 增加左边距
marker_size = legend_width * 0.05;  % 增加标记大小
text_start_x = legend_x + margin_left + marker_size + legend_width * 0.03;

% 计算每行垂直位置，均匀分布
row_positions = [
    legend_y + legend_height * 0.92,  % 第一行
    legend_y + legend_height * 0.78,  % 第二行
    legend_y + legend_height * 0.64,  % 第三行
    legend_y + legend_height * 0.50,  % 第四行
    legend_y + legend_height * 0.36,  % 第五行
    legend_y + legend_height * 0.22   % 第六行
];

% 绘制图例内容
% 第一行：上平台表面工作范围
marker_x = legend_x + margin_left + marker_size/2;
% 使用方形标记以更好地填充空间
rectangle('Position', [marker_x - marker_size/2, row_positions(1) - marker_size/3, marker_size, marker_size*0.66], ...
    'FaceColor', [0.2, 0.4, 0.8], 'EdgeColor', [0.2, 0.4, 0.8], 'LineWidth', 0.5);
text(text_start_x, row_positions(1), '上平台表面工作范围', ...
     'FontSize', text_size, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');

% 第二行：初始平台位置
rectangle('Position', [marker_x - marker_size/2, row_positions(2) - marker_size/3, marker_size, marker_size*0.66], ...
    'FaceColor', 'r', 'EdgeColor', 'r', 'LineWidth', 0.5);
text(text_start_x, row_positions(2), '初始平台位置', ...
     'FontSize', text_size, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');

% 第三行：初始平台轮廓
line([marker_x - marker_size/2, marker_x + marker_size/2], [row_positions(3), row_positions(3)], ...
     'Color', 'r', 'LineWidth', 2.5);
text(text_start_x, row_positions(3), '初始平台轮廓', ...
     'FontSize', text_size, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');

% 第四行：UVW指定位置轮廓
line([marker_x - marker_size/2, marker_x + marker_size/2], [row_positions(4), row_positions(4)], ...
     'Color', 'g', 'LineWidth', 2.5);
text(text_start_x, row_positions(4), 'UVW指定位置轮廓', ...
     'FontSize', text_size, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');

% 第五行：工作范围轮廓
line([marker_x - marker_size/2, marker_x + marker_size/2], [row_positions(5), row_positions(5)], ...
     'Color', 'k', 'LineWidth', 2);
text(text_start_x, row_positions(5), '工作范围轮廓', ...
     'FontSize', text_size, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');

% 第六行：保留为空或添加其他项目
text(text_start_x, row_positions(6), '', ...
     'FontSize', text_size, 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');

% XY-θ三维点云图 - 完全避免使用colorbar
subplot(2,2,2);
theta_deg_3d = rad2deg(all_thetas_3d);

% 使用较小的点大小（从8减小到6）
h3 = scatter3(all_centers_3d(:,1), all_centers_3d(:,2), theta_deg_3d, ...
         6, theta_deg_3d, 'filled', 'MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.7);

% 手动设置colormap而不使用colorbar
colormap(jet);
% 不再使用colorbar，改为手动创建颜色条说明
% 获取当前坐标轴
ax2 = subplot(2,2,2);
% 添加颜色说明文本
text_pos_x = xlim_vals(1) + 0.05*(xlim_vals(2)-xlim_vals(1));  % 左侧位置
text_pos_y = ylim_vals(1) + 0.05*(ylim_vals(2)-ylim_vals(1));  % 底部位置
text(text_pos_x, text_pos_y, sprintf('颜色表示转角 θ\n范围: %.1f° ~ %.1f°', min(theta_deg_3d), max(theta_deg_3d)), ...
     'Parent', ax2, 'FontSize', 9, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontWeight', 'bold');

grid on;
xlabel('X (mm)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Y (mm)', 'FontSize', 10, 'FontWeight', 'bold');
zlabel('转角 θ (°)', 'FontSize', 10, 'FontWeight', 'bold');
title('XY-θ三维工作空间', 'FontSize', 11, 'FontWeight', 'bold');
view(45, 30);

% 角度分布直方图
subplot(2,2,4);
h4 = histogram(theta_deg_3d, 40, 'FaceColor', [0.2, 0.6, 0.8], 'FaceAlpha', 0.8, ...
          'EdgeColor', 'black', 'LineWidth', 0.5);
xlabel('转角 θ (°)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('点数', 'FontSize', 10, 'FontWeight', 'bold');
title('转角分布直方图', 'FontSize', 11, 'FontWeight', 'bold');
grid on;

% 计算覆盖范围统计
x_range = [min(all_surface_points(:,1)), max(all_surface_points(:,1))];
y_range = [min(all_surface_points(:,2)), max(all_surface_points(:,2))];
coverage_width = x_range(2) - x_range(1);
coverage_height = y_range(2) - y_range(1);

% 计算覆盖面积（使用点云边界框）
coverage_area = coverage_width * coverage_height;
platform_area = platform_length * platform_width;
coverage_ratio = coverage_area / platform_area;

% 添加统计信息框
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

annotation('textbox', [0.02, 0.02, 0.35, 0.25], 'String', stats_text, ...
           'FontSize', 9, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
           'VerticalAlignment', 'bottom', 'FontWeight', 'bold');

% 显示详细统计信息
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

% =====================================================================
% 如何调整图中显示点的数量：
% 
% 1. 调整二维平面图的点数（上平台表面点）：
%    - 修改 num_samples_2d 参数：减少数值可以减少点数
%    - 修改 grid_density_2d 参数：减少数值可以减少点数
%    当前设置：num_samples_2d = 200, grid_density_2d = 15
%    总点数 = 200 × 15×15 = 45,000 点
%
% 2. 调整三维点云图的点数（平台中心点）：
%    - 修改 num_samples_3d 参数：增加数值可以增加点数
%    当前设置：num_samples_3d = 8000
%    总点数 = 8000 点
%
% 3. 如果需要进一步调整：
%    - 二维平面图：可以尝试 num_samples_2d = 150, grid_density_2d = 12 (21,600点)
%    - 三维点云图：可以尝试 num_samples_3d = 12000 (12000点)
%
% 4. 调整点的大小（可选）：
%    - 二维平面图：修改 scatter 函数中的点大小参数（当前为2）
%    - 三维点云图：修改 scatter3 函数中的点大小参数（当前为6）
%    - 初始平台表面：修改 scatter 函数中的点大小参数（当前为4）
%
% 5. 修改UVW输入值：
%    - 修改 u_input, v_input, w_input 变量来改变绿色轮廓位置
%    当前设置：u_input = 15, v_input = -10, w_input = 20
% =====================================================================

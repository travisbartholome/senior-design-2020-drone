clc; clear; close all;

csv_dir = './local-figures/';

t_r_data = readmatrix(strcat(csv_dir, 't_r_data.csv'));
t_data = readmatrix(strcat(csv_dir, 't_data.csv'));
y_data = readmatrix(strcat(csv_dir, 'y_data.csv'));
z_data = readmatrix(strcat(csv_dir, 'z_data.csv'));

% Times at which we want to find interpolated data values
t_interp = 0:0.1:5;

num_rows = length(t_r_data);
num_cols = length(t_interp);

% Preallocate space for interpolated y and z data
y_interp = zeros(num_rows, num_cols);
z_interp = zeros(num_rows, num_cols);

% Plot drone movement
figure(1);
plot_colors = hsv(length(t_r_data));
colormap(plot_colors);
colorbar;
caxis([0, 5]);
hold on;
for ind = 1:num_rows
    disp(t_r_data(ind));
    
    % Filter out NaNs from the raw data for this row
    t_data_filtered = t_data(ind, ~isnan(t_data(ind, :)));
    y_data_filtered = y_data(ind, ~isnan(y_data(ind, :)));
    z_data_filtered = z_data(ind, ~isnan(z_data(ind, :)));
    
    % Create versions of the data vectors with regularized time intervals
    y_interp(ind, :) = interp1(t_data_filtered, y_data_filtered, t_interp);
    z_interp(ind, :) = interp1(t_data_filtered, z_data_filtered, t_interp);
    
    plot(y_interp(ind, :), z_interp(ind, :), ...
        '.', 'color', plot_colors(ind, :));
end
title('Flight paths for t_T = 0, t_R \epsilon [0, 5], t_{tot} = 5');
ylabel('Height');
xlabel('Horizontal position');
saveas(gcf, './local-figures/flight-paths-hsv.png');

% Plot isochrones
figure(2);
hold on;
iso_indexes = 1:3:num_cols;
plot_colors = hsv(length(iso_indexes));
colormap(plot_colors);
colorbar;
caxis([0, 5]);
for iso_ind = 1:length(iso_indexes)
    ind = iso_indexes(iso_ind);
    plot(y_interp(:, ind), z_interp(:, ind), ...
        '-', 'color', plot_colors(iso_ind, :));
end
title('Isochrones for t_T = 0, t_R \epsilon [0, 5], t_{tot} = 5');
ylabel('Height');
xlabel('Horizontal position');
saveas(gcf, './local-figures/isochrones-hsv.png');

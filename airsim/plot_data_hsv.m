clc; clear; close all;

t_r_data = csvread('./local-figures/t_r_data.csv');
t_data = csvread('./local-figures/t_data.csv');
y_data = csvread('./local-figures/y_data.csv');
z_data = csvread('./local-figures/z_data.csv');

% Plot drone movement
figure(1);
plot_colors = hsv(length(t_r_data));
colormap(plot_colors);
colorbar;
hold on;
for ind = 1:length(t_r_data)
    disp(t_r_data(ind));
    plot(y_data(ind, :), z_data(ind, :), '.', 'color', plot_colors(ind, :));
end

title('Flight paths for t_T = 0, t_R \epsilon [0, 5], t_{tot} = 5');
ylabel('Height');
xlabel('Horizontal position');
saveas(gcf, './local-figures/flight-paths-hsv.png');

% Plot isochrones
figure(2);
times = 1:size(y_data, 2);
hold on;
for ind = 1:5:length(times)
    plot(y_data(:, times(ind)), z_data(:, times(ind)));
end


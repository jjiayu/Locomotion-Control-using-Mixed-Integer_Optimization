%Plot Trajecotries

%Plot y
figure(1)
plot(TimeSeries, y_result,'LineWidth',2)

%Plot theta
figure(2)
plot(TimeSeries, theta_result, 'LineWidth', 2)

%Plot Force Profiles
figure(3)
hold on
plot(TimeSeries, [FFx_result;FFx_result(end)], 'LineWidth', 2)
plot(TimeSeries, [FFy_result;FFy_result(end)], 'LineWidth', 2)
plot(TimeSeries, [FHx_result;FHx_result(end)], 'LineWidth', 2)
plot(TimeSeries, [FHy_result;FHy_result(end)], 'LineWidth', 2)
hold off

%Plot Foot Locations
figure(4)
hold on
plot(PFx_result, PFy_result, 'LineWidth', 2)
plot(PHx_result, PHy_result , 'LineWidth', 2)
hold off
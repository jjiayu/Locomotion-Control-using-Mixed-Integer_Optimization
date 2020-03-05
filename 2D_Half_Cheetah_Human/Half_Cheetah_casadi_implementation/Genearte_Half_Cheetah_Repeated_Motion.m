TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %one cycle to two cycle
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %two cycle to four cycle
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %four cycle to eight cycle
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %eight cycle to 16 cycles

x_result = [x_result;x_result(end)+x_result(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];

y_result = [y_result;speed*Tend*sin(terrain_slope_rad)+y_result(2:end)];
y_result = [y_result;speed*Tend*sin(terrain_slope_rad)*2+y_result(2:end)];
y_result = [y_result;speed*Tend*sin(terrain_slope_rad)*4+y_result(2:end)];
y_result = [y_result;speed*Tend*sin(terrain_slope_rad)*8+y_result(2:end)];

theta_result = [theta_result;theta_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];

PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];

PFy_result = [PFy_result;speed*Tend*sin(terrain_slope_rad)+PFy_result(2:end)];
PFy_result = [PFy_result;speed*Tend*sin(terrain_slope_rad)*2+PFy_result(2:end)];
PFy_result = [PFy_result;speed*Tend*sin(terrain_slope_rad)*4+PFy_result(2:end)];
PFy_result = [PFy_result;speed*Tend*sin(terrain_slope_rad)*8+PFy_result(2:end)];

PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];

PHy_result = [PHy_result;speed*Tend*sin(terrain_slope_rad)+PHy_result(2:end)];
PHy_result = [PHy_result;speed*Tend*sin(terrain_slope_rad)*2+PHy_result(2:end)];
PHy_result = [PHy_result;speed*Tend*sin(terrain_slope_rad)*4+PHy_result(2:end)];
PHy_result = [PHy_result;speed*Tend*sin(terrain_slope_rad)*8+PHy_result(2:end)];

PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];

PFcenterY_result_world = [PFcenterY_result_world;speed*Tend*sin(terrain_slope_rad)+PFcenterY_result_world(2:end)];
PFcenterY_result_world = [PFcenterY_result_world;speed*Tend*sin(terrain_slope_rad)*2+PFcenterY_result_world(2:end)];
PFcenterY_result_world = [PFcenterY_result_world;speed*Tend*sin(terrain_slope_rad)*4+PFcenterY_result_world(2:end)];
PFcenterY_result_world = [PFcenterY_result_world;speed*Tend*sin(terrain_slope_rad)*8+PFcenterY_result_world(2:end)];

PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];

PHcenterY_result_world = [PHcenterY_result_world;speed*Tend*sin(terrain_slope_rad)+PHcenterY_result_world(2:end)];
PHcenterY_result_world = [PHcenterY_result_world;speed*Tend*sin(terrain_slope_rad)*2+PHcenterY_result_world(2:end)];
PHcenterY_result_world = [PHcenterY_result_world;speed*Tend*sin(terrain_slope_rad)*4+PHcenterY_result_world(2:end)];
PHcenterY_result_world = [PHcenterY_result_world;speed*Tend*sin(terrain_slope_rad)*8+PHcenterY_result_world(2:end)];

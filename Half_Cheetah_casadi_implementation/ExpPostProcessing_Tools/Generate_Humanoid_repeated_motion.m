%First Full Stride
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; 
%TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %two cycle to four cycle
%TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %four cycle to eight cycle
%TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %eight cycle to 16 cycles

x_result = [x_result;x_result(end)+x_result(2:end)];
%x_result = [x_result;x_result(end)+x_result(2:end)];
%x_result = [x_result;x_result(end)+x_result(2:end)];
%x_result = [x_result;x_result(end)+x_result(2:end)];

y_result = [y_result;y_result(2:end)];
%y_result = [y_result;y_result(2:end)];
%y_result = [y_result;y_result(2:end)];
%y_result = [y_result;y_result(2:end)];

theta_result = [theta_result;theta_result(2:end)];
%theta_result = [theta_result;theta_result(2:end)];
%theta_result = [theta_result;theta_result(2:end)];
%theta_result = [theta_result;theta_result(2:end)];

PFx_result_backup = PFx_result;
PHx_result_backup = PHx_result;
PFy_result_backup = PFy_result;
PHy_result_backup = PHy_result;
PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PHx_result_backup))];
%PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
%PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
%PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];

PFy_result = [PFy_result;PHy_result_backup(2:end)];
%PFy_result = [PFy_result;PFy_result(2:end)];
%PFy_result = [PFy_result;PFy_result(2:end)];
%PFy_result = [PFy_result;PFy_result(2:end)];

PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PFx_result_backup))];
%PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
%PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
%PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];

PHy_result = [PHy_result;PFy_result_backup(2:end)];
%PHy_result = [PHy_result;PHy_result(2:end)];
%PHy_result = [PHy_result;PHy_result(2:end)];
%PHy_result = [PHy_result;PHy_result(2:end)];

PFcenterX_result_world_backup = PFcenterX_result_world;
PHcenterX_result_world_backup = PHcenterX_result_world;
PFcenterY_result_world_backup = PFcenterY_result_world;
PHcenterY_result_world_backup = PHcenterY_result_world;
PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world_backup))];
%PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
%PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
%PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];

PFcenterY_result_world = [PFcenterY_result_world;PHcenterY_result_world_backup(2:end)];
%PFcenterY_result_world = [PFcenterY_result_world;PFcenterY_result_world(2:end)];
%PFcenterY_result_world = [PFcenterY_result_world;PFcenterY_result_world(2:end)];
%PFcenterY_result_world = [PFcenterY_result_world;PFcenterY_result_world(2:end)];

PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world_backup))];
%PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
%PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
%PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];

PHcenterY_result_world = [PHcenterY_result_world;PFcenterY_result_world_backup(2:end)];
%PHcenterY_result_world = [PHcenterY_result_world;PHcenterY_result_world(2:end)];
%PHcenterY_result_world = [PHcenterY_result_world;PHcenterY_result_world(2:end)];
%PHcenterY_result_world = [PHcenterY_result_world;PHcenterY_result_world(2:end)];

%one cycle to two cycle
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];
y_result = [y_result;y_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];
PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
PFy_result = [PFy_result;PFy_result(2:end)];
PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
PHy_result = [PHy_result;PHy_result(2:end)];
PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
PFcenterY_result_world = [PFcenterY_result_world;PFcenterY_result_world(2:end)];
PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
PHcenterY_result_world = [PHcenterY_result_world;PHcenterY_result_world(2:end)];

%two cycle to 4 cycles
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];
y_result = [y_result;y_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];
PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
PFy_result = [PFy_result;PFy_result(2:end)];
PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
PHy_result = [PHy_result;PHy_result(2:end)];
PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
PFcenterY_result_world = [PFcenterY_result_world;PFcenterY_result_world(2:end)];
PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
PHcenterY_result_world = [PHcenterY_result_world;PHcenterY_result_world(2:end)];

%four cycles to 8 cycles
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];
y_result = [y_result;y_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];
PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
PFy_result = [PFy_result;PFy_result(2:end)];
PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
PHy_result = [PHy_result;PHy_result(2:end)];
PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
PFcenterY_result_world = [PFcenterY_result_world;PFcenterY_result_world(2:end)];
PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
PHcenterY_result_world = [PHcenterY_result_world;PHcenterY_result_world(2:end)];

%8 cycles to 16 cycles
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];
y_result = [y_result;y_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];
PFx_result = [PFx_result;PFx_result(end)+cumsum(diff(PFx_result))];
PFy_result = [PFy_result;PFy_result(2:end)];
PHx_result = [PHx_result;PHx_result(end)+cumsum(diff(PHx_result))];
PHy_result = [PHy_result;PHy_result(2:end)];
PFcenterX_result_world = [PFcenterX_result_world;PFcenterX_result_world(end)+cumsum(diff(PFcenterX_result_world))];
PFcenterY_result_world = [PFcenterY_result_world;PFcenterY_result_world(2:end)];
PHcenterX_result_world = [PHcenterX_result_world;PHcenterX_result_world(end)+cumsum(diff(PHcenterX_result_world))];
PHcenterY_result_world = [PHcenterY_result_world;PHcenterY_result_world(2:end)];
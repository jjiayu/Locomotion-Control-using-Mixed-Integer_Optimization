TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %one cycle to two cycle
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %two cycle to four cycle
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %four cycle to eight cycle
TimeSeries = [TimeSeries;TimeSeries(end)+TimeSeries(2:end)]; %eight cycle to 16 cycles

x_result = [x_result;x_result(end)+x_result(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];
x_result = [x_result;x_result(end)+x_result(2:end)];

y_result = [y_result;y_result(2:end)];
y_result = [y_result;y_result(2:end)];
y_result = [y_result;y_result(2:end)];
y_result = [y_result;y_result(2:end)];

z_result = [z_result;z_result(2:end)];
z_result = [z_result;z_result(2:end)];
z_result = [z_result;z_result(2:end)];
z_result = [z_result;z_result(2:end)];

psi_result = [psi_result;psi_result(2:end)];
psi_result = [psi_result;psi_result(2:end)];
psi_result = [psi_result;psi_result(2:end)];
psi_result = [psi_result;psi_result(2:end)];

theta_result = [theta_result;theta_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];
theta_result = [theta_result;theta_result(2:end)];

phi_result = [phi_result;phi_result(2:end)];
phi_result = [phi_result;phi_result(2:end)];
phi_result = [phi_result;phi_result(2:end)];
phi_result = [phi_result;phi_result(2:end)];

Plfx_result = [Plfx_result;Plfx_result(end)+cumsum(diff(Plfx_result))];
Plfx_result = [Plfx_result;Plfx_result(end)+cumsum(diff(Plfx_result))];
Plfx_result = [Plfx_result;Plfx_result(end)+cumsum(diff(Plfx_result))];
Plfx_result = [Plfx_result;Plfx_result(end)+cumsum(diff(Plfx_result))];

Plfy_result = [Plfy_result;Plfy_result(2:end)];
Plfy_result = [Plfy_result;Plfy_result(2:end)];
Plfy_result = [Plfy_result;Plfy_result(2:end)];
Plfy_result = [Plfy_result;Plfy_result(2:end)];

Plfz_result = [Plfz_result;Plfz_result(2:end)];
Plfz_result = [Plfz_result;Plfz_result(2:end)];
Plfz_result = [Plfz_result;Plfz_result(2:end)];
Plfz_result = [Plfz_result;Plfz_result(2:end)];

Plhx_result = [Plhx_result;Plhx_result(end)+cumsum(diff(Plhx_result))];
Plhx_result = [Plhx_result;Plhx_result(end)+cumsum(diff(Plhx_result))];
Plhx_result = [Plhx_result;Plhx_result(end)+cumsum(diff(Plhx_result))];
Plhx_result = [Plhx_result;Plhx_result(end)+cumsum(diff(Plhx_result))];

Plhy_result = [Plhy_result;Plhy_result(2:end)];
Plhy_result = [Plhy_result;Plhy_result(2:end)];
Plhy_result = [Plhy_result;Plhy_result(2:end)];
Plhy_result = [Plhy_result;Plhy_result(2:end)];

Plhz_result = [Plhz_result;Plhz_result(2:end)];
Plhz_result = [Plhz_result;Plhz_result(2:end)];
Plhz_result = [Plhz_result;Plhz_result(2:end)];
Plhz_result = [Plhz_result;Plhz_result(2:end)];

Prfx_result = [Prfx_result;Prfx_result(end)+cumsum(diff(Prfx_result))];
Prfx_result = [Prfx_result;Prfx_result(end)+cumsum(diff(Prfx_result))];
Prfx_result = [Prfx_result;Prfx_result(end)+cumsum(diff(Prfx_result))];
Prfx_result = [Prfx_result;Prfx_result(end)+cumsum(diff(Prfx_result))];

Prfy_result = [Prfy_result;Prfy_result(2:end)];
Prfy_result = [Prfy_result;Prfy_result(2:end)];
Prfy_result = [Prfy_result;Prfy_result(2:end)];
Prfy_result = [Prfy_result;Prfy_result(2:end)];

Prfz_result = [Prfz_result;Prfz_result(2:end)];
Prfz_result = [Prfz_result;Prfz_result(2:end)];
Prfz_result = [Prfz_result;Prfz_result(2:end)];
Prfz_result = [Prfz_result;Prfz_result(2:end)];

Prhx_result = [Prhx_result;Prhx_result(end)+cumsum(diff(Prhx_result))];
Prhx_result = [Prhx_result;Prhx_result(end)+cumsum(diff(Prhx_result))];
Prhx_result = [Prhx_result;Prhx_result(end)+cumsum(diff(Prhx_result))];
Prhx_result = [Prhx_result;Prhx_result(end)+cumsum(diff(Prhx_result))];

Prhy_result = [Prhy_result;Prhy_result(2:end)];
Prhy_result = [Prhy_result;Prhy_result(2:end)];
Prhy_result = [Prhy_result;Prhy_result(2:end)];
Prhy_result = [Prhy_result;Prhy_result(2:end)];

Prhz_result = [Prhz_result;Prhz_result(2:end)];
Prhz_result = [Prhz_result;Prhz_result(2:end)];
Prhz_result = [Prhz_result;Prhz_result(2:end)];
Prhz_result = [Prhz_result;Prhz_result(2:end)];

PlfxCenter_result_world = [PlfxCenter_result_world;PlfxCenter_result_world(end)+cumsum(diff(PlfxCenter_result_world))];
PlfxCenter_result_world = [PlfxCenter_result_world;PlfxCenter_result_world(end)+cumsum(diff(PlfxCenter_result_world))];
PlfxCenter_result_world = [PlfxCenter_result_world;PlfxCenter_result_world(end)+cumsum(diff(PlfxCenter_result_world))];
PlfxCenter_result_world = [PlfxCenter_result_world;PlfxCenter_result_world(end)+cumsum(diff(PlfxCenter_result_world))];

PlfyCenter_result_world = [PlfyCenter_result_world;PlfyCenter_result_world(2:end)];
PlfyCenter_result_world = [PlfyCenter_result_world;PlfyCenter_result_world(2:end)];
PlfyCenter_result_world = [PlfyCenter_result_world;PlfyCenter_result_world(2:end)];
PlfyCenter_result_world = [PlfyCenter_result_world;PlfyCenter_result_world(2:end)];

PlfzCenter_result_world = [PlfzCenter_result_world;PlfzCenter_result_world(2:end)];
PlfzCenter_result_world = [PlfzCenter_result_world;PlfzCenter_result_world(2:end)];
PlfzCenter_result_world = [PlfzCenter_result_world;PlfzCenter_result_world(2:end)];
PlfzCenter_result_world = [PlfzCenter_result_world;PlfzCenter_result_world(2:end)];

PlhxCenter_result_world = [PlhxCenter_result_world;PlhxCenter_result_world(end)+cumsum(diff(PlhxCenter_result_world))];
PlhxCenter_result_world = [PlhxCenter_result_world;PlhxCenter_result_world(end)+cumsum(diff(PlhxCenter_result_world))];
PlhxCenter_result_world = [PlhxCenter_result_world;PlhxCenter_result_world(end)+cumsum(diff(PlhxCenter_result_world))];
PlhxCenter_result_world = [PlhxCenter_result_world;PlhxCenter_result_world(end)+cumsum(diff(PlhxCenter_result_world))];

PlhyCenter_result_world = [PlhyCenter_result_world;PlhyCenter_result_world(2:end)];
PlhyCenter_result_world = [PlhyCenter_result_world;PlhyCenter_result_world(2:end)];
PlhyCenter_result_world = [PlhyCenter_result_world;PlhyCenter_result_world(2:end)];
PlhyCenter_result_world = [PlhyCenter_result_world;PlhyCenter_result_world(2:end)];

PlhzCenter_result_world = [PlhzCenter_result_world;PlhzCenter_result_world(2:end)];
PlhzCenter_result_world = [PlhzCenter_result_world;PlhzCenter_result_world(2:end)];
PlhzCenter_result_world = [PlhzCenter_result_world;PlhzCenter_result_world(2:end)];
PlhzCenter_result_world = [PlhzCenter_result_world;PlhzCenter_result_world(2:end)];

PrfxCenter_result_world = [PrfxCenter_result_world;PrfxCenter_result_world(end)+cumsum(diff(PrfxCenter_result_world))];
PrfxCenter_result_world = [PrfxCenter_result_world;PrfxCenter_result_world(end)+cumsum(diff(PrfxCenter_result_world))];
PrfxCenter_result_world = [PrfxCenter_result_world;PrfxCenter_result_world(end)+cumsum(diff(PrfxCenter_result_world))];
PrfxCenter_result_world = [PrfxCenter_result_world;PrfxCenter_result_world(end)+cumsum(diff(PrfxCenter_result_world))];

PrfyCenter_result_world = [PrfyCenter_result_world;PrfyCenter_result_world(2:end)];
PrfyCenter_result_world = [PrfyCenter_result_world;PrfyCenter_result_world(2:end)];
PrfyCenter_result_world = [PrfyCenter_result_world;PrfyCenter_result_world(2:end)];
PrfyCenter_result_world = [PrfyCenter_result_world;PrfyCenter_result_world(2:end)];

PrfzCenter_result_world = [PrfzCenter_result_world;PrfzCenter_result_world(2:end)];
PrfzCenter_result_world = [PrfzCenter_result_world;PrfzCenter_result_world(2:end)];
PrfzCenter_result_world = [PrfzCenter_result_world;PrfzCenter_result_world(2:end)];
PrfzCenter_result_world = [PrfzCenter_result_world;PrfzCenter_result_world(2:end)];

PrhxCenter_result_world = [PrhxCenter_result_world;PrhxCenter_result_world(end)+cumsum(diff(PrhxCenter_result_world))];
PrhxCenter_result_world = [PrhxCenter_result_world;PrhxCenter_result_world(end)+cumsum(diff(PrhxCenter_result_world))];
PrhxCenter_result_world = [PrhxCenter_result_world;PrhxCenter_result_world(end)+cumsum(diff(PrhxCenter_result_world))];
PrhxCenter_result_world = [PrhxCenter_result_world;PrhxCenter_result_world(end)+cumsum(diff(PrhxCenter_result_world))];

PrhyCenter_result_world = [PrhyCenter_result_world;PrhyCenter_result_world(2:end)];
PrhyCenter_result_world = [PrhyCenter_result_world;PrhyCenter_result_world(2:end)];
PrhyCenter_result_world = [PrhyCenter_result_world;PrhyCenter_result_world(2:end)];
PrhyCenter_result_world = [PrhyCenter_result_world;PrhyCenter_result_world(2:end)];

PrhzCenter_result_world = [PrhzCenter_result_world;PrhzCenter_result_world(2:end)];
PrhzCenter_result_world = [PrhzCenter_result_world;PrhzCenter_result_world(2:end)];
PrhzCenter_result_world = [PrhzCenter_result_world;PrhzCenter_result_world(2:end)];
PrhzCenter_result_world = [PrhzCenter_result_world;PrhzCenter_result_world(2:end)];

NetForceX = [NetForceX;NetForceX];
NetForceX = [NetForceX;NetForceX];
NetForceX = [NetForceX;NetForceX];
NetForceX = [NetForceX;NetForceX];

NetForceY = [NetForceY;NetForceY];
NetForceY = [NetForceY;NetForceY];
NetForceY = [NetForceY;NetForceY];
NetForceY = [NetForceY;NetForceY];

NetForceZ = [NetForceZ;NetForceZ];
NetForceZ = [NetForceZ;NetForceZ];
NetForceZ = [NetForceZ;NetForceZ];
NetForceZ = [NetForceZ;NetForceZ];

NetTorqueX = [NetTorqueX;NetTorqueX];
NetTorqueX = [NetTorqueX;NetTorqueX];
NetTorqueX = [NetTorqueX;NetTorqueX];
NetTorqueX = [NetTorqueX;NetTorqueX];

NetTorqueY = [NetTorqueY;NetTorqueY];
NetTorqueY = [NetTorqueY;NetTorqueY];
NetTorqueY = [NetTorqueY;NetTorqueY];
NetTorqueY = [NetTorqueY;NetTorqueY];

NetTorqueZ = [NetTorqueZ;NetTorqueZ];
NetTorqueZ = [NetTorqueZ;NetTorqueZ];
NetTorqueZ = [NetTorqueZ;NetTorqueZ];
NetTorqueZ = [NetTorqueZ;NetTorqueZ];
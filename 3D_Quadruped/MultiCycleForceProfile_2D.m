Flfx_full_result = [];
Flfy_full_result = [];
Flfz_full_result = [];
Frfx_full_result = [];
Frfy_full_result = [];
Frfz_full_result = [];
Flhx_full_result = [];
Flhy_full_result = [];
Flhz_full_result = [];
Frhx_full_result = [];
Frhy_full_result = [];
Frhz_full_result = [];

for i = 1:16
    Flfx_full_result = [Flfx_full_result,FFx_result'];
    Flfy_full_result = [Flfy_full_result,zeros(size(FFx_result'))];
    Flfz_full_result = [Flfz_full_result,FFy_result'];
    Frfx_full_result = [Frfx_full_result,FFx_result'];
    Frfy_full_result = [Frfy_full_result,zeros(size(FFx_result'))];
    Frfz_full_result = [Frfz_full_result,FFy_result'];
    Flhx_full_result = [Flhx_full_result,FHx_result'];
    Flhy_full_result = [Flhy_full_result,zeros(size(FHx_result'))];
    Flhz_full_result = [Flhz_full_result,FHy_result'];
    Frhx_full_result = [Frhx_full_result,FHx_result'];
    Frhy_full_result = [Frhy_full_result,zeros(size(FFx_result'))];
    Frhz_full_result = [Frhz_full_result,FHy_result'];
end

    Flfx_full_result = [Flfx_full_result,FFx_result(1)];
    Flfy_full_result = [Flfy_full_result,0];
    Flfz_full_result = [Flfz_full_result,FFy_result(1)];
    Frfx_full_result = [Frfx_full_result,FFx_result(1)];
    Frfy_full_result = [Frfy_full_result,0];
    Frfz_full_result = [Frfz_full_result,FFy_result(1)];
    Flhx_full_result = [Flhx_full_result,FHx_result(1)];
    Flhy_full_result = [Flhy_full_result,0];
    Flhz_full_result = [Flhz_full_result,FHy_result(1)];
    Frhx_full_result = [Frhx_full_result,FHx_result(1)];
    Frhy_full_result = [Frhy_full_result,0];
    Frhz_full_result = [Frhz_full_result,FHy_result(1)];

ForceProfile = [Flfx_full_result;...
                Flfy_full_result;...
                Flfz_full_result;...
                Frfx_full_result;...
                Frfy_full_result;...
                Frfz_full_result;...
                Flhx_full_result;...
                Flhy_full_result;...
                Flhz_full_result;...
                Frhx_full_result;...
                Frhy_full_result;...
                Frhz_full_result];
            
writematrix(ForceProfile,'Forcetemp.csv')

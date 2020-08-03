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
    Flfx_full_result = [Flfx_full_result,Flfx_result'];
    Flfy_full_result = [Flfy_full_result,Flfy_result'];
    Flfz_full_result = [Flfz_full_result,Flfz_result'];
    Frfx_full_result = [Frfx_full_result,Frfx_result'];
    Frfy_full_result = [Frfy_full_result,Frfy_result'];
    Frfz_full_result = [Frfz_full_result,Frfz_result'];
    Flhx_full_result = [Flhx_full_result,Flhx_result'];
    Flhy_full_result = [Flhy_full_result,Flhy_result'];
    Flhz_full_result = [Flhz_full_result,Flhz_result'];
    Frhx_full_result = [Frhx_full_result,Frhx_result'];
    Frhy_full_result = [Frhy_full_result,Frhy_result'];
    Frhz_full_result = [Frhz_full_result,Frhz_result'];
end

Flfx_full_result = [Flfx_full_result,Flfx_result(1)];
Flfy_full_result = [Flfy_full_result,Flfy_result(1)];
Flfz_full_result = [Flfz_full_result,Flfz_result(1)];
Frfx_full_result = [Frfx_full_result,Frfx_result(1)];
Frfy_full_result = [Frfy_full_result,Frfy_result(1)];
Frfz_full_result = [Frfz_full_result,Frfz_result(1)];
Flhx_full_result = [Flhx_full_result,Flhx_result(1)];
Flhy_full_result = [Flhy_full_result,Flhy_result(1)];
Flhz_full_result = [Flhz_full_result,Flhz_result(1)];
Frhx_full_result = [Frhx_full_result,Frhx_result(1)];
Frhy_full_result = [Frhy_full_result,Frhy_result(1)];
Frhz_full_result = [Frhz_full_result,Frhz_result(1)];

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
            
writematrix(ForceProfile,'temp.csv')

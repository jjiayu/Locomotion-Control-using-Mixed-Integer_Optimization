function [nlcons] = nlconstraint(vars,TimeSeriesLength,names,m,h)
%NLCONSTRAINT: Non-Linear Constraint
%   System dynamics

% x-axis dynamics
% xdot(k+1) - xdot(k) - 1/2/m*h*FFx(k+1)*CF(k+1) - 1/2/m*h*FHx(k+1)*CH(k+1) - 1/2/m*h*FFx(k)*CF(k) - 1/2/m*h*FHx(k)*CH(k) = 0
x_dyn = zeros(TimeSeriesLength-1,1);
for k = 0:TimeSeriesLength-2
   x_dyn(k+1) = vars(find(names == strcat('xdot',num2str(k+1))))...xdot(k+1)
                - vars(find(names == strcat('xdot',num2str(k)))) ...-xdot(k)
                - 1/2/m*h*vars(find(names == strcat('FFx',num2str(k+1))))*vars(find(names == strcat('CF',num2str(k+1)))) ...- 1/2*h/m*FFx(k+1)*CF(k+1)
                - 1/2/m*h*vars(find(names == strcat('FHx',num2str(k+1))))*vars(find(names == strcat('CH',num2str(k+1)))) ...- 1/2*h/m*FHx(k+1)*CH(k+1)
                - 1/2/m*h*vars(find(names == strcat('FFx',num2str(k))))*vars(find(names == strcat('CF',num2str(k)))) ...- 1/2*h/m*FFx(k)*CF(k)
                - 1/2/m*h*vars(find(names == strcat('FHx',num2str(k))))*vars(find(names == strcat('CH',num2str(k))));  %- 1/2*h/m*FHx(k)*CH(k)
end

nlcons = [x_dyn;];

end

%lagecy code
% nlcons_CF = zeros(xdotLength,1);
% 
% for k = 1:xdotLength
%     nlcons_CF(k) = vars(find(names == strcat('CF',num2str(k-1))))*(1-vars(find(names == strcat('CF',num2str(k-1)))));
% end
% 
% nlcons_CH = zeros(xdotLength,1);
% 
% for k = 1:xdotLength
%     nlcons_CH(k) = vars(find(names == strcat('CH',num2str(k-1))))*(1-vars(find(names == strcat('CH',num2str(k-1)))));
% end
% 
% nlcons = [nlcons_CF;nlcons_CH];

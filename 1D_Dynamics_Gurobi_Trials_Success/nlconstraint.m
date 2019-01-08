function [nlcons] = nlconstraint(vars,xdotLength,names,m,h)
%NLCONSTRAINT: Non-Linear Constraint
%   System dynamics

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

for k = 0:xdotLength-2
   nlcons(k+1) = vars(find(names == strcat('xdot',num2str(k+1)))) - vars(find(names == strcat('xdot',num2str(k)))) - 1/2/m*h*vars(find(names == strcat('FFx',num2str(k+1))))*vars(find(names == strcat('CF',num2str(k+1)))) - 1/2/m*h*vars(find(names == strcat('FHx',num2str(k+1))))*vars(find(names == strcat('CH',num2str(k+1)))) - 1/2/m*h*vars(find(names == strcat('FFx',num2str(k))))*vars(find(names == strcat('CF',num2str(k)))) - 1/2/m*h*vars(find(names == strcat('FHx',num2str(k))))*vars(find(names == strcat('CH',num2str(k))));  
end

end
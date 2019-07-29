%Import Data First
StridePeriod = 1.0:0.1:1.8;
%StridePeriod = repelem(StridePeriod,2);
Speed = 2.1:-0.1:0.3;
StrideLengthMap = zeros(length(Speed),length(StridePeriod));

for i = 1:length(StridePeriod)
    for j = 1:length(Speed)
        StrideLengthMap(j,i) = StridePeriod(i)*Speed(j);
    end
end

figure()
hold on

for i = 1:length(StridePeriod)
for j = 1:length(Speed)
TempStridePeriod = StridePeriod(i);
TempStrideLength = StrideLengthMap(j,i);
TempSpeed = Speed(j);
Gait = data(j,i);
if contains(Gait,'Walking') == 1
walking_sample = [TempStridePeriod,TempSpeed];
%plot(TempStridePeriod,TempSpeed,'y-p','MarkerSize',30, 'LineWidth',3,'LineStyle','none')%,'MarkerFaceColor','yellow')
elseif contains(Gait,'Running') == 1
running_sample = [TempStridePeriod,TempSpeed];
%plot(TempStridePeriod,TempSpeed,'g-d','MarkerSize',25, 'LineWidth',3,'LineStyle','none')
%elseif contains(Gait,'Bounding') == 1
%plot(TempStridePeriod,TempSpeed,'r-s','MarkerSize',35, 'LineWidth',3)%,'MarkerFaceColor','red')
elseif contains(Gait,'Pronking') == 1
pronking_sample = [TempStridePeriod,TempSpeed];
%plot(TempStridePeriod,TempSpeed,'b-o','MarkerSize',15,'LineWidth',1,'MarkerFaceColor','blue','LineStyle','none')
elseif contains(Gait,'Bounding-S') == 1
bounding_s_sample = [TempStridePeriod,TempSpeed];
%plot(TempStridePeriod,TempSpeed,'r-s','MarkerSize',34,'LineWidth',4,'LineStyle','none')%,'MarkerFaceColor','blue')
elseif contains(Gait,'Bounding-D') == 1
bounding_d_sample = [TempStridePeriod,TempSpeed];
%plot(TempStridePeriod,TempSpeed,'d','MarkerSize',25,'LineWidth',1,'MarkerFaceColor','#EDB120', 'MarkerEdgeColor','#EDB120','LineStyle','none')
elseif contains(Gait,'Trotting') == 1
trotting_sample = [TempStridePeriod,TempSpeed];
%plot(TempStridePeriod,TempSpeed,'h','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')%,'MarkerFaceColor','blue')
end
end
end


for i = 1:length(StridePeriod)
for j = 1:length(Speed)
TempStridePeriod = StridePeriod(i);
TempStrideLength = StrideLengthMap(j,i);
TempSpeed = Speed(j);  '#7E2F8E'
Gait = data(j,i);
if contains(Gait,'Walking') == 1
plot(TempStridePeriod,TempSpeed,'y-*','MarkerSize',30, 'LineWidth',3,'LineStyle','none', 'MarkerEdgeColor', '#7E2F8E')%,'MarkerFaceColor','yellow')
elseif contains(Gait,'Running') == 1
plot(TempStridePeriod,TempSpeed,'g-+','MarkerSize',30, 'LineWidth',5,'LineStyle','none', 'MarkerEdgeColor', '#4DBEEE')
%elseif contains(Gait,'Bounding') == 1
%plot(TempStridePeriod,TempSpeed,'r-s','MarkerSize',35, 'LineWidth',3)%,'MarkerFaceColor','red')
elseif contains(Gait,'Pronking') == 1
plot(TempStridePeriod,TempSpeed,'b-o','MarkerSize',15,'LineWidth',1,'MarkerFaceColor','blue','LineStyle','none')
elseif contains(Gait,'Bounding-S') == 1
plot(TempStridePeriod,TempSpeed,'r-s','MarkerSize',34,'LineWidth',4,'LineStyle','none')%,'MarkerFaceColor','blue')
elseif contains(Gait,'Bounding-D') == 1
plot(TempStridePeriod,TempSpeed,'d','MarkerSize',25,'LineWidth',1,'MarkerFaceColor','#EDB120', 'MarkerEdgeColor','#EDB120','LineStyle','none')
elseif contains(Gait,'Trotting') == 1
plot(TempStridePeriod,TempSpeed,'h','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F')%,'MarkerFaceColor','blue')
end
end
end

p1 = plot(walking_sample(1),walking_sample(2),'y-*','MarkerSize',30, 'LineWidth',3,'LineStyle','none', 'MarkerEdgeColor', '#7E2F8E');
p2 = plot(running_sample(1),running_sample(2),'g-+','MarkerSize',30, 'LineWidth',5,'LineStyle','none', 'MarkerEdgeColor', '#4DBEEE');
%p3 = plot(bounding_d_sample(1),bounding_d_sample(2),'d','MarkerSize',25,'LineWidth',1,'MarkerFaceColor','#EDB120', 'MarkerEdgeColor','#EDB120','LineStyle','none');
%p4 = plot(trotting_sample(1),trotting_sample(2),'h','MarkerSize',30,'LineWidth',1,'LineStyle','none','MarkerFaceColor','#A2142F', 'MarkerEdgeColor','#A2142F');%,'MarkerFaceColor','blue')

legend([p1,p2],{'Walking','Running'})

xlim([0.95,1.85])
yticks([0.2:0.1:2.2])
set(gca,'FontSize',24)

xlabel('Stride Period (s)')
ylabel('Average Locomotion Speed (m/s)')


grid on

hold off

hold off
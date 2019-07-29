%Import Data First
StridePeriod = 0.3:0.1:0.9;
StridePeriod = repelem(StridePeriod,2);
Speed = 2.1:-0.1:0.3;
StrideLengthMap = zeros(length(Speed),length(StridePeriod));

for i = 1:length(StridePeriod)
    for j = 1:length(Speed)
        StrideLengthMap(j,i) = StridePeriod(i)*Speed(j);
    end
end

gallop_flag = 0;
walking_flag = 0;
bounding_flag = 0;
pronking_flag = 0;

figure()
hold on
for i = 1:length(StridePeriod)
for j = 1:length(Speed)
TempStridePeriod = StridePeriod(i);
TempStrideLength = StrideLengthMap(j,i);
TempSpeed = Speed(j);
Gait = data(j,i);
if contains(Gait,'Gallop') == 1
%plot(TempStridePeriod,TempSpeed,'m-p','MarkerSize',30, 'LineWidth',3, 'MarkerFaceColor','m')
gallop_sample = [TempStridePeriod,TempSpeed];
elseif contains(Gait,'Walking') == 1
%plot(TempStridePeriod,TempSpeed,'g-d','MarkerSize',25, 'LineWidth',4, 'DisplayName')
walking_sample = [TempStridePeriod,TempSpeed];
elseif contains(Gait,'Bounding') == 1
%plot(TempStridePeriod,TempSpeed,'r-s','MarkerSize',35, 'LineWidth',4, 'DisplayName')%,'MarkerFaceColor','red')
bounding_sample = [TempStridePeriod,TempSpeed];
elseif contains(Gait,'Pronking') == 1
%plot(TempStridePeriod,TempSpeed,'b-o','MarkerSize',15,'LineWidth',3,'MarkerFaceColor','blue')
pronking_sample = [TempStridePeriod,TempSpeed];
%elseif contains(Gait,'Bounding-S') == 1
%plot(TempStridePeriod,TempSpeed,'b-^','MarkerSize',15,'LineWidth',3)%,'MarkerFaceColor','blue')
%elseif contains(Gait,'Bounding-D') == 1
%plot(TempStridePeriod,TempSpeed,'b->','MarkerSize',15,'LineWidth',3)%,'MarkerFaceColor','blue')
%elseif contains(Gait,'Trotting') == 1
%plot(TempStridePeriod,TempSpeed,'b-h','MarkerSize',30,'LineWidth',3)%,'MarkerFaceColor','blue')
end
end
end


for i = 1:length(StridePeriod)
for j = 1:length(Speed)
TempStridePeriod = StridePeriod(i);
TempStrideLength = StrideLengthMap(j,i);
TempSpeed = Speed(j);
Gait = data(j,i);
if contains(Gait,'Gallop') == 1
plot(TempStridePeriod,TempSpeed,'m-p','MarkerSize',30, 'LineWidth',1, 'MarkerFaceColor','m', 'LineStyle','none')
%gallop_sample = [TempStridePeriod,TempSpeed];
elseif contains(Gait,'Walking') == 1
plot(TempStridePeriod,TempSpeed,'g-^','MarkerSize',30, 'LineWidth',4, 'LineStyle','none')
%walking_sample = [TempStridePeriod,TempSpeed];
elseif contains(Gait,'Bounding') == 1
plot(TempStridePeriod,TempSpeed,'r-s','MarkerSize',35, 'LineWidth',4, 'LineStyle','none')%,'MarkerFaceColor','red')
%bounding_sample = [TempStridePeriod,TempSpeed];
elseif contains(Gait,'Pronking') == 1
plot(TempStridePeriod,TempSpeed,'b-o','MarkerSize',15,'LineWidth',1,'MarkerFaceColor','blue', 'LineStyle','none')
%pronking_sample = [TempStridePeriod,TempSpeed];
%elseif contains(Gait,'Bounding-S') == 1
%plot(TempStridePeriod,TempSpeed,'b-^','MarkerSize',15,'LineWidth',3)%,'MarkerFaceColor','blue')
%elseif contains(Gait,'Bounding-D') == 1
%plot(TempStridePeriod,TempSpeed,'b->','MarkerSize',15,'LineWidth',3)%,'MarkerFaceColor','blue')
%elseif contains(Gait,'Trotting') == 1
%plot(TempStridePeriod,TempSpeed,'b-h','MarkerSize',30,'LineWidth',3)%,'MarkerFaceColor','blue')
end
end
end

p1 = plot(gallop_sample(1),gallop_sample(2),'m-p','MarkerSize',30, 'LineWidth',1, 'MarkerFaceColor','m', 'LineStyle','none');
p2 = plot(walking_sample(1),walking_sample(2),'g-^','MarkerSize',30, 'LineWidth',4, 'LineStyle','none');
p3 = plot(bounding_sample(1),bounding_sample(2),'r-s','MarkerSize',35, 'LineWidth',4, 'LineStyle','none');%,'MarkerFaceColor','red');
p4 = plot(pronking_sample(1),pronking_sample(2),'b-o','MarkerSize',15,'LineWidth',1,'MarkerFaceColor','blue', 'LineStyle','none');
legend([p1,p2,p3,p4],{'Galloping','Walking','Bounding-S','Pronking'})

xlim([0.25,0.95])
yticks([0.2:0.1:2.2])
set(gca,'FontSize',24)

xlabel('Stride Period (s)')
ylabel('Average Locomotion Speed (m/s)')

grid on

hold off
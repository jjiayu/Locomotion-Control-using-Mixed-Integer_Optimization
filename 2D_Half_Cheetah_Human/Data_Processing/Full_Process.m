run('Build_Gait_Mapping.m');

plot_flag = input('Decide if we plot the gait mapping or not: 1 -> Yes 2 -> No \n');
if plot_flag == 1
    run('Plot_Gait_Mapping.m')
end

database_flag = input('Decide if we build the database file in csv file: 1 -> Yes 2 -> No \n');
if database_flag == 1
    run('Build_CSV_DataBase.m')
end
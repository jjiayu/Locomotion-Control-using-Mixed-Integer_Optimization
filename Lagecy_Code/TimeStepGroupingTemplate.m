%Time Step Grouping (Each Phase has Fixed Number of Time Steps) Template

NumTimeSteps = 20;
NumPhases = 4;
h = 0.05;
TimeSeries = 0:h:h*NumTimeSteps;
TimeSeriesLength = length(TimeSeries);
NumLocalTimesteps = NumTimeSteps/NumPhases;

for k = 0:TimeSeriesLength - 2 %(0 to the last knot -1)
    disp([num2str(k),':', 'Phase ', num2str(floor(k/NumLocalTimesteps+1))])
end
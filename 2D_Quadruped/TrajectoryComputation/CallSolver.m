% Call the Solver to Solver the Optimization Problem

%   Display some Info
disp('===================================================')
disp('Optimization Started')
%   Assemble optimization problem definitions
prob = struct('f', J, 'x', DecisionVars, 'g', vertcat(g{:}));

%       Build Solver Option Structure
if strcmp(SolverSelected, 'knitro')
    solverOption = struct('mip_outinterval', 100,...      % (Log Output Frequency) Log Output per Nodes
                          'mip_heuristic',   0,...     %-1,let sover to select heuristics method, 0, disable heuristics
                          'mip_outlevel',    2,...      % Print accumulated time for every node.
                          'mip_selectrule',  3,...      % 
                          'mip_branchrule',  2,...      % MIP Branching rule The rule for selecting nodes 2 has the best performance
                          'mip_maxnodes',    NumMaxNodes);      % Max Number of Nodes wish to be explored
% elseif strcmp(SolverSelected, 'bonmin')
%     solverOption = struct('option_file_name', 'bonmin.opt');  
end

%   Construct Nonlinear Programming Function Object
solver = nlpsol('solver', SolverSelected, prob, struct('discrete', varstype, SolverSelected, solverOption));

%   Solver the Problem
sol = solver('x0',  DecisionVarsInit, ...
             'lbx', lb_DecisionVars,...
             'ubx', ub_DecisionVars,...
             'lbg', lbg,...
             'ubg', ubg);
return_status = solver.stats();
return_status.success 

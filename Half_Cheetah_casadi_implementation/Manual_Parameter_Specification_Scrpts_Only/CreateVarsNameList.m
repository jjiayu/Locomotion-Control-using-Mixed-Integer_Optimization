function nameList = CreateVarsNameList(varsCasADi)
%CREATEVARSNAMELIST Create Variables Name List defined on every
%Knots/Discretization
%   INPUT:  variable vector and list defined in CasADi 
%   OUTPUT: Variable Name List Defined at Every Knot/Discretization
    
    nameList = strings(1,varsCasADi.length());
    
    for i = 1:varsCasADi.length()
        temp = varsCasADi(i);
        nameList(i) = temp.name();
    end

end


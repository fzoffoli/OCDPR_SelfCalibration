%@brief This function modify output data to show only teiw analysis
%
%@return posCardouSens, rotCardouSens, pose, only inside teiw
function [x,y,z,criteria,criteria_ind] = TeiwOutManipulation(x,y,z,criteria,out)

x = x(out.teiw~=0);
y = y(out.teiw~=0);
z = z(out.teiw~=0);
criteria = criteria(out.teiw~=0);
[~,criteria_ind] = sort(criteria);
end
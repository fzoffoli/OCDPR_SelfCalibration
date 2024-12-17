%@brief This function modify output data to show only criteria analysis
%
%@return 
function [x,y,z,criteria_out] = CriteriaOutManipulation(x,y,z,criteria_out,criteria_in)

x = x(criteria_in~=0);
y = y(criteria_in~=0);
z = z(criteria_in~=0);
criteria_out = criteria_out(criteria_in~=0);
end
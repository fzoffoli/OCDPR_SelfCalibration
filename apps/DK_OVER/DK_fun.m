function constraint = DK_fun(p,l,cdpr_parameters,cdpr_variables)

p(2) = 0;
p(4) = 0;
p(6) = 0;
[~,constraint] = CalcKinZeroOrdConstr(p(1:3),p(4:6),l,...
  cdpr_parameters,cdpr_variables);

end
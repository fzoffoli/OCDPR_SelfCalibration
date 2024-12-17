function V = CalcInstallationSpace(cdpr_parameters)

eps_xy=0.2; % divide the minor and major basis of the trapezoid
for i=1:cdpr_parameters.n_cables
x_lims(i)=cdpr_parameters.cable(i).pos_OD_glob(1);
y_lims(i)=cdpr_parameters.cable(i).pos_OD_glob(2);
z_lims(i)=cdpr_parameters.cable(i).pos_OD_glob(3);

x_loc_lims(i)=cdpr_parameters.cable(i).pos_PA_loc(1);
y_loc_lims(i)=cdpr_parameters.cable(i).pos_PA_loc(2);
z_loc_lims(i)=cdpr_parameters.cable(i).pos_PA_loc(3);
end

%%% local points storage
x_loc=max(abs(x_loc_lims));
y_loc=max(abs(y_loc_lims));
z_loc=max(abs(z_loc_lims));

%%% global limits storage
z_max=max(z_lims)-z_loc;
z_min=min(z_lims)+z_loc;
x_Max=max(x_lims)-x_loc;
x_Min=min(x_lims)+x_loc;
x_max=max(x_lims(x_lims<max(x_lims)-eps_xy))-x_loc;
x_min=min(x_lims(x_lims>min(x_lims)+eps_xy))+x_loc;
y_Max=max(y_lims)-y_loc;
y_Min=min(y_lims)+y_loc;
y_max=max(y_lims(y_lims<max(y_lims)-eps_xy))-y_loc;
y_min=min(y_lims(y_lims>min(y_lims)+eps_xy))+y_loc;

%%% do the attach points shape a trapezoid?
if (x_min<x_max)
    Sb=(x_max-x_min)*(y_max-y_min);
    SB=(x_Max-x_Min)*(y_Max-y_Min);
    V=(Sb+SB+sqrt(SB*Sb))*(z_max-z_min)/3;
    
    x=[x_min x_max x_min x_max x_Max x_Min x_Min x_Max];
    y=[y_min y_min y_max y_max y_Max y_Max y_Min y_Min];
    z=[z_min z_min z_min z_min z_max z_max z_max z_max];
else
    V=(x_Max-x_Min)*(y_Max-y_Min)*(z_max-z_min);

    x=[x_Max x_Max x_Max x_Max x_Min x_Min x_Min x_Min];
    y=[y_Min y_Max y_Max y_Min y_Max y_Min y_Max y_Min];
    z=[z_min z_min z_max z_max z_max z_max z_min z_min];
end


%uncomment for plot limits
scatter3(x,y,z);
end
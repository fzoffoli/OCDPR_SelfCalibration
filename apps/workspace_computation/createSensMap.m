function crep=createSensMap(cdpr_p,cdpr_v,crit,ws_info)
if ws_info.display_criteria == DisplayCriteria.TENSION_SENSITIVITYmultiplicity
    cmap = [0,0,1;%	Blue
    1.0000 ,0.4980, 0.3137	;%	coral1,0.1,0
    1,0,0;%	Red
    	0,1,1;%	Cyan
        	1,1,0;%	Yellow
    	0,0,0.5020	;%	Navy
    	0,0.498039215686275,1	;%	SlateBlue
       0,0.5020,0.5020 %Teal
        0.5294,0.8078,0.9216; %SkyBlue
    %yellow-brown-orange
    
    	0.956862745098039,0.643137254901961,0.376470588235294	;%	SandyBrown
    	0.545098039215686,0.270588235294118,0.0745098039215686	;%	SaddleBrown
    	0.556862745098039,0.419607843137255,0.137254901960784	;%	Sienna
    	1,0.843137254901961,0	;%	gold1 light
    	0.803921568627451,0.678431372549020,0	;%	gold3 ocra
       1,0.3,0  %orange
    	
    %greens
    	0,1,0;%	Green
    	0.419607843137255,0.556862745098039,0.137254901960784	;%	OliveDrab
    	0.635294117647059,0.803921568627451,0.352941176470588	;%	DarkOliveGreen3   
    	0,0.803921568627451,0	;%	green3
       0,0.5,0 ;%deepgreen
         0 ,1.0000, 0.7; %sprinGgreen
       0.6039,0.8039,0.1961; %yellowGreen
    %violets and reds
    	
    	1,0,1;%	Magenta
    	0.800000000000000,0.196078431372549,0.600000000000000	;%	Violet,Red
    	0.545098039215686,0,0.545098039215686	;%	magenta4
    	0.600000000000000,0.196078431372549,0.800000000000000	;%	DarkOrchid
    	0.941176470588235,0.501960784313726,0.501960784313726	;%	LightCoral/rosa carne
    0.8627 ,0.0784,   0.2353]; %Crimson];    
else
cmap = [0,0,1;%	Blue
    	0,1,1;%	Cyan
    	0,0,0.5020	;%	Navy
    	0,0.498039215686275,1	;%	SlateBlue
       0,0.5020,0.5020 %Teal
        0.5294,0.8078,0.9216; %SkyBlue
    %yellow-brown-orange
    	1,1,0;%	Yellow
    	0.956862745098039,0.643137254901961,0.376470588235294	;%	SandyBrown
    	0.545098039215686,0.270588235294118,0.0745098039215686	;%	SaddleBrown
    	0.556862745098039,0.419607843137255,0.137254901960784	;%	Sienna
    	1,0.843137254901961,0	;%	gold1 light
    	0.803921568627451,0.678431372549020,0	;%	gold3 ocra
       1,0.3,0  %orange
    	1.0000 ,0.4980, 0.3137	;%	coral1,0.1,0
    %greens
    	0,1,0;%	Green
    	0.419607843137255,0.556862745098039,0.137254901960784	;%	OliveDrab
    	0.635294117647059,0.803921568627451,0.352941176470588	;%	DarkOliveGreen3   
    	0,0.803921568627451,0	;%	green3
       0,0.5,0 ;%deepgreen
         0 ,1.0000, 0.7; %sprinGgreen
       0.6039,0.8039,0.1961; %yellowGreen
    %violets and reds
    	1,0,0;%	Red
    	1,0,1;%	Magenta
    	0.800000000000000,0.196078431372549,0.600000000000000	;%	Violet,Red
    	0.545098039215686,0,0.545098039215686	;%	magenta4
    	0.600000000000000,0.196078431372549,0.800000000000000	;%	DarkOrchid
    	0.941176470588235,0.501960784313726,0.501960784313726	;%	LightCoral/rosa carne
    0.8627 ,0.0784,   0.2353]; %Crimson];
end
crep=[];
ij=1;
com=nchoosek(length(cdpr_v.cable),(cdpr_p.n_cables-cdpr_p.pose_dim));
for i=1:com
   colour_ind(ij)=nnz(crit==i);
   crep=[crep; repmat(cmap(i,:),colour_ind(ij),1)];
   ij=ij+1;
end


function legendCables8
                        
figure()
plot(ones(1,10),'Linewidth',4,'Color', 'b');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '1.0000 ,0.4980, 0.3137');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '1,0,0');
hold on
plot(ones(1,10),'Linewidth',4,'Color', 'c');
hold on
plot(ones(1,10),'Linewidth',4,'Color', 'y');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0,0,0.5020');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0,0.498039215686275,1');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0,0.5020,0.5020');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.5294,0.8078,0.9216;');

hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.956862745098039,0.643137254901961,0.376470588235294');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.545098039215686,0.270588235294118,0.0745098039215686');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.556862745098039,0.419607843137255,0.137254901960784');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '1,0.843137254901961,0');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.803921568627451,0.678431372549020,0');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '1,0.3,0');

hold on
plot(ones(1,10),'Linewidth',4,'Color', 'g');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.419607843137255,0.556862745098039,0.137254901960784');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.635294117647059,0.803921568627451,0.352941176470588');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0,0.803921568627451,0');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0,0.5,0');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0 ,1.0000, 0.7');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.6039,0.8039,0.1961');

hold on
plot(ones(1,10),'Linewidth',4,'Color', '1,0,1');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.800000000000000,0.196078431372549,0.600000000000000');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.545098039215686,0,0.545098039215686');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.600000000000000,0.196078431372549,0.800000000000000');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.941176470588235,0.501960784313726,0.501960784313726');
hold on
plot(ones(1,10),'Linewidth',4,'Color', '0.8627 ,0.0784,   0.2353');
hold on

axis equal
axis([0 0.5 0 0.5])
leg1=legend('show');
title(leg1,'Cable pairs')
legend('1 2','1 3','1 4','1 5','1 6','1 7','1 8','2 3','2 4','2 5','2 6','2 7','2 8','3 4','3 5','3 6','3 7','3 8','4 5','4 6','4 7','4 8','5 6','5 7','5 8','6 7','6 8','7 8')

% figure()
% plot(ones(1,10),'Linewidth',4,'Color', 'b');
% hold on
% plot(ones(1,10),'Linewidth',4,'Color', CustomColors('Coral'));
% hold on
% plot(ones(1,10),'Linewidth',4,'Color', 'r');
% hold on
% plot(ones(1,10),'Linewidth',4,'Color', 'k');
% hold on
% axis equal
% axis([0 0.5 0 0.5])
% leg2=legend('show');
% title(leg2,'Multiplicity')
% legend('1','2','3','more')
end
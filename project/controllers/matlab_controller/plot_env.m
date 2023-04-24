function plots = plot_env(traj, params)
figure(1);
clf;
xlim([-110, 110]);
ylim([-110, 110]);
set(gca,'XTickLabel',[]);
set(gca,'YTickLabel',[]);
x0=10;
y0=10;
width=1000;
height=1000;
set(gcf,'position',[x0,y0,width,height])

% Create obstacle 1
rectangle('Position',[params.obsX1 params.obsY1 params.obswidth1 params.obsheight1],...
    'EdgeColor','r','FaceColor',[1 0 0],'Curvature',0.1);
hold on;
text(params.obsX1+(params.obswidth1)/2,params.obsY1+(params.obsheight1)/2,'B1','FontSize',18)

% Create obstacle 2
% rectangle('Position',[params.obscenX-params.obsR params.obscenY-params.obsR ...
%     2*params.obsR 2*params.obsR],...
%     'EdgeColor','r','FaceColor',[1 0 0],'Curvature',[1 1]);
% text(params.obscenX,params.obscenY,'B2','FontSize',18)

% Create obstacle 1
rectangle('Position',[params.obsX2 params.obsY2 params.obswidth2 params.obsheight2],...
    'EdgeColor','r','FaceColor',[1 0 0],'Curvature',0.1);
hold on;
text(params.obsX2+(params.obswidth2)/2,params.obsY2+(params.obsheight2)/2,'B2','FontSize',18)

% Create goal point
viscircles([params.goalX, params.goalY], [params.goalR], 'color', 'g');
rectangle('Position',[params.goalX-params.goalR params.goalY-params.goalR ...
    2*params.goalR 2*params.goalR],...
    'EdgeColor','k','FaceColor',[0 1 0],'Curvature',[1 1]);

text(params.goalX,params.goalY+0.5,'Goal','FontSize',16)

if ~isempty(traj)
    % initial state
    xinit = traj(:,1);
    % Create initial state
    viscircles(xinit(1:2,:)', [0.1], 'color', 'b');
%     xline(-110);
%     xline(-92);
%     xline(-54.5);
%     xline(-33);
%     xline(33);
%     xline(54.5);
%     xline(92);
%     xline(110);
%     
%     yline(-110);
%     yline(-92);
%     yline(-54.5);
%     yline(-33);
%     yline(33);
%     yline(54.5);
%     yline(92);
%     yline(110);
%     
    rectangle('Position',[-54.5 -54.5 164.5 164.5],'Curvature',0.1);
    rectangle('Position',[-33 -33 125 125],'Curvature',0.1);

    rectangle('Position',[-110 -110 164.5 164.5],'Curvature',0.1);
    rectangle('Position',[-92 -92 125 125],'Curvature',0.1);

    
    
    hold on;
    plot(traj(1, :), traj(2, :), 'color', 'b', 'LineWidth', 2);
    hold off;
end
axis square;
box on
grid on
plots = 0;


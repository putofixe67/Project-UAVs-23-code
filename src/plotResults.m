function plotResults(models, t, ref)

nModels = numel(models);

COL = reshape([models.color], nModels, [])'; % fallback-safe

disp('==========================================');

if isfield(ref,'v') && ~isempty(ref.v)
    disp("With desired velocity");
else
    disp("Without desired velocity");
end

if isfield(ref,'a') && ~isempty(ref.a)
    disp("With desired acceleration (feedforward)");
else
    disp("Without desired acceleration (feedforward)");
end

disp('==========================================');

% ==================================================
%               Figura 1: 3D Trajectory
% ==================================================
fig1 = figure('Name','3D Trajectory','Color','w');
ax1 = axes(fig1); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');

plot3(ax1, ref.p(:,1), ref.p(:,2), ref.p(:,3), 'k--', ...
    'LineWidth',1.5,'DisplayName','Reference');

for k = 1:nModels
    plot3(ax1, models(k).x(:,1), models(k).x(:,2), models(k).x(:,3), ...
        'Color', models(k).color, ...
        'LineWidth',1.8, ...
        'DisplayName', models(k).name);
end

view(ax1,30,30);
legend(ax1,'Location','best');
xlabel(ax1,'$p_x$ [m]');
ylabel(ax1,'$p_y$ [m]');
zlabel(ax1,'$p_z$ [m]');
title(ax1,'3D Trajectory Comparison');

% ==================================================
%           Figura 2: Position Comparison
% ==================================================
fig2 = figure('Name','Position States','Color','w');
tlo1 = tiledlayout(fig2,3,1,'TileSpacing','compact');

p_labels = {'$p_x$','$p_y$','$p_z$'};

for i = 1:3
    ax = nexttile(tlo1);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');

    plot(ax, t, ref.p(:,i), 'k:', 'LineWidth',1.5,'DisplayName','Reference');

    for k = 1:nModels
        plot(ax, t, models(k).x(:,i), ...
            'Color', models(k).color, ...
            'LineWidth',1.5, ...
            'DisplayName', models(k).name);
    end

    ylabel(ax,p_labels{i});

    if i==1
        title(ax,'Position States');
        legend(ax,'Location','best');
    end
    if i==3
        xlabel(ax,'$t$ [s]');
    end
end

% ==================================================
% Figura 3: Velocity Comparison
% ==================================================
fig3 = figure('Name','Velocity States','Color','w');
tlo2 = tiledlayout(fig3,3,1,'TileSpacing','compact');

v_labels = {'$v_x$','$v_y$','$v_z$'};

for i = 1:3
    ax = nexttile(tlo2);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');

    if isfield(ref,'v')
        plot(ax, t, ref.v(:,i), 'k:', 'LineWidth',1.5,'DisplayName','Reference');
    end

    for k = 1:nModels
        plot(ax, t, models(k).x(:,i+3), ...
            'Color', models(k).color, ...
            'LineWidth',1.5, ...
            'DisplayName', models(k).name);
    end

    ylabel(ax,v_labels{i});

    if i==1
        title(ax,'Velocity States');
        legend(ax,'Location','best');
    end
    if i==3
        xlabel(ax,'$t$ [s]');
    end
end

% ==================================================
%           Figura 4: Actuation Inputs
% ==================================================
fig4 = figure('Name','Actuation Inputs','Color','w');
tlo3 = tiledlayout(fig4,1,3,'TileSpacing','compact');

u_labels = {'$u_1$','$u_2$','$u_3$'};
    
    for i = 1:3
        ax = nexttile(tlo3);
        hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        
        for k = 1:nModels
        plot(ax, t, models(k).u(i,:), ...
            'Color', models(k).color, ...
            'LineWidth',1.5, ...
            'DisplayName', models(k).name);
        end
        
        ylabel(ax,u_labels{i});
        xlabel(ax,'$t$ [s]');
        
        if i==2
        title(ax,'Control Inputs');
        end
        
        if i==1
        legend(ax,'Location','best');
        end
    end

% ==================================================
%           Figura 5: Position error plot
% ==================================================
fig5 = figure('Name','Position Error','Color','w');
tlo = tiledlayout(fig5,3,1,'TileSpacing','compact');

labels = {'$p_x$ error','$p_y$ error','$p_z$ error'};

for i = 1:3

    ax = nexttile(tlo);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');

    % reference line (zero error)
    plot(ax, t, zeros(size(t)), 'k--', 'LineWidth', 1.2, ...
        'DisplayName','Zero error');

    for k = 1:numel(models)

        % error
        e = models(k).x(:,i) - ref.p(:,i);

        % mean error (optional)
        plot(ax, t, e, ...
            'Color', models(k).color, ...
            'LineWidth', 1.2, ...
            'DisplayName', [models(k).name ' error']);
    end

    ylabel(ax, labels{i});

    if i == 1
        title(ax,'Position Error');
        legend(ax,'Location','best');
    end

    if i == 3
        xlabel(ax,'$t$ [s]');
    end
end

end
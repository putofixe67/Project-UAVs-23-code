function plotResults(models, t, ref, graphList, outputDir)
if nargin < 5 || isempty(outputDir)
    outputDir = '';
elseif ~isfolder(outputDir)
    mkdir(outputDir);
end

nModels = numel(models);
sys = models(1).sysConsts;
m   = sys(1);
g   = sys(2); %#ok<NASGU>

disp('==========================================');
if isfield(ref,'v') && ~isempty(ref.v), disp("With desired velocity");
else, disp("Without desired velocity"); end
if isfield(ref,'a') && ~isempty(ref.a), disp("With desired acceleration (feedforward)");
else, disp("Without desired acceleration (feedforward)"); end
disp('==========================================');

for gi = 1:length(graphList)
    graphName = string(graphList(gi));

    %% ==========================================================
    %                       POSITION
    % ===========================================================
    if strcmpi(graphName, "Position")

        fig1 = figure('Name','3D Trajectory','Color','w');
        ax1 = axes(fig1);
        hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
        plot3(ax1, ref.p(:,1), ref.p(:,2), ref.p(:,3), ...
            'k--','LineWidth',1.5,'DisplayName','Reference');
        for k = 1:nModels
            plot3(ax1, models(k).x(:,1), models(k).x(:,2), models(k).x(:,3), ...
                'Color',models(k).color,'LineWidth',1.8,'DisplayName',models(k).name);
        end
        view(ax1,30,30); legend(ax1,'Location','best');
        xlabel(ax1,'$p_x$ [m]','Interpreter','latex');
        ylabel(ax1,'$p_y$ [m]','Interpreter','latex');
        zlabel(ax1,'$p_z$ [m]','Interpreter','latex');
        title(ax1,'3D Trajectory Comparison');

        fig2 = figure('Name','Position States','Color','w');
        tlo = tiledlayout(fig2,3,1,'TileSpacing','compact');
        p_labels = {'$p_x$ [m]','$p_y$ [m]','$p_z$ [m]'};
        for j = 1:3
            ax = nexttile(tlo);
            hold(ax,'on'); grid(ax,'on'); box(ax,'on');
            plot(ax, t, ref.p(:,j), 'k:','LineWidth',1.5,'DisplayName','Reference');
            for k = 1:nModels
                plot(ax, t, models(k).x(:,j), 'Color',models(k).color, ...
                    'LineWidth',1.5,'DisplayName',models(k).name);
            end
            ylabel(ax, p_labels{j}, 'Interpreter','latex');
            if j == 1, title(ax,'Position States'); legend(ax,'Location','best'); end
            if j == 3, xlabel(ax,'$t$ [s]','Interpreter','latex'); end
        end

        if ~isempty(outputDir)
            exportgraphics(fig1, fullfile(outputDir,'3D_Trajectory.png'), 'Resolution',300);
            exportgraphics(fig2, fullfile(outputDir,'Position_States.png'), 'Resolution',300);
        end
    end

    %% ==========================================================
    %                       VELOCITY
    % ===========================================================
    if strcmpi(graphName, "Velocity")

        fig3 = figure('Name','Velocity States','Color','w');
        tlo = tiledlayout(fig3,3,1,'TileSpacing','compact');
        v_labels = {'$v_x$ [m/s]','$v_y$ [m/s]','$v_z$ [m/s]'};
        for j = 1:3
            ax = nexttile(tlo);
            hold(ax,'on'); grid(ax,'on'); box(ax,'on');
            if isfield(ref,'v') && ~isempty(ref.v)
                plot(ax, t, ref.v(:,j), 'k:','LineWidth',1.5,'DisplayName','Reference');
            end
            for k = 1:nModels
                plot(ax, t, models(k).x(:,j+3), 'Color',models(k).color, ...
                    'LineWidth',1.5,'DisplayName',models(k).name);
            end
            ylabel(ax, v_labels{j}, 'Interpreter','latex');
            if j == 1, title(ax,'Velocity States'); legend(ax,'Location','best'); end
            if j == 3, xlabel(ax,'$t$ [s]','Interpreter','latex'); end
        end

        if ~isempty(outputDir)
            exportgraphics(fig3, fullfile(outputDir,'Velocity_States.png'), 'Resolution',300);
        end
    end

    %% ==========================================================
    %                    CONTROL INPUTS
    % ===========================================================
    if strcmpi(graphName, "Inputs")

        fig4 = figure('Name','Actuation Inputs','Color','w');
        tlo = tiledlayout(fig4,1,3,'TileSpacing','compact');
        u_labels = {'$\theta$ [deg]','$\phi$ [deg]','$T$ [N]'};
        u_sat    = [10, 10, 0.588];

        for j = 1:3
            ax = nexttile(tlo);
            hold(ax,'on'); grid(ax,'on'); box(ax,'on');

            for k = 1:nModels
                uF = models(k).u;
                if strcmpi(models(k).name, "Lyapunov")
                    a_vec = uF + [0; 0; m*g];
                    T     = vecnorm(a_vec, 2, 1);
                    b3    = a_vec ./ T;
                    theta = 180/pi * atan2(-b3(1,:), b3(3,:));
                    phi   = 180/pi * atan2( b3(2,:), b3(3,:));
                    u_plot = [theta; phi; T];
                else
                    T = uF(3,:) + m*g;
                    u_plot = [180/pi*(-uF(2,:)./T); 180/pi*(uF(1,:)./T); T];
                end
                plot(ax, t, u_plot(j,:), 'Color',models(k).color, ...
                    'LineWidth',1.5,'DisplayName',models(k).name);
            end

            if j == 1 || j == 2
                yline(ax,  40,'--r','LineWidth',1.5,'DisplayName','Physical limit');
                yline(ax, -40,'--r','LineWidth',1.5,'HandleVisibility','off');
            end
            yline(ax,  u_sat(j),'--k','LineWidth',1.5,'DisplayName','Model limit');
            yline(ax, -u_sat(j),'--k','LineWidth',1.5,'HandleVisibility','off');

            ylabel(ax, u_labels{j}, 'Interpreter','latex');
            xlabel(ax, '$t$ [s]',   'Interpreter','latex');
            if j == 2, title(ax,'Control Inputs'); end
            if j == 1, legend(ax,'Location','best'); end
        end

        if ~isempty(outputDir)
            exportgraphics(fig4, fullfile(outputDir,'Actuation_Inputs.png'), 'Resolution',300);
        end
    end

    %% ==========================================================
    %                    POSITION ERROR
    % ===========================================================
    if strcmpi(graphName, "PositionError")

        fig5 = figure('Name','Position Error','Color','w');
        tlo = tiledlayout(fig5,3,1,'TileSpacing','compact');
        labels = {'$p_x$ error [m]','$p_y$ error [m]','$p_z$ error [m]'};

        for j = 1:3
            ax = nexttile(tlo);
            hold(ax,'on'); grid(ax,'on'); box(ax,'on');
            plot(ax, t, zeros(size(t)), 'k--','LineWidth',1.2,'DisplayName','Zero error');
            for k = 1:nModels
                e     = models(k).x(:,j) - ref.p(:,j);
                bound = 3 * std(e);
                plot(ax, t, e, 'Color',models(k).color,'LineWidth',1.2, ...
                    'DisplayName',[models(k).name ' error']);
                yline(ax,  bound,'--','Color',models(k).color,'LineWidth',1.0,'HandleVisibility','off');
                yline(ax, -bound,'--','Color',models(k).color,'LineWidth',1.0,'HandleVisibility','off');
            end
            ylabel(ax, labels{j}, 'Interpreter','latex');
            if j == 1, title(ax,'Position Error'); legend(ax,'Location','best'); end
            if j == 3, xlabel(ax,'$t$ [s]','Interpreter','latex'); end
        end

        if ~isempty(outputDir)
            exportgraphics(fig5, fullfile(outputDir,'Position_Error.png'), 'Resolution',300);
        end
    end

    %% ==========================================================
    %                    VELOCITY ERROR
    % ===========================================================
    if strcmpi(graphName, "VelocityError")

        fig6 = figure('Name','Velocity Error','Color','w');
        tlo = tiledlayout(fig6,3,1,'TileSpacing','compact');
        labels = {'$v_x$ error [m/s]','$v_y$ error [m/s]','$v_z$ error [m/s]'};

        for j = 1:3
            ax = nexttile(tlo);
            hold(ax,'on'); grid(ax,'on'); box(ax,'on');
            plot(ax, t, zeros(size(t)), 'k--','LineWidth',1.2,'DisplayName','Zero error');
            for k = 1:nModels
                ev    = models(k).x(:,j+3) - ref.v(:,j);
                bound = 3 * std(ev);
                plot(ax, t, ev, 'Color',models(k).color,'LineWidth',1.2, ...
                    'DisplayName',[models(k).name ' error']);
                yline(ax,  bound,'--','Color',models(k).color,'LineWidth',1.0,'HandleVisibility','off');
                yline(ax, -bound,'--','Color',models(k).color,'LineWidth',1.0,'HandleVisibility','off');
            end
            ylabel(ax, labels{j}, 'Interpreter','latex');
            if j == 1, title(ax,'Velocity Error'); legend(ax,'Location','best'); end
            if j == 3, xlabel(ax,'$t$ [s]','Interpreter','latex'); end
        end

        if ~isempty(outputDir)
            exportgraphics(fig6, fullfile(outputDir,'Velocity_Error.png'), 'Resolution',300);
        end
    end

    %% ==========================================================
    %              STATE ERROR NORM (POSITION + VELOCITY)
    % ===========================================================
    if strcmpi(graphName, "ErrorNorm")

        fig7 = figure('Name','State Error Norm','Color','w');
        ax = axes(fig7);
        hold(ax,'on'); grid(ax,'on'); box(ax,'on');

        for k = 1:nModels
            ep = models(k).x(:,1:3) - ref.p;
            if isfield(ref,'v') && ~isempty(ref.v)
                e = [ep, models(k).x(:,4:6) - ref.v];
            else
                e = ep;
            end
            plot(ax, t, vecnorm(e,2,2), 'Color',models(k).color, ...
                'LineWidth',1.8,'DisplayName',models(k).name);
        end

        xlabel(ax,'$t$ [s]','Interpreter','latex');
        ylabel(ax,'$\|e\|$','Interpreter','latex');
        if isfield(ref,'v') && ~isempty(ref.v)
            title(ax,'State Error Norm (Position + Velocity)');
        else
            title(ax,'Position Error Norm');
        end
        legend(ax,'Location','best');

        if ~isempty(outputDir)
            exportgraphics(fig7, fullfile(outputDir,'Error_Norm.png'), 'Resolution',300);
        end
    end

end % for gi

%% Performance metrics (console only)
for k = 1:nModels
    ep = models(k).x(:,1:3) - ref.p;
    if isfield(ref,'v') && ~isempty(ref.v)
        e_norm = vecnorm([ep, models(k).x(:,4:6) - ref.v], 2, 2);
    else
        e_norm = vecnorm(ep, 2, 2);
    end
    fprintf('\n--- %s ---\n', models(k).name);
    fprintf('RMSE = %.4f\n', sqrt(mean(e_norm.^2)));
    fprintf('ISE  = %.4f\n', trapz(t, e_norm.^2));
    fprintf('ITAE = %.4f\n', trapz(t, t .* e_norm));
    fprintf('Peak = %.4f\n', max(e_norm));
end

end

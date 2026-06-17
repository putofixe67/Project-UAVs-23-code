function plotResults(models, t, ref, graphList)

nModels = numel(models);

sys = models(1).sysConsts;

m = sys(1);
g = sys(2);
d = sys(3);

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

nList = length(graphList);

for k = 1:nList

    graphName = string(graphList(k));

    %% ==========================================================
    %                       POSITION
    % ===========================================================
    if strcmpi(graphName,"Position")

        % ---------------- 3D trajectory ----------------
        fig1 = figure('Name','3D Trajectory','Color','w');
        ax1 = axes(fig1);
        hold(ax1,'on');
        grid(ax1,'on');
        box(ax1,'on');

        plot3(ax1, ...
            ref.p(:,1), ref.p(:,2), ref.p(:,3), ...
            'k--', ...
            'LineWidth',1.5, ...
            'DisplayName','Reference');

        for k = 1:nModels

            plot3(ax1, ...
                models(k).x(:,1), ...
                models(k).x(:,2), ...
                models(k).x(:,3), ...
                'Color', models(k).color, ...
                'LineWidth',1.8, ...
                'DisplayName', models(k).name);
        end

        view(ax1,30,30);
        legend(ax1,'Location','best');

        xlabel(ax1,'$p_x$ [m]','Interpreter','latex');
        ylabel(ax1,'$p_y$ [m]','Interpreter','latex');
        zlabel(ax1,'$p_z$ [m]','Interpreter','latex');

        title(ax1,'3D Trajectory Comparison');

        % ---------------- Position states ----------------
        fig2 = figure('Name','Position States','Color','w');
        tlo1 = tiledlayout(fig2,3,1, ...
            'TileSpacing','compact');

        p_labels = {'$p_x$','$p_y$','$p_z$'};

        for j = 1:3

            ax = nexttile(tlo1);
            hold(ax,'on');
            grid(ax,'on');
            box(ax,'on');

            plot(ax, t, ref.p(:,j), ...
                'k:', ...
                'LineWidth',1.5, ...
                'DisplayName','Reference');

            for k = 1:nModels
                plot(ax, t, models(k).x(:,j), ...
                    'Color', models(k).color, ...
                    'LineWidth',1.5, ...
                    'DisplayName', models(k).name);
            end

            ylabel(ax,p_labels{j}, ...
                'Interpreter','latex');

            if j == 1
                title(ax,'Position States');
                legend(ax,'Location','best');
            end

            if j == 3
                xlabel(ax,'$t$ [s]', ...
                    'Interpreter','latex');
            end
        end
    end


    %% ==========================================================
    %                       VELOCITY
    % ===========================================================
    if strcmpi(graphName,"Velocity")

        fig3 = figure('Name','Velocity States','Color','w');
        tlo2 = tiledlayout(fig3,3,1, ...
            'TileSpacing','compact');

        v_labels = {'$v_x$','$v_y$','$v_z$'};

        for j = 1:3

            ax = nexttile(tlo2);
            hold(ax,'on');
            grid(ax,'on');
            box(ax,'on');

            if isfield(ref,'v') && ~isempty(ref.v)
                plot(ax, t, ref.v(:,j), ...
                    'k:', ...
                    'LineWidth',1.5, ...
                    'DisplayName','Reference');
            end

            for k = 1:nModels
                
                plot(ax, ...
                    t, ...
                    models(k).x(:,j+3), ...
                    'Color', models(k).color, ...
                    'LineWidth',1.5, ...
                    'DisplayName', models(k).name);
            end

            ylabel(ax,v_labels{j}, ...
                'Interpreter','latex');

            if j == 1
                title(ax,'Velocity States');
                legend(ax,'Location','best');
            end

            if j == 3
                xlabel(ax,'$t$ [s]', ...
                    'Interpreter','latex');
            end
        end
    end


    %% ==========================================================
    %                    CONTROL INPUTS
    % ===========================================================
if strcmpi(graphName,"Inputs")
        fig4 = figure('Name','Actuation Inputs','Color','w');
        tlo3 = tiledlayout(fig4,1,3,'TileSpacing','compact');
        u_labels = {'$\theta$ [deg]','$\phi$ [deg]','$T$ [N]'};
        u_sat = [10, 10, 0.588 ]; % [deg, deg, N]
        
        for j = 1:3
            ax = nexttile(tlo3);
            hold(ax,'on'); grid(ax,'on'); box(ax,'on');
            
            for k = 1:nModels
                uF = models(k).u; 
                
                if strcmpi(models(k).name, "Lyapunov")
                    
                    a = uF + [0; 0; m*g]; 
                    T = vecnorm(a, 2, 1);
                    
                    % Normalize columns to find directional components
                    b3 = a ./ T; 
                    theta = 180/pi * atan2(-b3(1,:), b3(3,:));
                    phi   = 180/pi * atan2(b3(2,:), b3(3,:));
                    u_plot = [theta; phi; T];
                else
                    % For LQR models (3 x N matrix)
                    T = uF(3,:) + m*g; % 1 x N
                    u_plot = [
                        180/pi * (-uF(2,:) ./ T); % Theta (1 x N)
                        180/pi * ( uF(1,:) ./ T); % Phi (1 x N)
                        T
                    ];
                end % <-- Fixed from '}' to 'end'
                
                plot(ax, t, u_plot(j,:), ...
                    'Color', models(k).color, ...
                    'LineWidth',1.5, ...
                    'DisplayName', models(k).name); 
            end
            
            if j==1 || j==2
                yline(ax, 40, '--r','LineWidth',1.5,'DisplayName','Physical limit');
                yline(ax,-40, '--r','LineWidth',1.5,'HandleVisibility','off');
            end 
            if j == 1
                yline(ax, u_sat(1), '--k','LineWidth',1.5,'DisplayName','Physical & Model limit');
                yline(ax,-u_sat(1), '--k','LineWidth',1.5,'HandleVisibility','off');
            end
            if j == 2
                yline(ax, u_sat(2), '--k','LineWidth',1.5,'DisplayName','Physical & Model limit');
                yline(ax,-u_sat(2), '--k','LineWidth',1.5,'HandleVisibility','off');
            end
            if j == 3
                yline(ax, u_sat(3), '--k','LineWidth',1.5,'DisplayName','Physical & Model limit');
                yline(ax,-u_sat(3), '--k','LineWidth',1.5,'HandleVisibility','off');
            end
            ylabel(ax,u_labels{j},'Interpreter','latex');
            xlabel(ax,'$t$ [s]','Interpreter','latex');
            if j == 2
                title(ax,'Control Inputs');
            end
            if j == 1
                legend(ax,'Location','best');
            end
        end
    end


    %% ==========================================================
    %                    POSITION ERROR
    % ===========================================================
    if strcmpi(graphName,"PositionError")

        fig5 = figure('Name','Position Error','Color','w');
        tlo4 = tiledlayout(fig5,3,1, ...
            'TileSpacing','compact');

        labels = { ...
            '$p_x$ error', ...
            '$p_y$ error', ...
            '$p_z$ error'};

        for j = 1:3

            ax = nexttile(tlo4);
            hold(ax,'on');
            grid(ax,'on');
            box(ax,'on');

            % Zero reference line
            plot(ax, t, zeros(size(t)), ...
                'k--', ...
                'LineWidth',1.2, ...
                'DisplayName','Zero error');

            for k = 1:nModels

                e = models(k).x(:,j) - ref.p(:,j);

                sigma = std(e);
                bound = 3 * sigma;

                plot(ax, t, e, ... 
                'Color', models(k).color, ...
                'LineWidth',1.2, 'DisplayName', ...
                [models(k).name ' error']);

                yline(ax,  bound, '--', 'Color', models(k).color, ...
            'LineWidth',1.0, 'HandleVisibility','off');

                yline(ax, -bound, '--', 'Color', models(k).color, ...
            'LineWidth',1.0, 'HandleVisibility','off');
            end

            ylabel(ax, labels{j}, ...
                'Interpreter','latex');

            if j == 1
                title(ax,'Position Error');
                legend(ax,'Location','best');
            end

            if j == 3
                xlabel(ax,'$t$ [s]', ...
                    'Interpreter','latex');
            end
        end
    end
end

%% ==========================================================
%                    VELOCITY ERROR
% ===========================================================

if strcmpi(graphName,"VelocityError")

    fig6 = figure('Name','Velocity Error','Color','w');
    tlo5 = tiledlayout(fig6,3,1, ...
        'TileSpacing','compact');

    labels = { ...
        '$v_x$ error', ...
        '$v_y$ error', ...
        '$v_z$ error'};

    for j = 1:3

        ax = nexttile(tlo5);
        hold(ax,'on');
        grid(ax,'on');
        box(ax,'on');

        % Zero reference line
        plot(ax, t, zeros(size(t)), ...
            'k--', ...
            'LineWidth',1.2, ...
            'DisplayName','Zero error');

        for k = 1:nModels

            v_model = models(k).x(:,4:6);
            v_ref   = ref.v;

            ev = v_model(:,j) - v_ref(:,j);

            sigma = std(ev);
            bound = 3 * sigma;

            plot(ax, t, ev, ... 
            'Color', models(k).color, ...
            'LineWidth',1.2, 'DisplayName', ...
            [models(k).name ' error']);

            yline(ax,  bound, '--', 'Color', models(k).color, ...
            'LineWidth',1.0, 'HandleVisibility','off');

            yline(ax, -bound, '--', 'Color', models(k).color, ...
            'LineWidth',1.0, 'HandleVisibility','off');
        end

        ylabel(ax, labels{j}, ...
            'Interpreter','latex');

        if j == 1
            title(ax,'Velocity Error');
            legend(ax,'Location','best');
        end

        if j == 3
            xlabel(ax,'$t$ [s]', ...
                'Interpreter','latex');
        end
    end
end

%% ==========================================================
%              STATE ERROR NORM (POSITION + VELOCITY)
% ===========================================================
if strcmpi(graphName,"ErrorNorm")

    fig6 = figure('Name','State Error Norm','Color','w');
    ax = axes(fig6);

    hold(ax,'on');
    grid(ax,'on');
    box(ax,'on');

    for k = 1:nModels

        % Position error
        ep = models(k).x(:,1:3) - ref.p;

        % Velocity error
        if isfield(ref,'v') && ~isempty(ref.v)
            ev = models(k).x(:,4:6) - ref.v;

            % Combined state error [ep ev]
            e = [ep ev];
        else
            % Only position if no velocity reference exists
            e = ep;
        end

        % Euclidean norm over time
        e_norm = vecnorm(e,2,2);

        plot(ax, ...
            t, ...
            e_norm, ...
            'Color', models(k).color, ...
            'LineWidth',1.8, ...
            'DisplayName', models(k).name);
    end

    xlabel(ax,'$t$ [s]', ...
        'Interpreter','latex');

    ylabel(ax,'$\|e\|$', ...
        'Interpreter','latex');

    if isfield(ref,'v') && ~isempty(ref.v)
        title(ax,'State Error Norm (Position + Velocity)');
    else
        title(ax,'Position Error Norm');
    end

    legend(ax,'Location','best');
end

for k = 1:nModels

    ep = models(k).x(:,1:3) - ref.p;
    ev = models(k).x(:,4:6) - ref.v;

    % weighted state error
    alpha_p = 1;
    alpha_v = 1;

    e_norm = sqrt( ...
        alpha_p*sum(ep.^2,2) + ...
        alpha_v*sum(ev.^2,2));

    % Metrics
    RMSE = sqrt(mean(e_norm.^2));
    ISE  = trapz(t, e_norm.^2);
    ITAE = trapz(t, t .* e_norm);
    peak = max(e_norm);

    fprintf('\n%s\n', models(k).name);
    fprintf('RMSE = %.4f\n', RMSE);
    fprintf('ISE  = %.4f\n', ISE);
    fprintf('ITAE = %.4f\n', ITAE);
    fprintf('Peak = %.4f\n', peak);
end

end
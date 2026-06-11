function ok = save_current_figure(filename)
%SAVE_CURRENT_FIGURE Robustly save current figure as PNG and close it.
%
% Compatible with MATLAB and GNU Octave.
% The function saves figures into the requested path and then closes the
% figure so that the script does not keep opening many windows.

    ok = false;

    try
        make_figures = evalin('base', 'MAKE_FIGURES');
    catch
        make_figures = true;
    end

    try
        close_after_save = evalin('base', 'CLOSE_FIGURES_AFTER_SAVE');
    catch
        close_after_save = true;
    end

    if ~make_figures
        return;
    end

    [folder, ~, ~] = fileparts(filename);
    ensure_dir(folder);

    try
        h = get(0, 'currentfigure');
    catch
        h = [];
    end

    if isempty(h)
        warning('save_current_figure:NoFigure', 'No current figure to save: %s', filename);
        return;
    end

    try
        set(h, 'visible', 'off');
    catch
    end

    % MATLAB modern path.
    try
        if exist('exportgraphics', 'file') == 2
            exportgraphics(h, filename, 'Resolution', 180);
            ok = true;
            if close_after_save
                close(h);
            end
            return;
        end
    catch
    end

    % MATLAB/Octave print fallback.
    try
        print(h, filename, '-dpng', '-r180');
        ok = true;
        if close_after_save
            close(h);
        end
        return;
    catch
    end

    % Last fallback.
    try
        saveas(h, filename);
        ok = true;
        if close_after_save
            close(h);
        end
        return;
    catch err2
        warning('save_current_figure:Failed', ...
            'Could not save figure %s. Last error: %s', filename, err2.message);
        ok = false;
        if close_after_save
            try
                close(h);
            catch
            end
        end
        return;
    end
end

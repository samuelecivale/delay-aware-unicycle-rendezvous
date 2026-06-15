function save_current_figure(filename)
%SAVE_CURRENT_FIGURE Save current figure as PNG/PDF/etc.
[path_str,~,~] = fileparts(filename);
if ~isempty(path_str)
    ensure_dir(path_str);
end
try
    set(gcf, 'PaperPositionMode', 'auto');
    print(gcf, filename, '-dpng', '-r160');
catch
    try
        saveas(gcf, filename);
    catch err
        warning('save_current_figure:Failed', 'Could not save %s: %s', filename, err.message);
    end
end
try
    close(gcf);
catch
end
end

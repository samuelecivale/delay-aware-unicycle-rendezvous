function write_cell_csv(filename, headers, rows)
%WRITE_CELL_CSV Write a cell array with a header row to CSV.
[fid, msg] = fopen(filename, 'w');
if fid < 0
    error('write_cell_csv:Open', 'Could not open %s: %s', filename, msg);
end
cleanup = onCleanup(@() fclose(fid));
write_row(fid, headers);
for r = 1:size(rows,1)
    write_row(fid, rows(r,:));
end
end

function write_row(fid, row)
for c = 1:numel(row)
    if c > 1
        fprintf(fid, ',');
    end
    val = row{c};
    if isnumeric(val) || islogical(val)
        if isempty(val)
            s = '';
        elseif isscalar(val)
            s = sprintf('%.12g', val);
        else
            s = mat2str(val);
        end
    elseif ischar(val)
        s = val;
    elseif isstring(val)
        s = char(val);
    else
        s = char(string(val));
    end
    s = strrep(s, '"', '""');
    if contains_for_csv(s)
        s = ['"' s '"'];
    end
    fprintf(fid, '%s', s);
end
fprintf(fid, '\n');
end

function tf = contains_for_csv(s)
tf = ~isempty(strfind(s, ',')) || ~isempty(strfind(s, '"')) || ~isempty(strfind(s, sprintf('\n')));
end

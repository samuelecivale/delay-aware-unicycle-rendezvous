function write_cell_csv(filename, headers, rows)
%WRITE_CELL_CSV Write a cell array table to CSV.
%
% headers: 1 x M cell array of strings
% rows:    R x M cell array with strings/numbers

    [folder, ~, ~] = fileparts(filename);
    ensure_dir(folder);

    fid = fopen(filename, 'w');

    if fid < 0
        error('Could not open file for writing: %s', filename);
    end

    % Header
    for j = 1:length(headers)
        fprintf(fid, '%s', headers{j});
        if j < length(headers)
            fprintf(fid, ',');
        else
            fprintf(fid, '\n');
        end
    end

    % Rows
    for i = 1:size(rows, 1)
        for j = 1:size(rows, 2)
            value = rows{i, j};

            if isnumeric(value)
                if isnan(value)
                    fprintf(fid, 'NaN');
                else
                    fprintf(fid, '%.12g', value);
                end
            else
                fprintf(fid, '%s', char(value));
            end

            if j < size(rows, 2)
                fprintf(fid, ',');
            else
                fprintf(fid, '\n');
            end
        end
    end

    fclose(fid);
    fprintf('Saved table: %s\n', filename);
end

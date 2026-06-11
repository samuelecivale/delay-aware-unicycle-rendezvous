function ensure_dir(pathname)
%ENSURE_DIR Create folder if it does not exist.
    if exist(pathname, 'dir') ~= 7
        mkdir(pathname);
    end
end

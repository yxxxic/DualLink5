function [best_value, best_l_drives, best_l_driven] = findBestLength(metric_map, l_drives_vec, l_driven_vec)

valid = isfinite(metric_map);

if ~any(valid(:))
    best_value = NaN;
    best_l_drives = NaN;
    best_l_driven = NaN;
    return;
end

temp = metric_map;
temp(~valid) = -inf;

[best_value, idx] = max(temp(:));

[row, col] = ind2sub(size(metric_map), idx);

best_l_driven = l_driven_vec(row);
best_l_drives = l_drives_vec(col);

end
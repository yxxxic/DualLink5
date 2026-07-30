function map_norm = normalizeMap(map)
% 归一化函数
map_norm = nan(size(map));

valid = isfinite(map);

if ~any(valid(:))
    return;
end

map_min = min(map(valid));
map_max = max(map(valid));

if abs(map_max - map_min) < 1e-12
    map_norm(valid) = 1;
else
    map_norm(valid) = (map(valid) - map_min) / (map_max - map_min);
end

end
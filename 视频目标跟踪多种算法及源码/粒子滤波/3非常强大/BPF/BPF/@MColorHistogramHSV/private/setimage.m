function obj = setimage(obj, val)

obj.MImage = val;
switch obj.type
    case 'global'
        obj.data   = get(val, 'hsvImage');
    case 'builtin'
        obj.data   = get(val, 'hsvImage');
    case 'local'
        obj.data   = get(val, 'image');
    otherwise
        error('Unknown type.');
end

obj.width  = size(obj.data, 2);
obj.height = size(obj.data, 1);
function temp_name = get_temp_name(org_name)

[pathstr, name, ext] = fileparts(org_name);
temp_name = [pathstr,'/temp_', name , ext];

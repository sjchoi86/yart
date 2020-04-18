function [mocap_name,action_str] = get_cmu_mocap_name(mocap_info,varargin)
%
% Get the name of CMU MoCap DB
%
persistent p
if isempty(p)
    p.FIRST_FLAG = true;
end

% Parse input arguments
parser = inputParser;
addParameter(parser,'mocap_folder','../mocap');
addParameter(parser,'VERBOSE',false);
parse(parser,varargin{:});
mocap_folder = parser.Results.mocap_folder;
VERBOSE = parser.Results.VERBOSE;

subject = mocap_info.subject;
action = mocap_info.action;

if p.FIRST_FLAG
    % Load excel file just once
    p.FIRST_FLAG = false;
    xml_path = dir([mocap_folder,'/**/cmu-mocap-index-spreadsheet.xls']);
    p.T = readcell([xml_path.folder,'/',xml_path.name]); % load excel file
    idx_match = idx_cell(p.T(:,1),[subject,'_',action]);
else
    idx_match = idx_cell(p.T(:,1),[subject,'_',action]);
end

if ~isempty(idx_match)
    action_str = p.T{idx_match,2}; % get action string from the xls file
    if ~isnumeric(action_str)
        action_str = strrep(action_str,'/',' ');
    else
        action_str = '';
    end
    mocap_name = sprintf('%s_%s %s',lower(subject),lower(action),action_str);
else
    action_str = '';
    mocap_name = sprintf('%s_%s',...
        lower(subject),lower(action));
end

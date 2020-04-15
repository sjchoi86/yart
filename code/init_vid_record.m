function vid_obj = init_vid_record(vid_path,varargin)
%
% Initialize the video object for recording
%
% Usage:
%   vid_obj = init_vid_record('');
%   record_vid(vid_obj);
%   end_vid_record(vid_obj);
%

% Parse options
p = inputParser;
addParameter(p,'HZ',10);
addParameter(p,'SAVE_VID',1);
parse(p,varargin{:});
HZ = p.Results.HZ;
SAVE_VID = p.Results.SAVE_VID;

% Open a video object
vid_path_temp = get_temp_name(vid_path);
if SAVE_VID
    [f,~,~] = fileparts(vid_path);
    make_dir_if_not_exist(f);
    vid_obj.writer = VideoWriter(vid_path_temp,'MPEG-4');
    vid_obj.writer.FrameRate = HZ;
    open(vid_obj.writer); % open video object
end

% Pass options
vid_obj.vid_path = vid_path;
vid_obj.vid_path_temp = vid_path_temp;
vid_obj.SAVE_VID = SAVE_VID;

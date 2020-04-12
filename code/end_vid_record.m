function end_vid_record(vid_obj)
%
% Finalize video recording
%
if vid_obj.SAVE_VID
    close(vid_obj.writer);
    movefile(vid_obj.vid_path_temp,vid_obj.vid_path);
    fprintf(2,'[%s] saved.\n',vid_obj.vid_path);
end

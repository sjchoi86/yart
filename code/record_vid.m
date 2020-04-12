function record_vid(vid_obj)
%
% Record video
%
if vid_obj.SAVE_VID
    writeVideo(vid_obj.writer, getframe(gcf));
end
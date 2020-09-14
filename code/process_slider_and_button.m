function sb = process_slider_and_button(sb)
global g

if g.play_button_pressed % if button pressed
    if sb.VERBOSE
        fprintf('Play button pressed.\n');
    end
    g.play_button_pressed = false;
    sb.PLAY = ~sb.PLAY; % toggle play
    if sb.PLAY
        pause(0.5); % little pause here
        sb.iclk = clock;
        sb.h_play_button.BackgroundColor = [0.9,0.3,0.15];
        sb.h_play_button.String = 'PAUSE';
        % Get speed based on edit box
        sb.SPEED = str2double(g.textbox_string);
        if isnan(sb.SPEED)
            sb.SPEED = 1.0;
        end
        % Update edit box
        sb.h_textbox.String = sprintf('%.2f',sb.SPEED);
    else
        sb.h_play_button.BackgroundColor = [0.5,0.7,0.9];
        sb.h_play_button.String = 'PLAY';
    end
end

if sb.PLAY % play
    esec = etime(clock, sb.iclk); % elapsed time in sec since start
    tick_estimated = round(esec*sb.HZ*sb.SPEED)+1; % estimated tick based on esec and SPEED
    g.slider_val = tick_estimated;
    sb.h_slider.Value = g.slider_val; % make sure to update slider value
    if g.slider_val > sb.slider_max
        g.slider_val = sb.h_slider.Min; % reset slider
        sb.h_slider.Value = g.slider_val; % make sure to update slider value
        sb.PLAY = false; % stop playing
        sb.h_play_button.BackgroundColor = [0.5,0.7,0.9];
        sb.h_play_button.String = 'PLAY';
    end
end

sb.tick_slider = round(g.slider_val);
sb.sec_sim = sb.tick_slider / sb.HZ; % simulator time
sb.sec_wall = etime(clock, sb.iclk); % wall-clock time

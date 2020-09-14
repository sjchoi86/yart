function sb = add_slider_and_button_to_figure(fig,slider_max,HZ,varargin)
global g

% Parse options
p = inputParser;
addParameter(p,'y_offset',0.005);
addParameter(p,'y_height',0.06);
addParameter(p,'sliderstep',max(1/slider_max,1/100));
addParameter(p,'fontsize',15);
addParameter(p,'VERBOSE',1);
parse(p,varargin{:});
y_offset = p.Results.y_offset;
y_height = p.Results.y_height;
sliderstep = p.Results.sliderstep;
fontsize = p.Results.fontsize;
VERBOSE = p.Results.VERBOSE;

% Options
sb.VERBOSE = VERBOSE;

% Add slider
sb.h_slider = uicontrol('parent',fig,'style','slider','units','normalized',...
    'sliderstep',[sliderstep,sliderstep],...
    'BackgroundColor', [0.5,0.7,0.9,0.5],...
    'position',[0.04, y_offset, 0.82, y_height],'min',1,'max',slider_max);
addlistener(sb.h_slider,'Value','PostSet',@cb_slider);
g.slider_val = sb.h_slider.Min;
sb.h_slider.Value = g.slider_val; % make sure to update slider value
sb.slider_max = slider_max; % slider max

% Add play button 
sb.h_play_button = uicontrol('parent',fig,'style','pushbutton','units','normalized',...
    'position',[0.8615, y_offset, 0.05, y_height],'string','PLAY','fontsize',fontsize,...
    'BackgroundColor', [0.5,0.7,0.9],'CallBack',@cb_play_button);
g.play_button_pressed = false;
sb.PLAY = false; % default is to not play
sb.HZ = HZ;
sb.iclk = clock; % initial clock 

% Add text box
sb.h_textbox = uicontrol('parent',fig,'style','edit','units','normalized',...
    'position',[0.915, y_offset, 0.05, y_height],'string','1.00',...
    'fontsize',fontsize,'BackgroundColor', [0.5,0.7,0.9], 'CallBack', @cb_textbox);
g.textbox_string = '1.00';
sb.SPEED = 1.00;

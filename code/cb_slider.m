function cb_slider(~, eventdata)
global g

slider_val = get(eventdata.AffectedObject, 'Value');
g.slider_val = slider_val;

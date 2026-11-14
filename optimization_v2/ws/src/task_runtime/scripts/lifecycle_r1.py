"""Strictly opt-in process-local shutdown compatibility, ABI/hash locked."""
from common_r1 import *

def validate():
    lock=read(APP/'config/shutdown.json')
    if lock['schema']!='R01-GAZEBO-SHUTDOWN/1':raise ValueError('SHUTDOWN_SCHEMA')
    for item in lock['libraries']:
        p=Path(item['path'])
        if not p.is_absolute() or not p.is_file() or sha(p)!=item['sha256']:raise ValueError('SHUTDOWN_ABI_OR_LIBRARY_CHANGED '+str(p))
    shim=Path(lock['preload'])
    if not shim.resolve().is_relative_to(APP/'build') or str(shim) not in [x['path'] for x in lock['libraries']]:raise ValueError('SHUTDOWN_SHIM_SCOPE')
    return lock

def environment(name,inherited,out):
    e=dict(inherited)
    if name=='rviz':
        # The upstream view mixes regional map coordinates with an odom frame.
        # Follow the measured base transform; change only this generated view.
        import yaml
        view_path=Path(out)/'scene/localization.rviz'
        view=yaml.safe_load(view_path.read_text())
        # The installed ImageDisplay destructor disconnects a dead filter
        # (captured GDB stack). Omit only that viewer plugin; the camera,
        # perception subscriber and annotated ROS image topic are unchanged.
        view['Visualization Manager']['Displays']=[d for d in view['Visualization Manager'].get('Displays',[]) if d.get('Class')!='rviz_default_plugins/Image']
        current=view['Visualization Manager']['Views']['Current']
        current.update({'Target Frame':'base_link','Focal Point':dict(X=0.,Y=0.,Z=.25),'Distance':12.})
        view_path.write_text(yaml.safe_dump(view,sort_keys=False))
        dump(Path(out)/'view_r1.json',dict(target_frame='base_link',fixed_frame=view['Visualization Manager']['Global Options']['Fixed Frame'],scope='Generated RViz view only',omitted_viewer_plugin='rviz_default_plugins/Image',reason='Installed ImageDisplay destructor failure; camera sensor and perception remain enabled'))
    if name!='server':return e
    if e.get('LD_PRELOAD'):raise ValueError('EXTERNAL_PRELOAD_NOT_QUALIFIED')
    lock=validate();e['LD_PRELOAD']=lock['preload']
    dump(Path(out)/'shutdown_compat.json',dict(scope='GAZEBO_SERVER_ONLY',lock=lock,
        lock_sha256=sha(APP/'config/shutdown.json'),return_codes_unmodified=True,
        physics_and_sensor_parameters_unchanged=True))
    return e

def close_owned_views(commands,children,out):
    """Ask the window manager to close only our RViz window before ROS teardown."""
    import ctypes,subprocess,re,time
    events=[]
    for descriptor,child in zip(commands,children):
        if descriptor['role']!='rviz' or child.poll() is not None:continue
        record=dict(pid=child.pid,windows=[],method='WM_CLOSE_OWNED_PID')
        try:
            listing=subprocess.run(['xprop','-root','_NET_CLIENT_LIST'],capture_output=True,text=True,timeout=2,check=True).stdout
            for window in re.findall(r'0x[0-9a-fA-F]+',listing):
                prop=subprocess.run(['xprop','-id',window,'_NET_WM_PID'],capture_output=True,text=True,timeout=2).stdout
                if not re.search(r'=\s*'+str(child.pid)+r'\s*$',prop):continue
                class Data(ctypes.Union):_fields_=[('b',ctypes.c_char*20),('s',ctypes.c_short*10),('l',ctypes.c_long*5)]
                class Message(ctypes.Structure):_fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),('send_event',ctypes.c_int),('display',ctypes.c_void_p),('window',ctypes.c_ulong),('message_type',ctypes.c_ulong),('format',ctypes.c_int),('data',Data)]
                class Key(ctypes.Structure):_fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),('send_event',ctypes.c_int),('display',ctypes.c_void_p),('window',ctypes.c_ulong),('root',ctypes.c_ulong),('subwindow',ctypes.c_ulong),('time',ctypes.c_ulong),('x',ctypes.c_int),('y',ctypes.c_int),('x_root',ctypes.c_int),('y_root',ctypes.c_int),('state',ctypes.c_uint),('keycode',ctypes.c_uint),('same_screen',ctypes.c_int)]
                class Event(ctypes.Union):_fields_=[('message',Message),('key',Key),('pad',ctypes.c_long*24)]
                x=ctypes.CDLL('libX11.so.6');x.XOpenDisplay.argtypes=[ctypes.c_char_p];x.XOpenDisplay.restype=ctypes.c_void_p
                x.XDefaultRootWindow.argtypes=[ctypes.c_void_p];x.XDefaultRootWindow.restype=ctypes.c_ulong
                x.XInternAtom.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int];x.XInternAtom.restype=ctypes.c_ulong
                x.XSendEvent.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,ctypes.POINTER(Event)]
                x.XKeysymToKeycode.argtypes=[ctypes.c_void_p,ctypes.c_ulong];x.XKeysymToKeycode.restype=ctypes.c_uint
                x.XFlush.argtypes=[ctypes.c_void_p];x.XCloseDisplay.argtypes=[ctypes.c_void_p]
                display=x.XOpenDisplay(None)
                if not display:raise RuntimeError('X11_DISPLAY_UNAVAILABLE')
                try:
                    # Save only this run's generated .rviz view, avoiding RViz's
                    # modal "unsaved changes" prompt. Keys target this PID's
                    # window directly; desktop focus and other apps are untouched.
                    view_path=Path(out)/'scene/localization.rviz';before=view_path.stat().st_mtime_ns
                    key=Event();key.key.send_event=1;key.key.display=display;key.key.window=int(window,16)
                    key.key.root=x.XDefaultRootWindow(display);key.key.state=4;key.key.keycode=x.XKeysymToKeycode(display,ord('s'));key.key.same_screen=1
                    for event_type,mask in [(2,1),(3,2)]:
                        key.key.type=event_type;x.XSendEvent(display,int(window,16),1,mask,ctypes.byref(key))
                    x.XFlush(display);time.sleep(.3)
                    record['generated_view_saved']=view_path.stat().st_mtime_ns!=before
                    event=Event();event.message.type=33;event.message.send_event=1;event.message.display=display
                    event.message.window=int(window,16);event.message.message_type=x.XInternAtom(display,b'_NET_CLOSE_WINDOW',0)
                    event.message.format=32;event.message.data.l[1]=1
                    sent=x.XSendEvent(display,x.XDefaultRootWindow(display),0,(1<<20)|(1<<19),ctypes.byref(event));x.XFlush(display)
                    record['windows'].append(dict(id=window,sent=bool(sent)))
                finally:x.XCloseDisplay(display)
            if record['windows']:child.wait(timeout=5)
        except Exception as exc:record['error']=repr(exc)
        record['exit_code']=child.poll();events.append(record)
    dump(Path(out)/'view_shutdown_r1.json',events)

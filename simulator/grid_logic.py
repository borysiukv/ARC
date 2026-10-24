"""Calibration-first grid model. Mock only; physical coordinates require measurement."""
import math

DEFAULT_GRID = {
    'columns': 3, 'rows': 3, 'pitch_x_mm': 50.0, 'pitch_y_mm': 50.0,
    'origin_x_mm': 0.0, 'origin_y_mm': 0.0,
    'rail_travel_mm': 300.0, 'rotation_step_deg': 15.0,
    'base_zero_deg': 90.0, 'grip_open_deg': 35.0, 'grip_closed_deg': 90.0,
}
AXES = ('base', 'shoulder', 'elbow', 'gripper', 'rail')

def validate_grid(g):
    if not isinstance(g,dict): raise ValueError('Grid must be an object')
    if not all(k in g for k in DEFAULT_GRID): raise ValueError('Missing grid parameters')
    q={}
    for k in DEFAULT_GRID:
        v=g[k]
        if type(v) not in (int,float) or not math.isfinite(v): raise ValueError('Invalid '+k)
        q[k] = int(v) if k in ('columns','rows') else float(v)
    if type(g['columns']) is not int or type(g['rows']) is not int or not (1<=q['columns']<=8 and 1<=q['rows']<=8): raise ValueError('Grid size 1..8')
    if not (1<=q['pitch_x_mm']<=500 and 1<=q['pitch_y_mm']<=500):raise ValueError('Pitch 1..500 mm')
    if not (1<=q['rail_travel_mm']<=3000): raise ValueError('Rail travel 1..3000 mm')
    if not (1<=q['rotation_step_deg']<=90):raise ValueError('Rotation step 1..90 deg')
    for k in ('base_zero_deg','grip_open_deg','grip_closed_deg'):
        if not 0<=q[k]<=180:raise ValueError(k+' 0..180 deg')
    return q

def cells(g):
    return [f'{chr(65+c)}{r+1}' for r in range(g['rows']) for c in range(g['columns'])]

def cell_xy(g,name):
    if name not in cells(g):raise ValueError('Cell outside grid')
    return {'x_mm':round(g['origin_x_mm']+(ord(name[0])-65)*g['pitch_x_mm'],2),
            'y_mm':round(g['origin_y_mm']+(int(name[1:])-1)*g['pitch_y_mm'],2)}

def validate_pose(p,g):
    if not isinstance(p,dict) or set(p)!=set(AXES)|{'grip_closed'}:raise ValueError('Pose requires five axes and grip_closed')
    if type(p['grip_closed']) is not bool:raise ValueError('grip_closed must be boolean')
    out={}
    for k in AXES:
        v=p[k]
        if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('Invalid '+k)
        if not (0 <= v <= (100 if k == 'rail' else 180)): raise ValueError('Axis out of bounds: '+k)
        out[k]=round(float(v),2)
    out['grip_closed']=p['grip_closed']
    return out

def steps_to_degrees(g,step):
    if type(step) is not int:raise ValueError('Rotation index must be integer')
    angle=g['base_zero_deg']+step*g['rotation_step_deg']
    if not 0<=angle<=180:raise ValueError('Rotation outside servo range')
    return angle

def rail_mm_to_percent(g,mm):
    if not 0<=mm<=g['rail_travel_mm']:raise ValueError('Rail target out of travel')
    return round(100*mm/g['rail_travel_mm'],2)

def cycle_frames(src,dst,safe,home,g):
    """Nominal simulation poses; not collision-free physical trajectories."""
    if src['grip_closed'] or dst['grip_closed']:raise ValueError('Pickup/drop presets must have gripper open')
    def grip(p,closed):
        o=dict(p);o['grip_closed']=closed;o['gripper']=g['grip_closed_deg'] if closed else g['grip_open_deg'];return o
    return [('APPROACH / GRIP OPEN',grip(src,False)),
            ('CLOSE GRIP',grip(src,True)),
            ('SAFE / LIFT',grip(safe,True)),
            ('MOVE TO TARGET / SAFE',grip({**safe,'rail':dst['rail'],'base':dst['base']},True)),
            ('LOWER TO TARGET',grip(dst,True)),
            ('OPEN GRIP',grip(dst,False)),
            ('RETURN TO SAFE',grip(safe,False)),
            ('HOME',grip(home,False))]

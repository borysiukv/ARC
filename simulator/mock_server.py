"""ARC'26 mock ESP32 API: manual + grid + logical step control. No real motors."""
import argparse
import json
import math
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
from grid_logic import DEFAULT_GRID, validate_grid, validate_pose, cells, cell_xy, cycle_frames

ROOT = Path(__file__).resolve().parent.parent
INDEX = ROOT / 'data' / 'index.html'
STORE = ROOT / 'simulator' / 'positions.json'
LIMITS = {'base': (0,180), 'shoulder': (0,180), 'elbow': (0,180), 'gripper': (0,180), 'rail': (0,100)}
DEFAULTS = {
    'HOME': dict(zip(LIMITS,(90,90,90,35,50))),
    'SAFE': dict(zip(LIMITS,(90,125,55,35,50))),
    'PICK': dict(zip(LIMITS,(78,63,105,70,25))),
    'A1': dict(zip(LIMITS,(65,65,105,40,20))),
    'C3': dict(zip(LIMITS,(118,70,100,40,80))),
}
POSITIONS = dict(DEFAULTS)
try:
    disk = json.loads(STORE.read_text(encoding='utf-8'))
    for name, pos in disk.items():
        if name not in DEFAULTS and isinstance(pos, dict) and set(pos) == set(LIMITS) and all(type(v) in (int,float) and math.isfinite(v) and LIMITS[k][0] <= v <= LIMITS[k][1] for k,v in pos.items()):
            POSITIONS[name] = pos
except (OSError, ValueError, TypeError):
    pass

GRID_FILE = ROOT / 'simulator' / 'grid_calibration.json'
GRID = dict(DEFAULT_GRID)
CALIBRATED = {}
try:
    saved = json.loads(GRID_FILE.read_text(encoding='utf-8'))
    GRID = validate_grid(saved['grid'])
    CALIBRATED = {k:validate_pose(v,GRID) for k,v in saved.get('cells',{}).items() if k in cells(GRID)}
except (ValueError,KeyError,OSError,TypeError):
    pass
GRID_CYCLE = None

def grid_store():
    temp=GRID_FILE.with_suffix('.tmp')
    temp.write_text(json.dumps({'grid':GRID,'cells':CALIBRATED},ensure_ascii=False,indent=2),encoding='utf-8')
    temp.replace(GRID_FILE)

def grid_snapshot():
    return {'grid':GRID,'cells':CALIBRATED,'logical_cells':{name:cell_xy(GRID,name) for name in cells(GRID)},
            'cycle':GRID_CYCLE,'axes':state['axes'],'locked':state['locked'],'simulation_only':True}

def grid_act(method,path,data):
    global GRID,GRID_CYCLE,CALIBRATED
    with lock:
        if method == 'GET' and path == '/api/grid':return grid_snapshot()
        if method != 'POST':raise LookupError('Unknown grid endpoint')
        if path == '/api/grid/stop':
            state['locked']=True;GRID_CYCLE=None;state['cycle']=None;state['commands']+=1;return grid_snapshot()
        if path == '/api/grid/unlock':
            state['locked']=False;state['commands']+=1;return grid_snapshot()
        if state['locked']:raise PermissionError('Locked: use unlock')
        if path == '/api/grid/config':
            if GRID_CYCLE is not None:raise PermissionError('Cycle active')
            GRID=validate_grid(data.get('grid'))
            CALIBRATED={k:v for k,v in CALIBRATED.items() if k in cells(GRID)}
            grid_store()
        elif path == '/api/grid/teach':
            if GRID_CYCLE is not None:raise PermissionError('Cycle active')
            name=data.get('cell')
            if name not in cells(GRID):raise ValueError('Unknown cell')
            # Teaching saves the presently selected VIRTUAL angles, NOT measured robot pose.
            p=validate_pose(data.get('pose'),GRID)
            if p['grip_closed']:raise ValueError('Teach target with gripper open')
            CALIBRATED[name]=p;grid_store()
        elif path == '/api/grid/goto':
            if GRID_CYCLE is not None:raise PermissionError('Cycle active')
            name=data.get('cell')
            if name not in CALIBRATED:raise ValueError('Cell has not been taught')
            p=CALIBRATED[name]
            state['axes']={a:p[a] for a in ('base','shoulder','elbow','gripper','rail')}
        elif path == '/api/grid/cycle/start':
            if GRID_CYCLE is not None:raise PermissionError('Cycle already active')
            if state['cycle'] is not None:raise PermissionError('Legacy cycle active')
            src,dst=data.get('from'),data.get('to')
            if src==dst or src not in CALIBRATED or dst not in CALIBRATED:raise ValueError('Teach two different cells before cycling')
            safe={**POSITIONS['SAFE'], 'grip_closed':False}
            home={**POSITIONS['HOME'], 'grip_closed':False}
            frames=cycle_frames(CALIBRATED[src],CALIBRATED[dst],safe,home,GRID)
            GRID_CYCLE={'from':src,'to':dst,'step':0,'last_step':None,'total':len(frames)}
        elif path == '/api/grid/cycle/next':
            if GRID_CYCLE is None:raise ValueError('No active cycle')
            src,dst=GRID_CYCLE['from'],GRID_CYCLE['to']
            frames=cycle_frames(CALIBRATED[src],CALIBRATED[dst],{**POSITIONS['SAFE'],'grip_closed':False},{**POSITIONS['HOME'],'grip_closed':False},GRID)
            name,p=frames[GRID_CYCLE['step']]
            state['axes']={a:p[a] for a in ('base','shoulder','elbow','gripper','rail')}
            GRID_CYCLE['last_step']=name;GRID_CYCLE['step']+=1
            if GRID_CYCLE['step']==len(frames):GRID_CYCLE=None
        elif path == '/api/grid/cycle/reset':GRID_CYCLE=None
        else:raise LookupError('Unknown grid endpoint')
        state['commands']+=1
        return grid_snapshot()

# Logical step control: no motors are commanded. X steps are NOT millimeters.
STEP_X_MIN, STEP_X_MAX = 0, 5
STEP_R_MIN, STEP_R_MAX = 0, 12
STEP_R_DEG = 15
STEP = {'x_step': 0, 'rotation_step': 6, 'grip_closed': False}


def step_snapshot():
    return {
        **STEP,
        'rotation_deg': STEP['rotation_step'] * STEP_R_DEG,
        'simulation_only': True,
        'locked': state['locked'],
    }


def integer_step(value, low, high):
    # bool is an int subclass: reject it and fractional/invalid input.
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'Step must be an integer from {low} to {high}')
    return value


def step_act(method, path, data):
    with lock:
        if method == 'GET' and path == '/api/step':
            return step_snapshot()
        if method != 'POST':
            raise LookupError('Unknown step endpoint')
        if state['locked'] or GRID_CYCLE is not None or state['cycle'] is not None:
            raise PermissionError('Robot locked or automatic cycle active')
        if path == '/api/step/x':
            STEP['x_step'] = integer_step(data.get('step'), STEP_X_MIN, STEP_X_MAX)
            # NOTE: rail in /api/state remains a legacy percent, not step position.
        elif path == '/api/step/rotation':
            STEP['rotation_step'] = integer_step(data.get('step'), STEP_R_MIN, STEP_R_MAX)
            state['axes']['base'] = STEP['rotation_step'] * STEP_R_DEG
        elif path == '/api/step/grip':
            closed = data.get('closed')
            if type(closed) is not bool:
                raise ValueError('closed must be true or false')
            STEP['grip_closed'] = closed
            # Simulation-only servo angle, not calibrated for actual grip hardware.
            state['axes']['gripper'] = 115 if closed else 35
        else:
            raise LookupError('Unknown step endpoint')
        state['commands'] += 1
        return step_snapshot()


lock = threading.RLock()
state = {'axes': DEFAULTS['HOME'].copy(), 'locked': False, 'cycle': None, 'commands': 0, 'mode': 'MOCK', 'hardware': False}
STEPS = ['MOVE TO PICK', 'CLOSE GRIP', 'MOVE TO SAFE', 'MOVE TO PLACE', 'OPEN GRIP', 'RETURN HOME']


def snapshot():
    return {**state, 'positions': POSITIONS, 'steps': STEPS, 'timestamp': datetime.now(timezone.utc).isoformat()}


def position(value):
    if not isinstance(value, dict) or set(value) != set(LIMITS):
        raise ValueError('Expected all five axes: base, shoulder, elbow, gripper, rail')
    result = {}
    for key,(low,high) in LIMITS.items():
        number = value[key]
        if type(number) not in (int,float) or not math.isfinite(number) or not low <= number <= high:
            raise ValueError(f'{key} must be a number from {low} to {high}')
        result[key] = round(number, 2)
    return result


def act(method,path,data):
    global GRID_CYCLE
    with lock:
        if method == 'GET' and path == '/api/state': return snapshot()
        if method == 'GET' and path == '/api/positions': return {'positions': POSITIONS}
        if method != 'POST': raise LookupError('Unknown endpoint')
        if path == '/api/stop':
            GRID_CYCLE = None
            state['locked'] = True
            state['cycle'] = None
            state['commands'] += 1
            return snapshot()
        if path == '/api/unlock':
            state['locked'] = False
            state['commands'] += 1
            return snapshot()
        if state['locked']: raise PermissionError('Mock control locked; unlock first')
        if GRID_CYCLE is not None and path not in ('/api/stop', '/api/unlock'):
            raise PermissionError('Grid cycle active: complete or reset it first')
        if state['cycle'] is not None and path not in ('/api/cycle/reset','/api/cycle/next'):
            raise PermissionError('Cycle active: finish or reset before manual commands')
        if path == '/api/axes':
            state['axes'] = position(data.get('axes'))
        elif path == '/api/position/load':
            name = data.get('name')
            if not isinstance(name,str) or name not in POSITIONS: raise ValueError('Unknown saved position')
            state['axes'] = POSITIONS[name].copy()
        elif path == '/api/position/save':
            name = data.get('name')
            if not isinstance(name,str) or not (1 <= len(name) <= 12) or not all(ch in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for ch in name) or name in DEFAULTS:
                raise ValueError('Name: 1-12 uppercase letters/numbers/_/-, not a built-in preset')
            POSITIONS[name] = state['axes'].copy()
            temp = STORE.with_suffix('.tmp')
            temp.write_text(json.dumps({k:v for k,v in POSITIONS.items() if k not in DEFAULTS}, ensure_ascii=False, indent=2),encoding='utf-8')
            temp.replace(STORE)
        elif path == '/api/cycle/start':
            fr,to = data.get('from'),data.get('to')
            if not isinstance(fr,str) or not isinstance(to,str) or fr == to or fr not in POSITIONS or to not in POSITIONS:
                raise ValueError('Choose two distinct saved positions')
            state['cycle'] = {'from':fr,'to':to,'step':0,'last_step':None}
        elif path == '/api/cycle/next':
            c = state['cycle']
            if c is None: raise ValueError('No cycle in progress')
            step = c['step']
            if step == 0: state['axes'] = POSITIONS[c['from']].copy()
            elif step == 1: state['axes']['gripper'] = 115
            elif step == 2: state['axes'] = POSITIONS['SAFE'].copy()
            elif step == 3: state['axes'] = POSITIONS[c['to']].copy()
            elif step == 4: state['axes']['gripper'] = 25
            elif step == 5: state['axes'] = POSITIONS['HOME'].copy()
            c['last_step'] = STEPS[step]
            c['step'] += 1
            if c['step'] == len(STEPS): state['cycle'] = None
        elif path == '/api/cycle/reset': state['cycle'] = None
        else: raise LookupError('Unknown endpoint')
        state['commands'] += 1
        return snapshot()


class Handler(BaseHTTPRequestHandler):
    def send(self, status, obj, content_type='application/json; charset=utf-8'):
        raw = obj if isinstance(obj,bytes) else json.dumps(obj,ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ('/', '/index.html'):
            if not INDEX.is_file():
                self.send(404, {'error': 'data/index.html not found'})
                return
            self.send(200, INDEX.read_bytes(), 'text/html; charset=utf-8')
            return
        if path in ('/grid', '/grid/'):
            grid_html = ROOT / 'data' / 'grid.html'
            if not grid_html.is_file():
                self.send(404, {'error': 'data/grid.html not found'})
                return
            self.send(200, grid_html.read_bytes(), 'text/html; charset=utf-8')
            return
        if path == '/api/health':
            self.send(200, {'app': 'ARC26 Mock ESP32', 'version': '0.7', 'grid_enabled': True, 'step_enabled': True, 'simulation_only': True})
            return
        if path == '/favicon.ico': self.send(204,b''); return
        try:
            if path.startswith('/api/step'):
                result = step_act('GET', path, {})
            elif path.startswith('/api/grid'):
                result = grid_act('GET', path, {})
            else:
                result = act('GET', path, {})
            self.send(200, result)
        except LookupError as ex:
            self.send(404, {'error': str(ex)})
        except (ValueError, TypeError, KeyError) as ex:
            self.send(400, {'error': str(ex)})

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            length = int(self.headers.get('Content-Length','0'))
            if length > 16384: raise ValueError('Payload too large')
            data = json.loads(self.rfile.read(length)) if length else {}
            if not isinstance(data,dict): raise ValueError('JSON object required')
            if path.startswith('/api/step'):
                result = step_act('POST', path, data)
            elif path.startswith('/api/grid'):
                result = grid_act('POST', path, data)
            else:
                result = act('POST', path, data)
            self.send(200, result)
        except PermissionError as ex: self.send(409,{'error':str(ex)})
        except (ValueError,TypeError,KeyError,json.JSONDecodeError) as ex: self.send(400,{'error':str(ex)})
        except LookupError as ex: self.send(404,{'error':str(ex)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1', help='127.0.0.1 (local only) or 0.0.0.0 (LAN)')
    parser.add_argument('--port', type=int, default=8001)
    args = parser.parse_args()
    try:
        server = ThreadingHTTPServer((args.host,args.port), Handler)
    except OSError as exc:
        print(f'ERROR: Cannot bind to {args.host}:{args.port}: {exc}', flush=True)
        print('Try: python simulator/mock_server.py --port 8002', flush=True)
        raise SystemExit(1)
    print(f"ARC'26 Mock ESP32 running at http://{args.host}:{args.port}", flush=True)
    print('SIMULATION ONLY. No motors are connected.', flush=True)
    print(f'GRID UI: http://{args.host}:{args.port}/grid', flush=True)
    print(f'GRID API: http://{args.host}:{args.port}/api/grid', flush=True)
    print(f'STEP API: http://{args.host}:{args.port}/api/step', flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()

if __name__ == '__main__': main()

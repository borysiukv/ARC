"""ARC'26 local ESP32 API simulator. Python 3.9+, stdlib only; no physical motor control."""
import argparse
import json
import math
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

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
    with lock:
        if method == 'GET' and path == '/api/state': return snapshot()
        if method == 'GET' and path == '/api/positions': return {'positions': POSITIONS}
        if method != 'POST': raise LookupError('Unknown endpoint')
        if path == '/api/stop':
            state['locked'] = True
            state['cycle'] = None
            state['commands'] += 1
            return snapshot()
        if path == '/api/unlock':
            state['locked'] = False
            state['commands'] += 1
            return snapshot()
        if state['locked']: raise PermissionError('Mock control locked; unlock first')
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
            self.send(200, INDEX.read_bytes(), 'text/html; charset=utf-8'); return
        if path == '/favicon.ico': self.send(204,b''); return
        try: self.send(200, act('GET',path,{}))
        except LookupError as ex: self.send(404,{'error':str(ex)})

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            length = int(self.headers.get('Content-Length','0'))
            if length > 16384: raise ValueError('Payload too large')
            data = json.loads(self.rfile.read(length)) if length else {}
            if not isinstance(data,dict): raise ValueError('JSON object required')
            self.send(200, act('POST',path,data))
        except PermissionError as ex: self.send(409,{'error':str(ex)})
        except (ValueError,TypeError,KeyError,json.JSONDecodeError) as ex: self.send(400,{'error':str(ex)})
        except LookupError as ex: self.send(404,{'error':str(ex)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1', help='127.0.0.1 (local only) or 0.0.0.0 (LAN)')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host,args.port), Handler)
    print(f"ARC'26 Mock ESP32 running at http://{args.host}:{args.port}", flush=True)
    print('SIMULATION ONLY. No motors are connected.', flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()

if __name__ == '__main__': main()

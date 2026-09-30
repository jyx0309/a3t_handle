"""One explicit local request, no connection to the robot and no retries."""
import argparse
import json
import socket
import uuid


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--socket', required=True)
    p.add_argument('command', choices=['status', 'connect', 'start_mit', 'gravity', 'probe', 'batch', 'stop', 'emergency'])
    p.add_argument('--session', help='Session UUID returned by status')
    p.add_argument('--confirm', action='store_true')
    p.add_argument('--joint', type=int, choices=range(1, 7))
    p.add_argument('--direction', type=int, choices=[-1, 1])
    args = p.parse_args()
    if args.command != 'status' and not (args.confirm and args.session):
        p.error('Writes require --confirm and --session; do not retry an uncertain request.')
    request = dict(command=args.command, session=args.session, confirm=args.confirm,
                   id=str(uuid.uuid4()), joint=args.joint, direction=args.direction)
    try:
        with socket.socket(socket.AF_UNIX) as sock:
            sock.settimeout(3)
            sock.connect(args.socket)
            sock.sendall(json.dumps(request).encode() + b'\n')
            data = b''
            while b'\n' not in data:
                chunk = sock.recv(4096)
                if not chunk or len(data) > 65536:
                    raise RuntimeError('Missing or oversized reply')
                data += chunk
            result = json.loads(data)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result.get('ok') else 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'Local request failed/uncertain: {exc}. No automatic retry.')
        return 2


if __name__ == '__main__':
    raise SystemExit(main())

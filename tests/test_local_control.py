"""Offline integration: never issue connect or any accepted hardware command."""
import json
import os
import socket
import subprocess
import sys
import tempfile
import time

with tempfile.TemporaryDirectory(prefix='a3t-local-test-') as directory:
    endpoint = directory + '/control.sock'
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen', A3T_CONTROL_SOCKET=endpoint)
    process = subprocess.Popen([sys.argv[1]], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        def call(payload):
            with socket.socket(socket.AF_UNIX) as sock:
                sock.settimeout(2)
                sock.connect(endpoint)
                sock.sendall(json.dumps(payload).encode() + b'\n')
                data = b''
                while b'\n' not in data:
                    chunk = sock.recv(4096)
                    assert chunk
                    data += chunk
                return json.loads(data)
        deadline = time.monotonic() + 5
        while not os.path.exists(endpoint):
            assert process.poll() is None and time.monotonic() < deadline
            time.sleep(.02)
        status = call({'command': 'status'})
        assert status['ok'] and not status['connected'] and not status['mit']
        assert os.stat(endpoint).st_mode & 0o077 == 0
        assert not call({'command': 'start_mit'})['ok']
        base = dict(session=status['session'], confirm=True, id='offline')
        assert not call(dict(base, command='reset'))['ok']
        assert not call(dict(base, command='probe', joint=7, direction=1))['ok']
        assert not call(dict(base, command='start_mit'))['ok']
        assert not call(dict(base, command='connect', session='wrong-session'))['ok']
        # Occupied endpoints must not be removed or stolen by another instance.
        other = subprocess.run([sys.argv[1]], env=env, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=5)
        assert other.returncode == 4
        assert call({'command': 'status'})['session'] == status['session']
        assert not call({'command': 'status'})['connected']
    finally:
        process.terminate()
        process.wait(timeout=5)

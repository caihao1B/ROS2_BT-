#!/usr/bin/env python3
"""Capture the offline manual after actual layout, using local Chrome CDP.

Python standard library; temporary Chrome profile and loopback-only debugging.
Only the Chrome process created here is terminated. No ROS/Gazebo interactions.
"""
import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import tempfile
import time
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import urlopen

HERE=Path(__file__).resolve().parent


class CDP:
    def __init__(self,url):
        u=urlparse(url);self.sock=socket.create_connection((u.hostname,u.port),timeout=10);self.seq=0
        key=base64.b64encode(os.urandom(16)).decode()
        request=(f'GET {u.path} HTTP/1.1\r\nHost: {u.hostname}:{u.port}\r\nUpgrade: websocket\r\n'
                 f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n')
        self.sock.sendall(request.encode());header=b''
        while not header.endswith(b'\r\n\r\n'):header+=self.sock.recv(1)
        if b' 101 ' not in header:raise RuntimeError(header.decode())
    def read(self,size):
        data=b''
        while len(data)<size:
            chunk=self.sock.recv(size-len(data))
            if not chunk:raise EOFError('CDP closed')
            data+=chunk
        return data
    def frame(self,payload,opcode=1):
        mask=os.urandom(4);size=len(payload)
        header=bytes([0x80|opcode])
        if size<126:header+=bytes([0x80|size])
        elif size<65536:header+=bytes([0x80|126])+struct.pack('!H',size)
        else:header+=bytes([0x80|127])+struct.pack('!Q',size)
        self.sock.sendall(header+mask+bytes(x^mask[i%4] for i,x in enumerate(payload)))
    def receive(self):
        fragments=[]
        while True:
            b1,b2=self.read(2);size=b2&127
            if size==126:size=struct.unpack('!H',self.read(2))[0]
            elif size==127:size=struct.unpack('!Q',self.read(8))[0]
            mask=self.read(4) if b2&128 else None;data=self.read(size)
            if mask:data=bytes(x^mask[i%4] for i,x in enumerate(data))
            if b1&15==9:self.frame(data,10);continue
            if b1&15==8:raise EOFError('CDP closing')
            fragments.append(data)
            if b1&128:return json.loads(b''.join(fragments))
    def call(self,method,params=None):
        self.seq+=1;self.frame(json.dumps(dict(id=self.seq,method=method,params=params or {})).encode())
        while True:
            msg=self.receive()
            if msg.get('id')==self.seq:
                if 'error' in msg:raise RuntimeError(msg['error'])
                return msg.get('result',{})
    def evaluate(self,expression):
        result=self.call('Runtime.evaluate',dict(expression=expression,returnByValue=True,awaitPromise=True))
        if 'exceptionDetails' in result:raise RuntimeError(result['exceptionDetails'])
        return result.get('result',{}).get('value')


def main():
    chrome=shutil.which('google-chrome') or shutil.which('chromium')
    if not chrome:raise RuntimeError('Existing Chrome required; no installation attempted')
    screenshots=[]
    with tempfile.TemporaryDirectory(prefix='robot-learning-capture-') as profile:
        proc=subprocess.Popen([chrome,'--headless','--disable-gpu','--no-first-run',
                               '--disable-background-networking','--disable-dev-shm-usage',
                               '--remote-debugging-port=0','--remote-debugging-address=127.0.0.1',
                               '--user-data-dir='+profile,'about:blank'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        cdp=None
        try:
            active=Path(profile)/'DevToolsActivePort'
            for _ in range(200):
                if active.exists():break
                if proc.poll() is not None:raise RuntimeError('Chrome exited')
                time.sleep(.05)
            port=int(active.read_text().splitlines()[0])
            with urlopen(f'http://127.0.0.1:{port}/json/list',timeout=5) as response:targets=json.load(response)
            target=next(t for t in targets if t['type']=='page');cdp=CDP(target['webSocketDebuggerUrl'])
            cdp.call('Page.enable');cdp.call('Runtime.enable')
            cdp.call('Emulation.setDeviceMetricsOverride',dict(width=1400,height=1100,deviceScaleFactor=1,mobile=False))
            cdp.call('Page.navigate',dict(url=(HERE/'index.html').as_uri()))
            for _ in range(200):
                if cdp.evaluate('document.readyState === "complete" && window.learningReady === true'):break
                time.sleep(.05)
            else:raise RuntimeError('Offline manual did not initialize')
            assert cdp.evaluate('window.learningErrors.length')==0
            for name,width,height,selector in [('desktop',1400,1100,'#top'),('mobile',420,1050,'body'),
                                               ('experiment',1400,1000,'#wheel-r')]:
                cdp.call('Emulation.setDeviceMetricsOverride',dict(width=width,height=height,deviceScaleFactor=1,mobile=False))
                if name=='experiment':cdp.evaluate('document.getElementById("wheel-r").closest(".lab").scrollIntoView({behavior:"instant",block:"start"})')
                else:cdp.evaluate('window.scrollTo({top:0,behavior:"instant"})')
                cdp.evaluate('new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(() => resolve(true))))')
                time.sleep(.15)
                response=cdp.call('Page.captureScreenshot',dict(format='png',captureBeyondViewport=False,fromSurface=True))
                path=HERE/'checks'/f'{name}.png';path.write_bytes(base64.b64decode(response['data']))
                screenshots.append(dict(path=str(path.relative_to(HERE)),viewport=[width,height],bytes=path.stat().st_size,
                                        scroll_y=cdp.evaluate('window.scrollY')))
            (HERE/'checks/capture.json').write_text(json.dumps(dict(mode='actual Chrome CDP; offline file URL',screenshots=screenshots),indent=2)+'\n')
            print(json.dumps(screenshots,indent=2))
        finally:
            if cdp:cdp.sock.close()
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=5)


if __name__=='__main__':main()

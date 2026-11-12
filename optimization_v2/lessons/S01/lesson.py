#!/usr/bin/env python3
"""Teaching example: reject an invalid configuration before starting processes."""
import json,sys
from pathlib import Path

def validate(config):
    if config['ros_domain_id']!=119:raise ValueError('DOMAIN_MISMATCH')
    root=Path(config['project_root']).resolve()
    for relative in config['required_files']:
        path=(root/relative).resolve()
        if not path.is_relative_to(root):raise ValueError('PATH_OUTSIDE_PROJECT')
        if not path.is_file():raise ValueError('MISSING_RESOURCE: '+relative)
    return len(config['required_files'])

if __name__=='__main__':
    try:
        n=validate(json.loads(Path(sys.argv[1]).read_text()))
        print(json.dumps({'status':'READY','validated_files':n}))
    except (OSError,ValueError,KeyError,IndexError) as e:
        print(str(e),file=sys.stderr);sys.exit(2)

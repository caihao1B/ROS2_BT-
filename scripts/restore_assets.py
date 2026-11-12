#!/usr/bin/env python3
"""Restore the release's lossless asset bundle; never manufacture test receipts."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import tempfile


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def safe_path(root, name):
    rel = PurePosixPath(name)
    if not rel.parts or rel.is_absolute() or '..' in rel.parts or '\\' in name:
        raise ValueError('Unsafe asset path: ' + name)
    p = root
    for part in rel.parts:
        p = p / part
        if p.is_symlink():
            raise ValueError('Refusing symlink destination: ' + name)
    if not p.resolve().is_relative_to(root):
        raise ValueError('Asset escapes project root: ' + name)
    return p


def restore(root, manifest):
    root = root.resolve()
    spec = json.loads(manifest.read_text())
    if spec['schema'] != 'ROBOT-RELEASE-ASSETS/1':
        raise ValueError('Unsupported asset schema')
    entries = {e['path']: e for e in spec['files']}
    if len(entries) != len(spec['files']):
        raise ValueError('Duplicate asset destination')
    for name, entry in entries.items():
        path = safe_path(root, name)
        if path.exists() and (not path.is_file() or digest(path) != entry['sha256']):
            raise ValueError('Existing file differs; preserve it before restoring: ' + name)
    with tempfile.TemporaryDirectory(prefix='robot-assets-') as temporary:
        archive = Path(temporary) / 'assets.tar.gz'
        bundle_hash = hashlib.sha256()
        with archive.open('wb') as output:
            for part in spec['parts']:
                path = safe_path(root, part['path'])
                if path.stat().st_size != part['size'] or digest(path) != part['sha256']:
                    raise ValueError('Missing or corrupt bundle part: ' + part['path'])
                with path.open('rb') as stream:
                    for block in iter(lambda: stream.read(1024 * 1024), b''):
                        bundle_hash.update(block)
                        output.write(block)
        if bundle_hash.hexdigest() != spec['archive_sha256']:
            raise ValueError('Archive digest mismatch')
        # Validate every payload before changing any destination.
        with tarfile.open(archive, 'r:gz') as source:
            members = source.getmembers()
            if len(members) != len(entries) or {m.name for m in members} != set(entries):
                raise ValueError('Archive inventory differs from manifest')
            for member in members:
                entry = entries[member.name]
                if not member.isfile() or member.size != entry['size']:
                    raise ValueError('Unexpected archive member: ' + member.name)
                h = hashlib.sha256()
                with source.extractfile(member) as stream:
                    for block in iter(lambda: stream.read(1024 * 1024), b''):
                        h.update(block)
                if h.hexdigest() != entry['sha256']:
                    raise ValueError('Payload digest mismatch: ' + member.name)
            for member in members:
                path = safe_path(root, member.name)
                if path.exists():
                    continue
                path.parent.mkdir(parents=True, exist_ok=True)
                fd, temp_name = tempfile.mkstemp(prefix='.restore-', dir=path.parent)
                try:
                    with os.fdopen(fd, 'wb') as output, source.extractfile(member) as stream:
                        shutil.copyfileobj(stream, output)
                    os.chmod(temp_name, entries[member.name].get('mode', 0o644) & 0o777)
                    os.replace(temp_name, path)
                finally:
                    Path(temp_name).unlink(missing_ok=True)
    print(f'ASSETS_VERIFIED: {len(entries)} original files; no acceptance receipts changed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--manifest', type=Path)
    args = parser.parse_args()
    restore(args.root, args.manifest or args.root / 'assets_bundle/manifest.json')

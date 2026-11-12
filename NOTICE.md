# Code and asset notice

This public snapshot contains the current native ROS 2 optimization project.
Existing package and third-party license notices are retained; this export does
not grant a new blanket license to third-party material. NanoDet uses Apache-2.0;
the pedestrian model carries its upstream Fuel metadata; pinned Trajectron++
source and weights retain the upstream MIT license and provenance manifest; RTAB-Map and other
reference source retain their own notices. The original Dino asset archive and
unlicensed legacy meshes are excluded. The new primitive chassis/world models,
authorized prior, measured simulated maps and validated plans are included.

Large inputs are packed losslessly. Run `python3 scripts/restore_assets.py` before
building. Restore verifies every original byte hash and refuses to overwrite
modified local files. It never creates a PASS receipt. Development-machine
results are recorded separately under `validation/`; a fresh checkout requires
dependency installation, rebuilding and requalification. The historical S01
Dino migration benchmark additionally needs separately authorized legacy assets.

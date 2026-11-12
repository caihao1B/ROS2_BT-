# Fixed upstream model provenance

Trajectron-plus-plus sources and ETH pedestrian checkpoint are from the official StanfordASL repository, ECCV commit `ef0165a93ee5ba8cdc14f9b999b3e00070cd8588`:

https://github.com/StanfordASL/Trajectron-plus-plus/tree/ef0165a93ee5ba8cdc14f9b999b3e00070cd8588

The upstream MIT license is retained at `Trajectron-plus-plus/LICENSE`. `upstream_files.json` records each original Git blob. The original source files have not been edited. The 545361-byte checkpoint has SHA-256 `157f56c98e721e888bdbee82000e93ed536af3820193d9c7b5b3ca669938d9e9` and Git blob `04ae98310ecefeb70b6eedd1fda9bf5ff993406f`.

Project adapter `../scripts/compat15.py` explicitly handles modern Python/PyTorch metadata and skips only the unused final modal grid search. `inference15.py` compares the resulting distributions with the complete original calculation. Network weights, conditional decoder and integration are preserved. Old serialized ModuleDict loading is limited to this verified checkpoint; no global deserialization security setting is disabled.

`*_install.json` are real pip installation reports including wheel origins and hashes. Exact install inputs are `optimization_v2/config/prediction_wheels.lock`; packages are installed in a separate CPU-only Python 3.12 environment. No raw ETH or nuScenes training datasets are redistributed. This project does not claim to have retrained the network.

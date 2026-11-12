"""Minimal explicit ECCV compatibility patches; leave vendor bytes unchanged."""
from pathlib import Path
import hashlib
import importlib.util
import sys
import importlib.abc
import importlib.machinery

VENDOR = Path(__file__).resolve().parents[1]/'vendor/Trajectron-plus-plus'
PATCHES = None


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError('PINNED_UPSTREAM_ADAPTER_MISMATCH: '+old[:90])
    return source.replace(old, new)


def install():
    global PATCHES
    if PATCHES is not None:
        return PATCHES
    sys.path.insert(0, str(VENDOR/'trajectron'))
    class CollectionLoader(importlib.machinery.SourceFileLoader):
        def get_code(self, fullname):
            source=Path(self.path).read_text()
            source=replace_once(source,'from collections import Sequence, OrderedDict',
                'from collections.abc import Sequence\nfrom collections import OrderedDict')
            return compile(source,self.path,'exec')
    class CollectionFinder(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path, target=None):
            if fullname=='environment.data_structures':
                filename=str(VENDOR/'trajectron/environment/data_structures.py')
                return importlib.util.spec_from_loader(fullname,CollectionLoader(fullname,filename))
    finder=CollectionFinder();sys.meta_path.insert(0,finder)
    # Import packages first, then replace only class methods on the actual classes.
    # This avoids circular imports and keeps serialized ModuleDict names unchanged.
    from model.components.gmm2d import GMM2D
    from model.mgcvae import MultimodalGenerativeCVAE
    import model.components.gmm2d as gmm
    import model.mgcvae as mgcvae
    sys.meta_path.remove(finder)
    import inspect
    import textwrap
    original = inspect.getsource(GMM2D.__init__)
    patched = replace_once(original,
        'batch_shape=log_pis.shape[0], event_shape=log_pis.shape[1:]',
        'batch_shape=log_pis.shape[:-1], event_shape=torch.Size([2]), validate_args=False')
    scope = dict(gmm.__dict__)
    exec(textwrap.dedent(patched), scope)
    GMM2D.__init__ = scope['__init__']
    original_decoder = inspect.getsource(MultimodalGenerativeCVAE.p_y_xz)
    patched_decoder = replace_once(original_decoder,
        'num_samples, num_components=1, gmm_mode=False):',
        'num_samples, num_components=1, gmm_mode=False, return_distribution_only=False):')
    patched_decoder = replace_once(patched_decoder,
        '        if mode == ModeKeys.PREDICT:\n            if gmm_mode:',
        '        if return_distribution_only:\n            return y_dist, None\n\n        if mode == ModeKeys.PREDICT:\n            if gmm_mode:')
    scope = dict(mgcvae.__dict__)
    exec(textwrap.dedent(patched_decoder), scope)
    MultimodalGenerativeCVAE.p_y_xz = scope['p_y_xz']
    PATCHES = {name:hashlib.sha256(text.encode()).hexdigest() for name,text in
            [('gmm_original',original),('gmm_adapted',patched),
             ('decoder_original',original_decoder),('decoder_adapted',patched_decoder)]}
    return PATCHES

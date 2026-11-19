"""Trim point-to-plane training correspondences before solving, retaining independent held-out checks."""
from common import dump, read, sha
from overlay_r08 import overlay as original_overlay


def derive(source):
    # All matches, held-out landmarks, multistart ambiguity, GNSS, jumps and temporal gates
    # are still evaluated by the original localizer. Only the point-to-plane least-squares fit changes.
    old = 'ix=self.planar_ids[indices];mask=ds<.6'
    new = """ix=self.planar_ids[indices]
                # Exclude the poorest training correspondences in locally imprecise frozen map surfaces.
                # Quantile and residual weights are computed from the fitting half only.
                distance_limit=min(.6, float(np.quantile(ds, .60))+1e-9)
                mask=ds<distance_limit"""
    old_weight = 'weight=np.minimum(1.,.08/np.maximum(np.abs(residual),1e-9))'
    if source.count(old)!=1 or source.count(old_weight)!=1:
        raise ValueError('Unexpected version of the point-to-plane matcher')
    source=source.replace(old,new).replace(old_weight,old_weight.replace('.08/', '.04/'))
    compile(source,'regional_math.py','exec')
    return source


def overlay(task,out):
    path=original_overlay(task,out)
    target=path/'regional_math.py'
    target.write_text(derive(target.read_text()))
    manifest=read(path/'adaptation.json')
    manifest['generated']['regional_math.py']=sha(target)
    manifest['connector_registration']=dict(training_distance_quantile=.60,huber_scale_m=.04,
        minimum_training_inliers=100,heldout_alternating_points=True,heldout_overlap=.55,
        heldout_p95_m=.30,gnss_innovation_m=.15,gnss_nis=9.21)
    dump(path/'adaptation.json',manifest)
    return path

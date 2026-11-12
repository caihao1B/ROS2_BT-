from common14 import *
import html,shutil

def render(out,data):
    manifest=BASE/'maps/s14_regions/manifest.json'
    if manifest.exists():
        a=read(manifest);data['registration']=dict(map_id=a['map_id'],frame_epoch=a['frame_epoch'],atlas_revision=a['revision'],scope='A1/B1 only; independent station validation is not full mission completion',regions={k:{q:v[q] for q in ['passed','train_residuals_m','heldout_residuals_m','heldout_rmse_m']} for k,v in a['regions'].items()})
        picture=manifest.parent/'registration_validation.png'
        if picture.exists():shutil.copy2(picture,out/picture.name)
        picture=manifest.parent/'navigation_validation.png'
        if picture.exists():shutil.copy2(picture,out/picture.name)
    payload=json.dumps(data,ensure_ascii=False).replace('</','<\\/')
    template=(PKG/'scripts/report_template.html').read_text()
    (out/'index.html').write_text(template.replace('__PAYLOAD__',payload))

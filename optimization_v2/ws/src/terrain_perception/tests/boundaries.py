"""Validate declared navigation permission without simulator obstacle knowledge."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common09 import *
from shapely.geometry import shape,Point,box
from shapely.ops import unary_union
areas=[]
for file in ['workspaces.geojson','connector.geojson']:
 areas.extend(shape(x['geometry']) for x in read(BASE/'maps/prior'/file)['features'] if x['geometry']['type'] in ['Polygon','MultiPolygon'])
apron=unary_union([shape(x['geometry']) for x in read(BASE/'config/perception_test_area.geojson')['features']]);work=unary_union(areas);allowed=unary_union([work,apron]);checks=dict(valid=apron.is_valid,apron_connects_to_work=apron.intersects(work),old_fixture_outside_work=not work.contains(Point(30.5,15.3)),fixture_inside_authorized_apron=apron.buffer(-.62).contains(Point(30.5,15.3)),outside_remains_forbidden=not allowed.contains(Point(24,10)),temporary_objects_not_in_boundary_source=all(not x['properties']['obstacle_geometry_included'] for x in read(BASE/'config/perception_test_area.geojson')['features']))
print(json.dumps(checks));raise SystemExit(0 if all(checks.values()) else 1)

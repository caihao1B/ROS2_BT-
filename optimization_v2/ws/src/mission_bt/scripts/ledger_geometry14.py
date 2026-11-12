"""Area accounting ignores zero-area contacts, retaining all polygonal parts.

Fixed-precision overlays may leave line/point remnants at a target boundary.
They represent zero mowing area and must not enter a polygon overlay as mixed
dimensional GeometryCollections. No buffering, hull, repair, or area expansion.
"""
import math
from shapely.geometry import Polygon
from shapely import set_precision, union_all

def polygon_parts(geometry):
    if geometry.is_empty:return []
    if not geometry.is_valid:raise ValueError('INVALID_LEDGER_GEOMETRY')
    if geometry.geom_type=='Polygon':return [geometry]
    if geometry.geom_type in ['MultiPolygon','GeometryCollection']:
        return [p for part in geometry.geoms for p in polygon_parts(part)]
    if geometry.geom_type in ['Point','MultiPoint','LineString','LinearRing','MultiLineString']:return []
    raise ValueError('UNSUPPORTED_LEDGER_GEOMETRY '+geometry.geom_type)

def area_union(credit,footprint,precision):
    if not math.isfinite(precision) or precision<=0:raise ValueError('INVALID_LEDGER_PRECISION')
    parts=[p for source in [credit,footprint] for part in polygon_parts(source)
           for p in polygon_parts(set_precision(part,precision))]
    return union_all(parts,grid_size=precision) if parts else Polygon()

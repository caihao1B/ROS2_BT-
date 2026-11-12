"""Explicit future DARP interface; no DARP result or theoretical guarantee fabricated."""
from geometry11 import components

def export_uniform_grid(graph,seed_units,cell_size_m):
    """Only original-like uniform, connected 4-neighbour grid inputs are eligible.
    Irregular partial cells and time-weighted units require a deliberate adapter.
    Returning this request does NOT run DARP or establish curvature feasibility.
    """
    if len(set(seed_units))!=len(seed_units) or any(i<0 or i>=len(graph['cells']) for i in seed_units):raise ValueError('DARP_SEEDS')
    if any(abs(g.area-cell_size_m**2)>1e-8 for g in graph['cells']):raise ValueError('DARP_PARTIAL_CELLS_NEED_ADAPTER')
    if max(graph['weights'])-min(graph['weights'])>1e-8:raise ValueError('DARP_TIME_WEIGHTS_NEED_ADAPTER')
    if len(components(graph['adj']))!=1:raise ValueError('DARP_DISCONNECTED')
    return {'schema':'S11-DARP-REQUEST/1','status':'INTERFACE_ONLY_NOT_EXECUTED','grid_cells':graph['grid'],
      'seed_units':seed_units,'blocked':'all grid cells absent from grid_cells',
      'assumptions':['uniform work per cell','four-neighbour cell connectivity','no Ackermann curvature guarantee','one vehicle sequential clusters differs from simultaneous robots']}

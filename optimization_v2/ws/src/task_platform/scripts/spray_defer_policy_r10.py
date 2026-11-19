"""A deferred spray point carries no dose and may be released only by the native BT."""

def selected_points(allpoints, ids):
    source = [p['id'] for p in allpoints]
    if not ids or len(set(ids)) != len(ids) or ids[0] not in source:
        raise ValueError('INVALID_ORIGINAL_POINT_SELECTION')
    begin = source.index(ids[0])
    if source[begin:begin+len(ids)] != ids:
        raise ValueError('SELECTION_IS_NOT_A_CONTIGUOUS_ORIGINAL_INTERVAL')
    return allpoints[begin:begin+len(ids)]


def exhausted_dock(events, point, limit):
    """A refused proposal or a localization hold is not an executed motion attempt."""
    local = [e for e in events if e.get('point') == point]
    starts = [e for e in local if e['event'] == 'LOCAL_DOCK_GOAL']
    outcomes = [e for e in local if e['event'] == 'LOCAL_DOCK_NAV_FAILED']
    attempts = []
    for i,start in enumerate(starts):
        stop = starts[i+1]['t'] if i+1<len(starts) else float('inf')
        failed = next((e for e in outcomes if start['t'] < e['t'] < stop),None)
        if failed:
            attempts.append(dict(attempt=start['attempt'],sent_t=start['t'],finished_t=failed['t'],
                                 path_sha256=start['path_sha256'],status=failed.get('status'),error_code=failed.get('error_code')))
    latest = next((e for e in reversed(local) if e['event']=='DOCKING_SETTLED_CHECK'),{})
    good = (limit>0 and len(attempts)==limit and [a['attempt'] for a in attempts]==list(range(limit)) and
            latest.get('attempt')==limit and latest.get('stable') is False and
            all(a['finished_t']<=latest['t'] for a in attempts))
    return dict(eligible=good,attempts=attempts,last_stopped_observation=latest.get('stability',{}))


def defer_admission(st,cfg,spec,toolcfg,active,remaining,require_dispatch=True):
    now = st['t']; bt=st.get('bt') or {};ev=bt.get('evidence') or {};loc=st.get('loc') or {};risk=st.get('safety') or {};g=st.get('gate') or {};fb=st.get('feedback') or {};tool=st.get('tool') or {}
    def fresh(row,limit):return -.03 <= now-row.get('t',-100) < limit
    verdict = dict(no_live_action=not active,other_original_point_exists=bool(remaining),worker_navigation_idle=bool(st.get('nav_idle')),
        correct_session=bt.get('session')==spec['session'],correct_version=bt.get('task_version')==cfg['version'] and ev.get('bindings_valid') is True,
        bt_fresh=fresh(bt,.3) and not ev.get('stale',True),bt_no_emergency=bt.get('action') not in ('STOP','HUMAN') and not bt.get('manual_latched',True),
        map_pose_admitted=fresh(loc,.15) and loc.get('valid') is True and not (loc.get('native_supervisor') or {}).get('plan_revalidation_required',True),
        risk_admitted=fresh(risk,.45) and risk.get('valid') is True and risk.get('boundary_valid') is True and risk.get('localization_epoch')==loc.get('localization_epoch'),
        measured_body_tool_stop=fresh(g,.15) and all(g.get(k) is True for k in ('stopped','tool_stopped','nav_idle')),
        tool_server_stopped=fresh(tool,.15) and tool.get('stopped_confirmed') is True,
        independent_tool_feedback=fresh(fb,toolcfg['feedback_age_sim_s']) and fb.get('healthy') is True and fb.get('request_on') is False and fb.get('tool_revision')==toolcfg['tool_revision'] and abs(fb.get('rpm_rad_s',1000))<toolcfg['stopped_max_rad_s'] and fb.get('flow')==0.)
    if require_dispatch:
        verdict['native_defer']=(bt.get('action')=='DEFER' and bt.get('reason_code')=='TRY_OTHER_OBLIGATION_RETAIN_RESIDUAL' and bt.get('stopped_confirmation') is True and ev.get('alternative') is True and ev.get('nav_idle') is True and ev.get('stopped') is True and ev.get('tool_stopped') is True and isinstance(bt.get('dispatch_id'),int))
    return verdict

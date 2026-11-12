"""Bounded hot-restart discovery and lifecycle handshake; caller keeps lease inactive."""
import time
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Trigger


def bringup_after_restart(node, tick, sim_time, audit_ready, log):
    start = sim_time()
    deadline = time.monotonic() + 60.0
    names = ['controller_server', 'planner_server', 'bt_navigator']
    specs = {name: (GetState, '/' + name + '/get_state') for name in names}
    specs['manager'] = (Trigger, '/lifecycle_manager/is_active')
    specs['startup'] = (ManageLifecycleNodes, '/lifecycle_manager/manage_nodes')
    clients = {name: node.create_client(*spec) for name, spec in specs.items()}

    def spin():
        if sim_time() - start > 20.0 or time.monotonic() > deadline:
            raise TimeoutError('NAV_RESTART_HANDSHAKE_DEADLINE')
        tick()

    def query(name):
        # GetState / is_active are read-only. A lost reply is safe to retry.
        while True:
            client = clients[name]
            while not client.service_is_ready():
                spin()
            future = client.call_async(specs[name][0].Request())
            log('lifecycle_query_sent', node=name)
            sent = sim_time()
            sent_wall = time.monotonic()
            while not future.done() and sim_time() - sent < 1.0 and time.monotonic() - sent_wall < 4.0:
                spin()
            if future.done() and future.result() is not None:
                return future.result()
            future.cancel()
            log('lifecycle_query_retry', node=name, reason='RESPONSE_NOT_RECEIVED')
            node.destroy_client(client)
            clients[name] = node.create_client(*specs[name])
            spin()

    try:
        states = {name: query(name).current_state.id for name in names}
        if any(state != 1 for state in states.values()) or query('manager').success:
            raise RuntimeError('RESTART_EXPECTS_UNCONFIGURED_NODES')
        # The manager created its internal clients with autostart=false. Keep
        # all request/reply endpoints alive during a bounded discovery interval.
        ready_at = sim_time()
        while sim_time() - ready_at < 2.0:
            spin()
        states = {name: query(name).current_state.id for name in names}
        if any(state != 1 for state in states.values()) or query('manager').success:
            raise RuntimeError('UNEXPECTED_LIFECYCLE_TRANSITION')
        log('restart_discovery_ready', states=states, elapsed_sim_s=sim_time()-start)
        startup = clients['startup']
        while not startup.service_is_ready():
            spin()
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = startup.call_async(request)
        sent = sim_time()
        while not future.done() and sim_time()-sent < 8.0:
            spin()
        if future.done() and future.result() is not None and not future.result().success:
            raise RuntimeError('NAV_MANAGER_STARTUP_FAILED')
        if not future.done():
            future.cancel()
            log('lifecycle_startup_reply_missing', reason='VERIFY_ACTUAL_STATES_WITHOUT_REPLAYING_TRANSITION')
        for name in names:
            while query(name).current_state.id != 3:
                spin()
        while not query('manager').success or not audit_ready():
            spin()
        log('nav_lifecycle_active_verified')
    finally:
        for client in clients.values():
            node.destroy_client(client)

"""Hold zero-command discovery until the existing startup deadline expires."""
def pending_stale_context(initialized,reason,t,startup_timeout):
    return not initialized and reason=='INPUT_STALE' and 0<=t<startup_timeout

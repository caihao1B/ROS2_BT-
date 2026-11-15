"""Large observer images get their own writer buffer; autonomy is unchanged."""
from transport_r08 import configure_transport,transport_environment as original_environment


def transport_environment(role,inherited):
    env=original_environment(role,inherited)
    if role=='video_bridge':
        profile=env['R08_CLOUD_TRANSPORT_PROFILE']
        env.update(FASTRTPS_DEFAULT_PROFILES_FILE=profile,
                   FASTDDS_DEFAULT_PROFILES_FILE=profile)
    return env

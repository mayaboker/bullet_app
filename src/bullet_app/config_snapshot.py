"""One snapshot contract for periodic publication and runtime parameter edits."""
from dataclasses import asdict


def configuration_snapshot(config, source, revisions, files=None):
    return dict(schema_version=1, source=source, role='app', settings=asdict(config),
                revisions=revisions, files=dict(files or {}))

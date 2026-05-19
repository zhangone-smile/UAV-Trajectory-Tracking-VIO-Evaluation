#!/bin/bash
# launch-prefix helper: poll for $1 to appear (up to ~40s), then exec the rest.
# Used so vins_node does not race the auto-staging node that writes its yaml.
set -u
target="${1:?wait_for_file: missing target path}"
shift
i=0
while [ ! -f "$target" ] && [ "$i" -lt 200 ]; do
    sleep 0.2
    i=$((i + 1))
done
if [ ! -f "$target" ]; then
    echo "_wait_for_file.sh: timed out waiting for $target" >&2
fi
exec "$@"

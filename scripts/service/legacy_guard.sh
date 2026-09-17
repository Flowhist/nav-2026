#!/usr/bin/env bash
# Source before any legacy cleanup or hardware startup.
FINAV_SERVICE_CONFIG="${XDG_CONFIG_HOME:-$HOME/.config}/finav/services.json"
if [[ -f "$FINAV_SERVICE_CONFIG" ]]; then
    printf 'Finav is managed by user services. Use systemctl --user start finav.target (or base-control.service).\n' >&2
    printf 'Refusing to start a second controller or run legacy process cleanup.\n' >&2
    exit 1
fi

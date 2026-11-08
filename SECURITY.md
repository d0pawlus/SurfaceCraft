# Security

SurfaceCraft reads untrusted files (meshes, projects, STEP) and untrusted JSON (scripts, MCP).
Readers are written to refuse malformed input without crashing; a panic or excessive allocation
on some input is a security bug. Please report it privately to the repository owner rather than
in a public issue.

The MCP server and the desktop control channel listen only on stdio or `127.0.0.1` and require a
per-session token for bridged connections.

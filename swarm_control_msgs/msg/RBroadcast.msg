# RBroadcast.msg
# Decentralized reference broadcast message
# Used by leader agents to share the external reference trajectory r(t)

# --- Header fields ---
string id                          # Unique identifier of broadcasting agent (e.g., "agent_1")
builtin_interfaces/Time stamp      # Time of broadcast creation

# --- Reference trajectory ---
geometry_msgs/Point point   # Reference r(t): point
# Info.msg
# Generic status and metadata for each robot in the swarm.

# --- Identification ---
string id                      # Unique ID (e.g., "bot1")
string role                    # "leader" or "agent"
int8 component_id

# --- Timestamp ---
builtin_interfaces/Time stamp  # Time when this status was generated

# --- Health / diagnostics ---
bool is_active                 # True if participating in current control cycle
string status_msg              # Short description, e.g. "OK", "LowBattery", "SensorError"

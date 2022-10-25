# HEADER
uint32 header_id         # header ID of the message. The headerId is defined per topic and incremented by 1 with each sent
                        # (but not necessarily received) message.

string timestamp        # Timestamp after ISO8601 in the format YYYY-MM-DDTHH:mm:ss.ssZ (e.g.“2017-04-15T11:40:03.12Z”)

string version          # Version of the protocol [Major].[Minor].[Patch] (e.g. 1.3.2)

string manufacturer     # Manufacturer of the AGV

string serial_number    # Serial Number of the AGV

# CONTENTS
AGVPosition agv_position # The AGV's position

Velocity velocity       # The AGV's velocity in vehicle coordinates

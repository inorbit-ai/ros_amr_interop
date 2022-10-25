# HEADER
uint32 header_id         # header ID of the message. The headerId is defined per topic and incremented by 1 with each sent
                        # (but not necessarily received) message.

string timestamp        # Timestamp after ISO8601 in the format YYYY-MM-DDTHH:mm:ss.ssZ (e.g.“2017-04-15T11:40:03.12Z”)

string version          # Version of the protocol [Major].[Minor].[Patch] (e.g. 1.3.2)

string manufacturer     # Manufacturer of the AGV

string serial_number    # Serial Number of the AGV

# CONTENTS
string connection_state # Enum{ONLINE, OFFLINE, CONNECTIONBROKEN}
                        # ONLINE: connection between AGV and broker is active.
                        # OFFLINE: connection between AGV and broker has gone offline in a coordinated way.
                        # CONNECTIONBROKEN: The connection between  AGV and  broker  has unexpectedly ended.


string ONLINE="ONLINE"
string OFFLINE="OFFLINE"
string CONNECTIONBROKEN="CONNECTIONBROKEN"

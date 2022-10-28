float64 battery_charge     # [%] State of Charge: if AGV only provides values for good or bad battery levels, these will
                           # be indicated as 20% (bad) and 80% (good).
float64 battery_voltage    # [V] Battery Voltage
int8 battery_health        # [%] State of Health
bool charging              # True: charging in progress False: AGV is currently not charging
uint32 reach               # Estimated reach with current SoC

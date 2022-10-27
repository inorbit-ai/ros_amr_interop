# Action the AGV can perform.

string action_type                                              # Name of action as described in the first column
                                                                # of "Actions and Parameters".
                                                                # Identifies the function of the action.

string action_id                                                # Unique ID to identify the action and map them to
                                                                # the actionState in the state.
                                                                # Suggestion: Use UUIDs.

string action_description                                       # Additional information on the action

string blocking_type                                            # Enum {NONE, SOFT, HARD}
                                                                # “NONE” – allows driving and other actions
                                                                # “SOFT” - allows other actions, but not driving
                                                                # “HARD” - is the only allowd action at that time


vda5050_msgs/ActionParameter[] action_parameters                # Array of actionParameter objects for the indicated
                                                                # action e. g. deviceId, loadId, external Triggers.
                                                                # See “Actions and Parameters”.
                                                                #
                                                                # Note on Porting to ROS:
                                                                # Since those parameter vary in type but their
                                                                # serialization is always a json dictionary with "key"
                                                                # and "value" we decided to serialize the value as
                                                                # string. This way the (de-)serialization has to be done by
                                                                # the user depending on the key, but the protocol is met


# Enums for blockingType
string NONE = "NONE"
string SOFT = "SOFT"
string HARD = "HARD"

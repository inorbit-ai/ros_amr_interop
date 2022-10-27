string action_type                              # Unique actionType corresponding to action.actionType
string action_description
string[] action_scopes                          # Allowed scopes for using this action-type
ActionParameterDefinition[] action_parameters   # List of parameters defined for the action
string result_description

# Enums for action scopes
string INSTANT = "INSTANT"
string NODE = "NODE"
string EDGE = "EDGE"

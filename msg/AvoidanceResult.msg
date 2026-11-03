# The header and lifetime are copied from the request, never renewed by rechecking.
std_msgs/Header header
string session
uint64 sequence
string source
geometry_msgs/Twist velocity
builtin_interfaces/Duration valid_for
string reason

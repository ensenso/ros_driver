# The error code of the exception. For a list of possible codes, see
# https://www.ensenso.com/manual/index.html?api_return_codes.htm
# In addition, we use the following error codes to indicate errors from
# the ROS interface:
#   100: Unknown exception
#   101: TF exception
uint8 code
string message
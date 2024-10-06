# This message is composed of these fields:
#
#    first_name: of type string
#
#    last_name: of type string
#
#    phone_number: of type string
#
#    phone_type: of type uint8, with several named constant values defined
#
# Note that it’s possible to set default values for fields within a message definition.

uint8 PHONE_TYPE_HOME=0
uint8 PHONE_TYPE_WORK=1
uint8 PHONE_TYPE_MOBILE=2

string first_name
string last_name
string phone_number
uint8 phone_type
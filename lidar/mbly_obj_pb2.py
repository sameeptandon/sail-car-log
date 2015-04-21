# Generated by the protocol buffer compiler.  DO NOT EDIT!

from google.protobuf import descriptor
from google.protobuf import message
from google.protobuf import reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)



DESCRIPTOR = descriptor.FileDescriptor(
  name='mbly_obj.proto',
  package='mbly',
  serialized_pb='\n\x0embly_obj.proto\x12\x04mbly\"\xe1\x08\n\x06Object\x12\x11\n\ttimestamp\x18\x01 \x02(\x04\x12\x0e\n\x06obj_id\x18\x02 \x02(\r\x12\r\n\x05pos_x\x18\x03 \x02(\x02\x12\r\n\x05pos_y\x18\x04 \x02(\x02\x12\x11\n\trel_vel_x\x18\x05 \x02(\x02\x12#\n\x08obj_type\x18\x06 \x02(\x0e\x32\x11.mbly.Object.Type\x12#\n\x06status\x18\x07 \x02(\x0e\x32\x13.mbly.Object.Status\x12%\n\x07\x62raking\x18\x08 \x02(\x0e\x32\x14.mbly.Object.Braking\x12\'\n\x08location\x18\t \x02(\x0e\x32\x15.mbly.Object.Location\x12%\n\x07\x62linker\x18\n \x02(\x0e\x32\x14.mbly.Object.Blinker\x12!\n\x05valid\x18\x0b \x02(\x0e\x32\x12.mbly.Object.Valid\x12\x0e\n\x06length\x18\x0c \x02(\x02\x12\r\n\x05width\x18\r \x02(\x02\x12\x0b\n\x03\x61ge\x18\x0e \x02(\r\x12\x1f\n\x04lane\x18\x0f \x02(\x0e\x32\x11.mbly.Object.Lane\x12\x16\n\x0e\x61\x63\x63\x65leration_x\x18\x10 \x02(\x02\"t\n\x04Type\x12\x0b\n\x07Vehicle\x10\x00\x12\t\n\x05Truck\x10\x01\x12\x08\n\x04\x42ike\x10\x02\x12\x07\n\x03Ped\x10\x03\x12\x0b\n\x07\x42icycle\x10\x04\x12\x10\n\x0cUnused_type5\x10\x05\x12\x10\n\x0cUnused_type6\x10\x06\x12\x10\n\x0cUnused_type7\x10\x07\"\x81\x01\n\x06Status\x12\x12\n\x0eUndefined_stat\x10\x00\x12\x0c\n\x08Standing\x10\x01\x12\x0b\n\x07Stopped\x10\x02\x12\n\n\x06Moving\x10\x03\x12\x0c\n\x08Oncoming\x10\x04\x12\n\n\x06Parked\x10\x05\x12\x10\n\x0cUnused_stat6\x10\x06\x12\x10\n\x0cUnused_stat7\x10\x07\":\n\x07\x42raking\x12 \n\x1cLights_off_or_not_identified\x10\x00\x12\r\n\tLights_on\x10\x01\"\xac\x01\n\x08Location\x12\x12\n\x0eUndefined_loc0\x10\x00\x12\x10\n\x0cIn_host_lane\x10\x01\x12\x11\n\rOut_host_lane\x10\x02\x12\n\n\x06\x43ut_in\x10\x03\x12\x0b\n\x07\x43ut_out\x10\x04\x12\x12\n\x0eUndefined_loc4\x10\x04\x12\x12\n\x0eUndefined_loc5\x10\x05\x12\x12\n\x0eUndefined_loc6\x10\x06\x12\x12\n\x0eUndefined_loc7\x10\x07\"B\n\x07\x42linker\x12\x0f\n\x0bUnavailable\x10\x00\x12\x07\n\x03Off\x10\x01\x12\x08\n\x04Left\x10\x02\x12\t\n\x05Right\x10\x03\x12\x08\n\x04\x42oth\x10\x04\"M\n\x05Valid\x12\x11\n\rUnused_valid0\x10\x00\x12\r\n\tNew_valid\x10\x01\x12\x0f\n\x0bOlder_valid\x10\x02\x12\x11\n\rUnused_valid3\x10\x03\"B\n\x04Lane\x12\x10\n\x0cNot_assigned\x10\x00\x12\x0c\n\x08\x45go_lane\x10\x01\x12\r\n\tNext_lane\x10\x02\x12\x0b\n\x07Invalid\x10\x03\"\'\n\x07Objects\x12\x1c\n\x06object\x18\x01 \x03(\x0b\x32\x0c.mbly.Object')



_OBJECT_TYPE = descriptor.EnumDescriptor(
  name='Type',
  full_name='mbly.Object.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='Vehicle', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Truck', index=1, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Bike', index=2, number=2,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Ped', index=3, number=3,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Bicycle', index=4, number=4,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Unused_type5', index=5, number=5,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Unused_type6', index=6, number=6,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Unused_type7', index=7, number=7,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=448,
  serialized_end=564,
)

_OBJECT_STATUS = descriptor.EnumDescriptor(
  name='Status',
  full_name='mbly.Object.Status',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='Undefined_stat', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Standing', index=1, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Stopped', index=2, number=2,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Moving', index=3, number=3,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Oncoming', index=4, number=4,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Parked', index=5, number=5,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Unused_stat6', index=6, number=6,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Unused_stat7', index=7, number=7,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=567,
  serialized_end=696,
)

_OBJECT_BRAKING = descriptor.EnumDescriptor(
  name='Braking',
  full_name='mbly.Object.Braking',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='Lights_off_or_not_identified', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Lights_on', index=1, number=1,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=698,
  serialized_end=756,
)

_OBJECT_LOCATION = descriptor.EnumDescriptor(
  name='Location',
  full_name='mbly.Object.Location',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='Undefined_loc0', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='In_host_lane', index=1, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Out_host_lane', index=2, number=2,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Cut_in', index=3, number=3,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Cut_out', index=4, number=4,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Undefined_loc4', index=5, number=4,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Undefined_loc5', index=6, number=5,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Undefined_loc6', index=7, number=6,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Undefined_loc7', index=8, number=7,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=759,
  serialized_end=931,
)

_OBJECT_BLINKER = descriptor.EnumDescriptor(
  name='Blinker',
  full_name='mbly.Object.Blinker',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='Unavailable', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Off', index=1, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Left', index=2, number=2,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Right', index=3, number=3,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Both', index=4, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=933,
  serialized_end=999,
)

_OBJECT_VALID = descriptor.EnumDescriptor(
  name='Valid',
  full_name='mbly.Object.Valid',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='Unused_valid0', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='New_valid', index=1, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Older_valid', index=2, number=2,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Unused_valid3', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1001,
  serialized_end=1078,
)

_OBJECT_LANE = descriptor.EnumDescriptor(
  name='Lane',
  full_name='mbly.Object.Lane',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='Not_assigned', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Ego_lane', index=1, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Next_lane', index=2, number=2,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='Invalid', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1080,
  serialized_end=1146,
)


_OBJECT = descriptor.Descriptor(
  name='Object',
  full_name='mbly.Object',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='timestamp', full_name='mbly.Object.timestamp', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='obj_id', full_name='mbly.Object.obj_id', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='pos_x', full_name='mbly.Object.pos_x', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='pos_y', full_name='mbly.Object.pos_y', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='rel_vel_x', full_name='mbly.Object.rel_vel_x', index=4,
      number=5, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='obj_type', full_name='mbly.Object.obj_type', index=5,
      number=6, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='status', full_name='mbly.Object.status', index=6,
      number=7, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='braking', full_name='mbly.Object.braking', index=7,
      number=8, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='location', full_name='mbly.Object.location', index=8,
      number=9, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='blinker', full_name='mbly.Object.blinker', index=9,
      number=10, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='valid', full_name='mbly.Object.valid', index=10,
      number=11, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='length', full_name='mbly.Object.length', index=11,
      number=12, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='width', full_name='mbly.Object.width', index=12,
      number=13, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='age', full_name='mbly.Object.age', index=13,
      number=14, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='lane', full_name='mbly.Object.lane', index=14,
      number=15, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='acceleration_x', full_name='mbly.Object.acceleration_x', index=15,
      number=16, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _OBJECT_TYPE,
    _OBJECT_STATUS,
    _OBJECT_BRAKING,
    _OBJECT_LOCATION,
    _OBJECT_BLINKER,
    _OBJECT_VALID,
    _OBJECT_LANE,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=25,
  serialized_end=1146,
)


_OBJECTS = descriptor.Descriptor(
  name='Objects',
  full_name='mbly.Objects',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='object', full_name='mbly.Objects.object', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=1148,
  serialized_end=1187,
)

_OBJECT.fields_by_name['obj_type'].enum_type = _OBJECT_TYPE
_OBJECT.fields_by_name['status'].enum_type = _OBJECT_STATUS
_OBJECT.fields_by_name['braking'].enum_type = _OBJECT_BRAKING
_OBJECT.fields_by_name['location'].enum_type = _OBJECT_LOCATION
_OBJECT.fields_by_name['blinker'].enum_type = _OBJECT_BLINKER
_OBJECT.fields_by_name['valid'].enum_type = _OBJECT_VALID
_OBJECT.fields_by_name['lane'].enum_type = _OBJECT_LANE
_OBJECT_TYPE.containing_type = _OBJECT;
_OBJECT_STATUS.containing_type = _OBJECT;
_OBJECT_BRAKING.containing_type = _OBJECT;
_OBJECT_LOCATION.containing_type = _OBJECT;
_OBJECT_BLINKER.containing_type = _OBJECT;
_OBJECT_VALID.containing_type = _OBJECT;
_OBJECT_LANE.containing_type = _OBJECT;
_OBJECTS.fields_by_name['object'].message_type = _OBJECT
DESCRIPTOR.message_types_by_name['Object'] = _OBJECT
DESCRIPTOR.message_types_by_name['Objects'] = _OBJECTS

class Object(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _OBJECT
  
  # @@protoc_insertion_point(class_scope:mbly.Object)

class Objects(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _OBJECTS
  
  # @@protoc_insertion_point(class_scope:mbly.Objects)

# @@protoc_insertion_point(module_scope)

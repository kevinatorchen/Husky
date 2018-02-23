// Auto-generated. Do not edit!

// (in-package extractor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let feature = require('./feature.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class map {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.features = null;
      this.robotposition = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('features')) {
        this.features = initObj.features
      }
      else {
        this.features = [];
      }
      if (initObj.hasOwnProperty('robotposition')) {
        this.robotposition = initObj.robotposition
      }
      else {
        this.robotposition = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type map
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [features]
    // Serialize the length for message field [features]
    bufferOffset = _serializer.uint32(obj.features.length, buffer, bufferOffset);
    obj.features.forEach((val) => {
      bufferOffset = feature.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [robotposition]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.robotposition, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type map
    let len;
    let data = new map(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [features]
    // Deserialize array length for message field [features]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.features = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.features[i] = feature.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [robotposition]
    data.robotposition = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.features.forEach((val) => {
      length += feature.getMessageSize(val);
    });
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'extractor/map';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '605ba2556bdcd481e241a16199e3f3a4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    feature[] features
    geometry_msgs/Pose robotposition
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: extractor/feature
    Header header
    geometry_msgs/Point position
    float64 diameter
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new map(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.features !== undefined) {
      resolved.features = new Array(msg.features.length);
      for (let i = 0; i < resolved.features.length; ++i) {
        resolved.features[i] = feature.Resolve(msg.features[i]);
      }
    }
    else {
      resolved.features = []
    }

    if (msg.robotposition !== undefined) {
      resolved.robotposition = geometry_msgs.msg.Pose.Resolve(msg.robotposition)
    }
    else {
      resolved.robotposition = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = map;

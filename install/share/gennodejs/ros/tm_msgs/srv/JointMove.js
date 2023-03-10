// Auto-generated. Do not edit!

// (in-package tm_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class JointMoveRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.jointTarget = null;
    }
    else {
      if (initObj.hasOwnProperty('jointTarget')) {
        this.jointTarget = initObj.jointTarget
      }
      else {
        this.jointTarget = new sensor_msgs.msg.JointState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointMoveRequest
    // Serialize message field [jointTarget]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.jointTarget, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointMoveRequest
    let len;
    let data = new JointMoveRequest(null);
    // Deserialize message field [jointTarget]
    data.jointTarget = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.JointState.getMessageSize(object.jointTarget);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tm_msgs/JointMoveRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cdca231acb928c832450acedf16ebc56';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/JointState jointTarget
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
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
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointMoveRequest(null);
    if (msg.jointTarget !== undefined) {
      resolved.jointTarget = sensor_msgs.msg.JointState.Resolve(msg.jointTarget)
    }
    else {
      resolved.jointTarget = new sensor_msgs.msg.JointState()
    }

    return resolved;
    }
};

class JointMoveResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isOK = null;
    }
    else {
      if (initObj.hasOwnProperty('isOK')) {
        this.isOK = initObj.isOK
      }
      else {
        this.isOK = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointMoveResponse
    // Serialize message field [isOK]
    bufferOffset = _serializer.bool(obj.isOK, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointMoveResponse
    let len;
    let data = new JointMoveResponse(null);
    // Deserialize message field [isOK]
    data.isOK = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tm_msgs/JointMoveResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '497312386196714e5f4a62411b9bd684';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isOK
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointMoveResponse(null);
    if (msg.isOK !== undefined) {
      resolved.isOK = msg.isOK;
    }
    else {
      resolved.isOK = false
    }

    return resolved;
    }
};

module.exports = {
  Request: JointMoveRequest,
  Response: JointMoveResponse,
  md5sum() { return '07f94568e46947f8fdd61bbf89ecd59c'; },
  datatype() { return 'tm_msgs/JointMove'; }
};

// Auto-generated. Do not edit!

// (in-package tm_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class CartesianMoveRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.poseTarget = null;
    }
    else {
      if (initObj.hasOwnProperty('poseTarget')) {
        this.poseTarget = initObj.poseTarget
      }
      else {
        this.poseTarget = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianMoveRequest
    // Serialize message field [poseTarget]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.poseTarget, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianMoveRequest
    let len;
    let data = new CartesianMoveRequest(null);
    // Deserialize message field [poseTarget]
    data.poseTarget = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tm_msgs/CartesianMoveRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46373f83afb6c46513d3d786edd59f39';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose poseTarget
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
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
    const resolved = new CartesianMoveRequest(null);
    if (msg.poseTarget !== undefined) {
      resolved.poseTarget = geometry_msgs.msg.Pose.Resolve(msg.poseTarget)
    }
    else {
      resolved.poseTarget = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

class CartesianMoveResponse {
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
    // Serializes a message object of type CartesianMoveResponse
    // Serialize message field [isOK]
    bufferOffset = _serializer.bool(obj.isOK, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianMoveResponse
    let len;
    let data = new CartesianMoveResponse(null);
    // Deserialize message field [isOK]
    data.isOK = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tm_msgs/CartesianMoveResponse';
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
    const resolved = new CartesianMoveResponse(null);
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
  Request: CartesianMoveRequest,
  Response: CartesianMoveResponse,
  md5sum() { return '7c6bc7eb9aa36ece17e84459bdba640a'; },
  datatype() { return 'tm_msgs/CartesianMove'; }
};

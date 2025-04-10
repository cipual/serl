// Auto-generated. Do not edit!

// (in-package serl_franka_controllers.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ZeroJacobian {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.zero_jacobian = null;
    }
    else {
      if (initObj.hasOwnProperty('zero_jacobian')) {
        this.zero_jacobian = initObj.zero_jacobian
      }
      else {
        this.zero_jacobian = new Array(42).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ZeroJacobian
    // Check that the constant length array field [zero_jacobian] has the right length
    if (obj.zero_jacobian.length !== 42) {
      throw new Error('Unable to serialize array field zero_jacobian - length must be 42')
    }
    // Serialize message field [zero_jacobian]
    bufferOffset = _arraySerializer.float64(obj.zero_jacobian, buffer, bufferOffset, 42);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ZeroJacobian
    let len;
    let data = new ZeroJacobian(null);
    // Deserialize message field [zero_jacobian]
    data.zero_jacobian = _arrayDeserializer.float64(buffer, bufferOffset, 42)
    return data;
  }

  static getMessageSize(object) {
    return 336;
  }

  static datatype() {
    // Returns string type for a message object
    return 'serl_franka_controllers/ZeroJacobian';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '573da0494fbe019a7da2ae31329663cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[42] zero_jacobian
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ZeroJacobian(null);
    if (msg.zero_jacobian !== undefined) {
      resolved.zero_jacobian = msg.zero_jacobian;
    }
    else {
      resolved.zero_jacobian = new Array(42).fill(0)
    }

    return resolved;
    }
};

module.exports = ZeroJacobian;

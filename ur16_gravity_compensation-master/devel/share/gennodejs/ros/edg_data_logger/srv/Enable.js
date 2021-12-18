// Auto-generated. Do not edit!

// (in-package edg_data_logger.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class EnableRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.EnableDataLogging = null;
    }
    else {
      if (initObj.hasOwnProperty('EnableDataLogging')) {
        this.EnableDataLogging = initObj.EnableDataLogging
      }
      else {
        this.EnableDataLogging = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EnableRequest
    // Serialize message field [EnableDataLogging]
    bufferOffset = _serializer.bool(obj.EnableDataLogging, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EnableRequest
    let len;
    let data = new EnableRequest(null);
    // Deserialize message field [EnableDataLogging]
    data.EnableDataLogging = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'edg_data_logger/EnableRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b0c5dcc43b0317c4daaf806b552b4d3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool EnableDataLogging
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EnableRequest(null);
    if (msg.EnableDataLogging !== undefined) {
      resolved.EnableDataLogging = msg.EnableDataLogging;
    }
    else {
      resolved.EnableDataLogging = false
    }

    return resolved;
    }
};

class EnableResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.OutputFileName = null;
    }
    else {
      if (initObj.hasOwnProperty('OutputFileName')) {
        this.OutputFileName = initObj.OutputFileName
      }
      else {
        this.OutputFileName = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EnableResponse
    // Serialize message field [OutputFileName]
    bufferOffset = _serializer.string(obj.OutputFileName, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EnableResponse
    let len;
    let data = new EnableResponse(null);
    // Deserialize message field [OutputFileName]
    data.OutputFileName = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.OutputFileName.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'edg_data_logger/EnableResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4cb91e9e55bf61a0d4abe7a49e8f55de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string OutputFileName
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EnableResponse(null);
    if (msg.OutputFileName !== undefined) {
      resolved.OutputFileName = msg.OutputFileName;
    }
    else {
      resolved.OutputFileName = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: EnableRequest,
  Response: EnableResponse,
  md5sum() { return 'f3f19e69803c97782b2fdad054f37b22'; },
  datatype() { return 'edg_data_logger/Enable'; }
};

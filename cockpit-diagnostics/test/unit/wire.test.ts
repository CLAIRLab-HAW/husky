// SPDX-License-Identifier: LGPL-2.1-or-later
// Schema decoding uses the advertised encoding before serializing the actual CDR payload.
import assert from 'node:assert/strict';
import { MessageReader, MessageWriter } from '@foxglove/rosmsg2-serialization';
import { messageDefinitionsOf } from '../../src/roslib/Impl';

const message = { counter: 42, text: 'diagnostics → ready' };
const schemas = [
    { encoding: 'ros2msg', schema: 'uint32 counter\nstring text' },
    {
        encoding: 'ros2idl',
        schema: `
================================================================================
IDL: test_msgs/msg/Reading
module test_msgs {
  module msg {
    struct Reading {
      uint32 counter;
      string text;
    };
  };
};`,
    },
];

for (const { encoding, schema } of schemas) {
    const definitions = messageDefinitionsOf(schema, encoding);
    const serialized = new MessageWriter(definitions).writeMessage(message);
    assert.deepEqual(new MessageReader(definitions).readMessage(serialized), message, encoding);
}

console.log('wire: OK (ROS 2 message and IDL schemas both round-trip through CDR)');

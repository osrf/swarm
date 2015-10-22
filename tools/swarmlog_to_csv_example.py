#!/usr/bin/env python

import struct, sys, functools
from swarm import log_entry_pb2

# A simple class to read log files
class LogReader:
    def __init__(self, logfile):
        self.logfile = logfile
        self.stream = open(fname, 'rb')

    # Get the next message
    def next(self):
        # Read the 4-byte size field
        sizestring = self.stream.read(4)
        if len(sizestring) < 4:
            self.stream.close()
            return None
        size = struct.unpack('<I', sizestring)[0]
        msg = self.stream.read(size)
        # Make a protobuf message out of it
        pbmsg = log_entry_pb2.LogEntry()
        pbmsg.ParseFromString(msg)
        return pbmsg

    # Apply a given function to each message in the file
    def apply(self, func):
        while True:
            msg = self.next()
            if not msg:
                break
            func(msg)

# An example of processing a single log entry
def process_msg(entry):
    # Check for the 'incoming_msgs' field, which tells us about what happened to
    # messages that were sent
    if entry.HasField('incoming_msgs') and entry.HasField('time'):
        time = entry.time
        num_msgs_sent = 0
        num_recipients = 0
        num_msgs_delivered = 0
        drop_ratio = 0
        for msg in entry.incoming_msgs.message:
            num_msgs_sent += 1
            for neighbor in msg.neighbor:
                num_recipients += 1
                if neighbor.status == log_entry_pb2.DELIVERED:
                    num_msgs_delivered += 1
        if num_recipients > 0:
            drop_ratio = (num_recipients - num_msgs_delivered)/float(num_recipients)
        print('%f,%d,%d,%d,%f'%
          (time,num_msgs_sent, num_recipients, num_msgs_delivered, drop_ratio))

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Specify a file name to read from')
        sys.exit(1)
    fname = sys.argv[1]
    reader = LogReader(fname)
    print('# time, msg_sent, potential_recipients, msgs_delivered, drop_ratio')
    reader.apply(process_msg)

#!/usr/bin/env python

import struct, sys, functools
from swarm import log_entry_pb2, log_header_pb2

# A simple class to read log files
class LogReader:
    def __init__(self, logfile):
        self.logfile = logfile
        self.stream = open(fname, 'rb')
        self.header = None

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
        # The first message in the log is a LogHeader; the rest are LogEntry
        if not self.header:
            pbmsg = log_header_pb2.LogHeader()
            pbmsg.ParseFromString(msg)
            self.header = pbmsg
        else:
            pbmsg = log_entry_pb2.LogEntry()
            pbmsg.ParseFromString(msg)
        return pbmsg

    # Apply a given function to each message in the file
    def apply(self, func):
        first = True
        while True:
            msg = self.next()
            if not msg:
                break
            # Only pass through LogEntry messages (e.g., skip LogHeader
            # messages)
            if type(msg) == log_entry_pb2.LogEntry:
                func(msg)

# An example of processing a single log entry
def process_msg(entry):
    # Check for the 'incoming_msgs' field, which tells us about what happened to
    # messages that were sent
    if entry.HasField('incoming_msgs') and entry.HasField('time') and entry.HasField('visibility'):
        step_time = 0.01
        time = entry.time

        num_msgs_sent = 0
        num_unicast_sent = 0
        num_broadcast_sent = 0
        num_multicast_sent = 0
        freq_msgs_sent = 0
        num_recipients = 0
        num_msgs_delivered = 0
        drop_ratio = 0
        bytes_sent = 0
        data_rate = 0
        num_neighbors = 0
        num_robots = 0
        avg_num_neighbors = 0
        for msg in entry.incoming_msgs.message:
            if msg.dst_address == 'broadcast':
                num_broadcast_sent += 1
            elif msg.dst_address == 'multicast':
                num_multicast_sent += 1
            else:
                num_unicast_sent += 1

            num_msgs_sent += 1
            bytes_sent += msg.size + 56
            for neighbor in msg.neighbor:
                num_recipients += 1
                if neighbor.status == log_entry_pb2.DELIVERED:
                    num_msgs_delivered += 1

        for msg in entry.visibility.row:
            if msg.src == 'boo':
                continue

            num_robots += 1
            for neighbor in msg.entry:
                if neighbor.status == 1:
                    num_neighbors += 1

        if num_robots > 0:
            avg_num_neighbors = num_neighbors / float(num_robots)

        freq_msgs_sent = num_msgs_sent / step_time
        data_rate = (bytes_sent * 8) / step_time
        if num_recipients > 0:
            drop_ratio = (num_recipients - num_msgs_delivered)/float(num_recipients)
        print('%f,%d,%f,%d,%d,%d,%d,%d,%f,%d,%f,%f'%
          (time, num_msgs_sent, freq_msgs_sent, num_unicast_sent, num_broadcast_sent, num_multicast_sent, num_recipients, num_msgs_delivered, drop_ratio, bytes_sent, data_rate, avg_num_neighbors))

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Specify a file name to read from')
        sys.exit(1)
    fname = sys.argv[1]
    reader = LogReader(fname)
    print('# time, msg_sent, msg_freq, num_unicast, num_broadcast, num_multicast, potential_recipients, msgs_delivered, drop_ratio, bytes_sent, data_rate, avg_num_neighbors')
    reader.apply(process_msg)

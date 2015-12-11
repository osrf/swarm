#!/usr/bin/env python

import struct, sys, functools
from swarm import log_entry_pb2, log_header_pb2

# Global variables.
total_msgs_sent = 0
total_msgs_unicast = 0
total_msgs_broadcast = 0
total_msgs_multicast = 0
total_msgs_sent_freq = 0
counter_msgs_sent_freq = 0.0
total_drop_ratio = 0
counter_drop_ratio = 0.0
total_data_rate = 0
counter_data_rate = 0.0
total_neighbors = 0
counter_total_neighbors = 0.0
wrong_boo_reports = 0
succeed = False
duration = 0.0
time_step = 0.01

# A simple class to read log files
class LogReader:
    def __init__(self, logfile):
        self.logfile = logfile
        self.stream = open(fname, 'rb')
        self.header = None

    # Get the next message
    def next(self):
        global time_step

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
            if self.header.HasField("time_step"):
                time_step = self.header.time_step
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
    global total_msgs_sent
    global total_msgs_unicast
    global total_msgs_broadcast
    global total_msgs_multicast
    global total_msgs_sent_freq
    global counter_msgs_sent_freq
    global total_drop_ratio
    global counter_drop_ratio
    global total_data_rate
    global counter_data_rate
    global total_neighbors
    global counter_total_neighbors
    global wrong_boo_reports
    global succeed
    global duration
    global time_step

    # Check for the 'incoming_msgs' field, which tells us about what happened to
    # messages that were sent
    if entry.HasField('incoming_msgs') and entry.HasField('time') and entry.HasField('visibility'):
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

        freq_msgs_sent = num_msgs_sent / time_step
        data_rate = (bytes_sent * 8) / time_step
        if num_recipients > 0:
            drop_ratio = (num_recipients - num_msgs_delivered)/float(num_recipients)
        print('%f,%d,%f,%d,%d,%d,%d,%d,%f,%d,%f,%f'%
          (time, num_msgs_sent, freq_msgs_sent, num_unicast_sent, num_broadcast_sent, num_multicast_sent, num_recipients, num_msgs_delivered, drop_ratio, bytes_sent, data_rate, avg_num_neighbors))

        # Update global variables.
        total_msgs_sent += num_msgs_sent
        total_msgs_unicast += num_unicast_sent
        total_msgs_broadcast += num_broadcast_sent
        total_msgs_multicast += num_multicast_sent
        total_msgs_sent_freq += freq_msgs_sent
        counter_msgs_sent_freq += 1
        total_drop_ratio += drop_ratio
        counter_drop_ratio += 1
        total_data_rate += data_rate
        counter_data_rate += 1
        total_neighbors += avg_num_neighbors
        counter_total_neighbors += 1

    for boo_report in entry.boo_report:
        if boo_report.succeed:
            succeed = True
        else:
            wrong_boo_reports += 1

    if not succeed:
        duration = entry.time

def create_swarm_summary_report(log_reader, fullpath_file):
    global total_msgs_sent
    global total_msgs_unicast
    global total_msgs_broadcast
    global total_msgs_multicast
    global total_msgs_sent_freq
    global counter_msgs_sent_freq
    global total_drop_ratio
    global counter_drop_ratio
    global total_data_rate
    global counter_data_rate
    global total_neighbors
    global counter_total_neighbors
    global wrong_boo_reports
    global succeed
    global duration

    avg_msgs_sent_freq = total_msgs_sent_freq / counter_msgs_sent_freq
    avg_drop_ratio = 100.0 * total_drop_ratio / counter_drop_ratio
    # mbps.
    avg_data_rate = 0.000001 * total_data_rate / counter_data_rate
    avg_neighbors = total_neighbors / counter_total_neighbors

    fd = open(fullpath_file, "wb")

    team_name = 'Unknown'
    num_ground = 'Unknown'
    num_fixed = 'Unknown'
    num_rotor = 'Unknown'
    terrain_name = 'Unknown'
    vegetation_name = 'Unknown'
    search_area = 'Unknown'

    # Environment.
    if log_reader.header and log_reader.header.HasField("team_name"):
        team_name = log_reader.header.team_name
        # Escape '_' to make latex happy.
        team_name = team_name.replace('_', '\_')
    fd.write('\\newcommand{\swarmTeamName}{' + team_name + '}\n')

    if log_reader.header and log_reader.header.HasField("num_ground_vehicles"):
        num_ground = log_reader.header.num_ground_vehicles
    fd.write('\\newcommand{\swarmNumGroundVehicles}{' + str(num_ground) + '}\n')

    if log_reader.header and log_reader.header.HasField("num_fixed_vehicles"):
        num_fixed = log_reader.header.num_fixed_vehicles
    fd.write('\\newcommand{\swarmNumFixedVehicles}{' + str(num_fixed) + '}\n')

    if log_reader.header and log_reader.header.HasField("num_rotor_vehicles"):
        num_rotor = log_reader.header.num_rotor_vehicles
    fd.write('\\newcommand{\swarmNumRotorVehicles}{' + str(num_rotor) + '}\n')

    if log_reader.header and log_reader.header.HasField("terrain_name"):
        terrain_name = log_reader.header.terrain_name
        # Escape '_' to make latex happy.
        terrain_name = terrain_name.replace('_', '\_')
    fd.write('\\newcommand{\swarmTerrainName}{' + terrain_name + '}\n')

    if log_reader.header and log_reader.header.HasField("vegetation_name"):
        vegetation_name = log_reader.header.vegetation_name
        # Escape '_' to make latex happy.
        vegetation_name = vegetation_name.replace('_', '\_')
    fd.write('\\newcommand{\swarmVegetationName}{' + vegetation_name + '}\n')

    if log_reader.header and log_reader.header.HasField("search_area"):
        search_area = log_reader.header.search_area
        # Escape '_' to make latex happy.
        search_area = search_area.replace('_', '\_')
    fd.write('\\newcommand{\swarmSearchArea}{' + search_area + '}\n')

    # Completion.
    total_score = 0.0
    max_duration = 7200
    max_wrong_reports = 20
    if succeed:
        total_score = score(duration, max_duration, wrong_boo_reports, max_wrong_reports)
        fd.write('\\newcommand{\swarmSucceed}{Yes}\n')
    else:
        fd.write('\\newcommand{\swarmSucceed}{No}\n')
    fd.write('\\newcommand{\swarmWrongBooReports}{' + str(wrong_boo_reports) + '}\n')
    fd.write('\\newcommand{\swarmDuration}{' + str(duration) + '}\n')

    if log_reader.header.HasField("max_time_allowed"):
        max_duration = log_reader.header.max_time_allowed
    else:
        print 'Warning: <max_time_allowed> not present in log file.'

    if log_reader.header.HasField("max_wrong_reports"):
        max_wrong_reports = log_reader.header.max_wrong_reports
    else:
        print 'Warning: <max_wrong_reports> not present in log file.'

    fd.write('\\newcommand{\swarmScore}{' + str(total_score) + '}\n')

    # Comms summary.
    fd.write('\\newcommand{\swarmNumMsgsSent}{' + str(total_msgs_sent) + '}\n')
    fd.write('\\newcommand{\swarmNumUnicastSent}{' + str(total_msgs_unicast) + '}\n')
    fd.write('\\newcommand{\swarmNumBroadcastSent}{' + str(total_msgs_broadcast) + '}\n')
    fd.write('\\newcommand{\swarmNumMulticastSent}{' + str(total_msgs_multicast) + '}\n')
    fd.write('\\newcommand{\swarmFreqMsgsSent}{' + str(avg_msgs_sent_freq) + '}\n')
    fd.write('\\newcommand{\swarmAvgMsgsDrop}{' + str(avg_drop_ratio) + '}\n')
    fd.write('\\newcommand{\swarmAvgDataRateRobot}{' + str(avg_data_rate) + '}\n')
    fd.write('\\newcommand{\swarmAvgNeighborsRobot}{' + str(avg_neighbors) + '}\n')
    fd.write('\\newcommand{\swarmMaxDuration}{' + str(max_duration) + '}\n')
    fd.write('\\newcommand{\swarmMaxWrongReports}{' + str(max_wrong_reports) + '}\n')
    fd.close()

def score(duration, max_duration, wrong_reports, max_wrong_reports):
    # Duration
    A = 0.8
    a = A * (1.0 - min(1.0, duration / max_duration))

    # Wrong reports.
    B = 0.2
    b = B * (1.0 - min(1.0, wrong_reports / max_wrong_reports))

    return a + b;

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: swarmlog_to_csv_comms.py <log_file> <summary_file>')
        print('Parameters:')
        print('  <log_file> Swarm log file name to read from')
        print('  <summary_file> Full path to the summary file that will be generated')
        sys.exit(1)
    fname = sys.argv[1]
    reader = LogReader(fname)
    print('# time, msg_sent, msg_freq, num_unicast, num_broadcast, num_multicast, potential_recipients, msgs_delivered, drop_ratio, bytes_sent, data_rate, avg_num_neighbors')
    reader.apply(process_msg)
    create_swarm_summary_report(reader, sys.argv[2])

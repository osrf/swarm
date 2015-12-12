#!/usr/bin/env ruby

# TODO: Fix this!!
$LOAD_PATH.push('/home/nkoenig/local/lib/ruby/swarm')
puts $LOAD_PATH

require 'protobuf'
require '/home/nkoenig/local/lib/ruby/swarm/log_entry.pb'
require '/home/nkoenig/local/lib/ruby/swarm/log_header.pb'

require 'timeout'
require 'fileutils'
require 'optparse'
require 'net/http'

# $ gem install parallel
require 'parallel'

$latexTemplate = %q{
\documentclass{article}

\usepackage{graphicx} % Required for the inclusion of images
\usepackage{amsmath} % Required for some math elements
\usepackage{subfigure}
\usepackage{geometry}
  \geometry{
  left=20mm,
  right=20mm,
  top=20mm,
}

\setlength\parindent{0pt} % Removes all indentation from paragraphs

\renewcommand{\labelenumi}{\alph{enumi}.} % Make numbering in the enumerate environment by letter rather than number (e.g. section 6)

\input{swarm_summary}

\title{Swarm experiment report} % Title

\author{\textsc{Open Source Robotics Foundation}} % Author name

\date{\today} % Date for the report

\begin{document}

\maketitle % Insert the title, author and date

\section*{Configuration}

\begin{tabular}{ll}
Team                           & \swarmTeamName\\\\
Number of ground vehicles      & \swarmNumGroundVehicles\\\\
Number of fixed wing vehicles  & \swarmNumFixedVehicles\\\\
Number of rotor craft vehicles & \swarmNumRotorVehicles\\\\
Terrain name                   & \swarmTerrainName\\\\
Vegetation name                & \swarmVegetationName\\\\
Search area                    & \swarmSearchArea\\\\
\end{tabular}


\section*{Completion}

\begin{tabular}{ll}
Success                                     & \swarmSucceed\\\\
Number of incorrect reports                 & \swarmWrongBooReports\\\\
Maximum number of incorrect reports allowed & \swarmMaxWrongReports\\\\
Duration                                    & \swarmDuration ~seconds\\\\
Maximum duration allowed                    & \swarmMaxDuration ~seconds\\\\
Score                                       & \swarmScore\\\\
\end{tabular}

The score is calculated using the following equation: \\

\begin{center}
\$score=0.8\cdot(1.0-min(1.0,~\frac{duration}{duration_{max}}))+0.2\cdot(1.0-min(1.0, ~\frac{incorrectReports}{incorrectReports_{max}}))$
\end{center}

\section*{Communications}

\begin{tabular}{ll}
Number of messages sent               & \swarmNumMsgsSent\\\\
Number of unicast messages sent       & \swarmNumUnicastSent\\\\
Number of broadcast messages sent     & \swarmNumBroadcastSent\\\\
Number of multicast messages sent     & \swarmNumMulticastSent\\\\
Average message publication frequency & \swarmFreqMsgsSent ~messages per second\\\\
Average percentage message drop       & \swarmAvgMsgsDrop ~\%\\\\
Average data rate per robot           & \swarmAvgDataRateRobot ~mbps\\\\
Average number of neighbors per robot & \swarmAvgNeighborsRobot\\\\
\end{tabular}

\begin{figure}[ht]
  \centering
  \includegraphics[width=\columnwidth, keepaspectratio]{swarm_msgs_sent}
  \label{fig:subfig5}
\end{figure}

\begin{figure}[ht]
  \centering
  \includegraphics[width=\columnwidth, keepaspectratio]{swarm_comms_drops}
  \label{fig:subfig6}
\end{figure}

\begin{figure}[ht]
  \centering
  \includegraphics[width=\columnwidth, keepaspectratio]{swarm_comms_datarate}
  \label{fig:subfig7}
\end{figure}

\begin{figure}[ht]
  \centering
  \includegraphics[width=\columnwidth, keepaspectratio]{swarm_comms_neighbors}
  \label{fig:subfig8}
\end{figure}

\end{document}
}

###############################################
# \brief A class that parses a log file.
class LogParser
  include Enumerable

  def initialize(_logFile)
    @logFile = _logFile
    @file = File.open(@logFile, 'rb')
    @header = nil
  end

  #################################################  
  def nextMsg
    # Read the 4-byte header
    sizeString = @file.read(4)
    if sizeString.length < 4 then
      @file.close
      return nil
    end

    # Get the message size
    size = sizeString.unpack('<I')[0]

    # Read the message, in binary
    msg = @file.read(size)

    # Make a protobuf message out of it
    # The first message in the log is a LogHeader; the rest are LogEntry
    if @header.nil?
      @header = Swarm::Msgs::LogHeader.new
      @header.decode(msg)
      return nextMsg
    end

    pbmsg = Swarm::Msgs::LogEntry.new
    pbmsg.decode(msg)

    return pbmsg
  end

  ###############################################
  # Implement an each message
  def each
    while true do
      msg = nextMsg
      if msg.nil? then
        break
      end
      yield msg
    end
  end
end

###############################################
# \brief A class that runs swarm tests.
class Runner
  @@worldPrefix = "worlds/final_01"
  @@searchAreas = [2, 10, 20]
  @@vehicleCounts = [10, 25, 50, 100, 200, 400, 800]
  @@densities = [0, 1, 2]
  @@densityStrings = ["low", "medium", "high"]

  ###############################################
  # \brief Initialize the runner with options from the command line
  def initialize(options)
    @jobs = options[:jobs]

    @minSearch =  options[:minSearch]
    @maxSearch =  options[:maxSearch]

    @minVehicle = options[:minVehicle]
    @maxVehicle = options[:maxVehicle]

    @minDensity = options[:minDensity]
    @maxDensity = options[:maxDensity]
    @clearLogs = options[:clear]

    @timeout = options[:timeout]
    @team = options[:team]
    @reps = options[:reps]

    @dir = options[:dir]

    if options.has_key?(:reports) then
      generate_reports(options[:reports])
    else
      run_tests
    end
  end


  ###############################################
  # Generate all the runs based on the provided parameters
  def generate_runs
    port = 11345
    @runs = []

    # Create all the runs. The outer loop is the number of repetitions to
    # perform each run
    @reps.times do

      # Iterate over the search areas
      @@searchAreas.each do |searchArea|
        if searchArea < @minSearch || searchArea > @maxSearch then
          next
        end

        # Iterate over the densities
        @@densities.each do |density|
          if density < @minDensity || density > @maxDensity then
            next
          end

          # Iterate over the vehicle counts
          @@vehicleCounts.each do |vehicleCount|
            if vehicleCount < @minVehicle || vehicleCount > @maxVehicle then
              next
            end

            # Generate the shell command
            world = "#{@@worldPrefix}_#{searchArea.to_s.rjust(2, '0')}_" +
                    "#{@@densityStrings[density]}_#{vehicleCount}.world"
            @runs.push("SWARM_LOG=1 SWARM_TEAMNAME=#{@team} " +
                       "GAZEBO_MASTER_URI=http://localhost:#{port} " +
                       "gzserver -r #{world}")
            port += 1
          end
        end
      end
    end
  end

  ###############################################
  # \brief Run all the tests
  def run_tests

    generate_runs

    # Output some useful information
    puts "Parameters:"
    puts "--------------------------------------------------"
    puts " Search area range: #{@minSearch} - #{@maxSearch}"
    puts " Vehicle range:     #{@minVehicle} - #{@maxVehicle}"
    puts " Density range:     #{@minDensity} - #{@maxDensity}"
    puts " Timeout:           #{@timeout}"
    puts " Clear logs:        #{@clearLogs ? "true" : "false"}"
    puts " Processes:         #{@jobs}"
    puts " Output dir:        #{@dir}"
    puts " Team:              #{@team}"
    puts " Repetitions:       #{@reps}"
    puts "--------------------------------------------------"
    puts
    puts "Tests to run:"
    puts @runs
    puts

    # Ask before proceeding
    print "Run tests ? (y/N) "
    ans = gets.chomp
    if ans != 'y' && ans != 'Y' then
      return
    end

    # Get the current time.
    time = Time.now.strftime("%Y-%m-%dT%H:%M:%S")

    # Clear the log directories, if specified on the command line.
    if @clearLogs then
      puts "Clearing logs"
      FileUtils.rm_rf("#{ENV['HOME']}/.swarm/log")
      FileUtils.rm_rf("#{ENV['HOME']}/.gazebo/log")
    end

    puts "Starting tests..."

    # Run the tests in different processes
    Parallel.map(@runs, :in_processes => @jobs) do |run|
      puts "Running test: #{run}"

       # Spawn the job
       pid = Process.spawn(run, {[:err,:out] => :close, :pgroup => true})
       begin
         Timeout.timeout(@timeout) do
           Process.waitpid(pid, 0)
         end
       rescue Timeout::Error
         Process.kill(15, -Process.getpgid(pid))
       end
    end

    # Gather log files into common location
    FileUtils.mkdir("#{@dir}/#{time}")
    FileUtils.cp_r("#{ENV['HOME']}/.swarm/log", "#{@dir}/#{time}/swarm")
    FileUtils.cp_r("#{ENV['HOME']}/.gazebo/log", "#{@dir}/#{time}/gazebo")

    # Generate the reports
    generate_reports("#{@dir}/#{time}/swarm")
  end

  #################################################
  # Generate report for all directories in _path
  def generate_reports(_path)
    # Plots to output
    plots = ["swarm_comms_msgs_sent.gplot",
             "swarm_comms_drops.gplot",
             "swarm_comms_datarate.gplot",
             "swarm_comms_neighbors.gplot"]

    # Get an array of all the reports to generate
    reports = Dir.entries(_path).select{ |entry|
      File.directory?(File.join(_path, entry)) && entry != '.' && entry != '..'
    }.collect{ |entry|
      File.join(_path, entry)
    }

    puts "Generate reports for:\n"
    puts reports

    # Generate reports in different processes
    Parallel.map(reports, :in_processes => @jobs) do |report|
      puts "Generating #{report}"
      swarmLog = "#{report}/swarm.log" 
      swarmSummary = "#{report}/swarm_summary.tex"
      swarmCsv = "#{report}/swarm.csv"

      # Step 1: Create csv and summary files
      `swarmlog_to_csv_comms.py #{swarmLog} #{swarmSummary} > #{swarmCsv}`
     
      # Step 2: Generate images 
      plots.each do |plot|
        `output_dir=#{report} logfile=#{swarmCsv} #{plot}`
      end

      # Step 3: Generate final report
      File.open("#{report}/report.tex", 'w') {|file|
        file.write($latexTemplate)
      }
      Dir.chdir(report) {
        `pdflatex #{report}/report.tex`
      }
    end
  end
end

#################################################
# Main
if __FILE__ == $0

  # Default options
  options = {:minSearch => 0,  :maxSearch => 20,
             :minVehicle => 0, :maxVehicle => 50,
             :minDensity => 0, :maxDensity => 2,
             :clear => false, :timeout => 3600, :jobs => 1,
             :dir => "/tmp", :team => "OSRF", :reps => 1}

  # Process command line options
  OptionParser.new do |opts|
    opts.banner = "Usage: run_swarm.rb [options]"

    opts.on("-c", "--clear-logs", "Clear log directory") do |n|
      options[:clear] = n
    end

    opts.on("-t N", "--timeout N", Integer, "Timeout in seconds") do |n|
      options[:timeout] = n
    end

    opts.on("--min-search N", Integer, "Min search area") do |n|
      options[:minSearch] = n
    end

    opts.on("--max-search N", Integer, "Max search area") do |n|
      options[:maxSearch] = n
    end

    opts.on("--min-vehicles N", Integer, "Min vehicle count") do |n|
      options[:minVehicle] = n
    end

    opts.on("--max-vehicles N", Integer, "Min vehicle count") do |n|
      options[:maxVehicle] = n
    end

    opts.on("--min-density N", Integer, "Min density") do |n|
      options[:minDensity] = n
    end

    opts.on("--max-density N", Integer, "Max density") do |n|
      options[:maxDensity] = n
    end

    opts.on("-j N", "--jobs N", Integer, "Number of processes to use") do |n|
      options[:jobs] = n
    end

    opts.on("-d DIR", "--out-dir DIR", String, "Directory to store logs") do |n|
      options[:dir] = n
    end

    opts.on("--team TEAM", String, "Team name") do |n|
      options[:team] = n
    end

    opts.on("-r N", "--reps N", Integer, "Number of repetitions") do |n|
      options[:reps] = n
    end

    opts.on("-g DIR", "--generate-report DIR", String, "Generate reports for a directory") do |n|
      options[:reports] = n
    end

  end.parse!

  parser = LogParser.new("/data/swarm_logs/2015-12-10T07:50:32/swarm/2015-12-10T07:50:40.110786/swarm.log")

  parser.each do |msg|
    puts msg.to_json
  end 

  # Create the runner
  # runner = Runner.new(options)
end

require 'timeout'
require 'fileutils'
require 'optparse'
require 'parallel'

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

    @minSearch =  options[:minSearch]
    @maxSearch =  options[:maxSearch]

    @minVehicle = options[:minVehicle]
    @maxVehicle = options[:maxVehicle]

    @minDensity = options[:minDensity]
    @maxDensity = options[:maxDensity]
    @clearLogs = options[:clear]

    @timeout = options[:timeout]
    @jobs = options[:jobs]
    @team = options[:team]
    @reps = options[:reps]

    @dir = options[:dir]

    port = 11345
    @runs = []

    # Create all the runs. The outer loop is the number of repetitions to
    # perform each run
    @reps.times do |i|

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
  end

  ###############################################
  # \brief Run all the tests
  def run
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
    results = Parallel.map(@runs, :in_processes => @jobs) do |run|
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

  end.parse!

  # Create the runner
  runner = Runner.new(options)

  # Run the tests.
  print "Run tests ? (y/N) "
  ans = gets.chomp
  if ans == 'y' || ans == 'Y' then
    runner.run
  end
end

require 'orocos'
require 'vizkit'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: run.rb <device serial number>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos::Process.run 'camera_davis::Task' => 'camera_davis', :gdb => false do
    # log all the output ports
    Orocos.log_all_ports 
    Orocos.conf.load_dir('../config')

    # Get the task
    driver = Orocos.name_service.get 'camera_davis'
    Orocos.conf.apply(driver, ['default','DAVIS-84010059'], :override => true)

    driver.serial_number = ARGV[0]

    driver.configure
    driver.start

    Vizkit.exec
end

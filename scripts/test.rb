require 'orocos'
require 'vizkit'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <device name>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos::Process.run 'imu_myahrs_plus::Task' => 'imu_myahrs_plus' do
    # log all the output ports
    Orocos.log_all_ports 

    # Get the task
    driver = Orocos.name_service.get 'imu_myahrs_plus'

    driver.port = ARGV[0]
    driver.baudrate = 115200

    driver.configure
    driver.start

    # Orientation visualization
    attitudeRBS = Vizkit.default_loader.RigidBodyStateVisualization
    attitudeRBS.setPluginName("Attitude")
    attitudeRBS.resetModel(0.4)

    Vizkit.display driver.port('orientation_samples_out'), :widget =>attitudeRBS


    ## Create a widget for 3d display
    view3d = Vizkit.vizkit3d_widget

    # Show it
    view3d.show

    Vizkit.exec

end

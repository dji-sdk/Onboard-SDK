# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  config.vm.box = "boxcutter/ubuntu1604-desktop"
  config.vm.provider "virtualbox" do |vb|
    vb.memory = "8192"
    vb.cpus = 4
    vb.gui = true
    vb.customize ["modifyvm", :id, "--usb", "on"]
    vb.customize ["usbfilter", "add", "0", 
      "--target", :id, 
      "--name", "Prolific Technology Inc. USB-Serial Controller [0300]",
      "--manufacturer", "Prolific Technology Inc.",
      "--product", "USB-Serial Controller"]
  end

  config.vm.provision "shell", path: "scripts/bootstrap.sh"
end

# Using the PR2

## Starting 
When plugging/unplugging there should be no current in the cable:
- plugging : first PR2, then wall
- unplugging : first wall, then PR2

Start the red switch. Wait until the computers started, you will hear to series of ascending tones.

The main computer is c1. You can ssh to it with `ssh pr2admin@134.157.19.192`. If doesn't work look for the ip of c1.
From the pr2 computer, you can access others pr2 computers with :
```bash
ssh pr2admin@c1
ssh pr2admin@c2
ssh pr2-head@pr2-head
```

## Configuring 

To see the topic from the pr2 on your computer.

### On your computer

Set the ROS master which is on c1 and set your ip (under *eth0* from `ifconfig`)
```bash
export ROS_MASTER_URI=http://134.157.19.192:11311
export ROS_IP=<your-ip>
```

Add routes.
```bash
sudo route add default gw 134.157.19.192
```

Make sure that the `/etc/hosts` has the following:
```
10.68.0.1 c1
10.68.0.2 c2
10.68.0.10 pr2-head
```

### On c1 

Set c1 ip on network.
```bash
export ROS_IP=134.157.19.192
```

Add routes.
```bash
sudo route add -net 134.157.19.0 netmask 255.255.255.0 gw 134.157.19.192
sudo route add -net 10.68.0.0 netmask 255.255.255.0 gw 10.68.0.1
```

### On c2 

Add routes.
```bash
sudo route add default gw 10.68.0.1
```

### On head

Add routes.
```bash
sudo route add default gw 10.68.0.1
```

## Running

Not available.

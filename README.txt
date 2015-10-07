NET-SYNC is a short for A Real-Time Implementation of Precise Timestamp-Free Network Synchronization

ABSTRACT
Real-time implementation of precise timestamp-free network synchronization is using audio-frequency signaling between a master and slave node. Rather than the conventional approach of exchanging digital timestamps through a dedicated synchronization protoco, timestamp-free synchronization is performed implicitly at the physical layer through the timing of the master node’s responses to the slave node. The master and slave nodes were both implemend on Texas Instruments digital signal processing boards (TMS320C6713) with real-time software implemented in C. Preliminary experimental results using modulated 100 Hz bandwidth sinc pulses demonstrate synchronization accuracies better than 0.33 us.

[READ FULL PAPER](http://spinlab.wpi.edu/publications.html)(available after the conference date November 8-11, 2015)

To switch between master and slave change the define in the "time_stamper_master.c".
//Board node definitions
#define MASTER_NODE  1
#define SLAVE_NODE   2
//Node type - This changes whether setting
#define NODE_TYPE MASTER_NODE

The states relate to the image below. Node i is slave, node j is master. Reception (both master and slave) of sinc pulse requires SEARCHING and RECORDING states. Master's transmittion uses CALCULATION and TRANSMIT states. Slave's transmission happens in SENDSINC state.
// State definitions
#define STATE_SEARCHING 0
#define STATE_RECORDING 1
#define STATE_CALCULATION 2
#define STATE_TRANSMIT 3
#define STATE_SENDSINC 4

![sinc pulse exchange](https://cloud.githubusercontent.com/assets/6517379/10353835/0c7e70ec-6d28-11e5-9fd2-2c3349e79b41.png)



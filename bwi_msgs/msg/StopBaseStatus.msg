### Stop base controller status.

uint8 status

# Constants defining the possible states:
uint8 RUNNING = 0       # robot base running normally
uint8 PAUSED =  1       # robot base motion paused
uint8 STOPPED = 2       # terminal state: no more movement allowed

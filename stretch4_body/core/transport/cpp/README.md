#INSTALL NOTES  
 cd readerwriterqueue/
  662  cmake .
  663  make
  666  sudo make install
  667  cd ..
  669  make
  670  make install



# Lock-free queue between thread (C++) + thread (Python binding to C++)

This method involves lock-free queues between two threads (that can be scheduled between multiple cores, similar to processes, but cheaper to context-switch). The transport thread reads a command queue / writes to status queue, and the body thread (main thread) does vice-versa. The queue implementation is provided by [Cameron](https://github.com/cameron314/readerwriterqueue). The Python bindings are provided through ctypes.

Whereas previous experiments synchronize a Command -> Status -> Think loop (hence one number measures loop FPS), this method frees transport to send commands as quick as it can, body to process statuses as quick as it can, and behaviors to loop without blocking (hence three numbers measure performance).

## Setup

 1. `git submodule update --init`

 2. `make`

 3. `python3 body.py`

# Variations

 - independent queues
 - lockstep queues

# CI (Continuous integration)

This folder contains all the files necessary to carry out CI mode using TCP to connect to the game controller (GC).

You need to get the ball information, robot information and geometry information. Then you need to send this as a packaged protobuf file to the GC. The "old" protobuf stuff to get information out of the system still uses the old protobuf files.

However we use these new files to establish a TCP connection using CI.
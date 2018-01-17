all:
	protoc --cpp_out=. Hand.proto
	c++ client.cc  Hand.pb.cc -o client `pkg-config --cflags --libs protobuf` -lwiringPi 
	c++ server.cc  Hand.pb.cc -o server `pkg-config --cflags --libs protobuf` -lwiringPi -LSM9DS
	g++ -o hand hand.cpp -lwiringPi
	g++ -o glove glove.cpp -lwiringPi -LSM9DS
#	protoc --cpp_out=. Employee.proto
#	g++ -g -Wall server.cc Employee.pb.cc -o server -lprotobuf
#	g++ -g -Wall client.cc Employee.pb.cc -o client -lprotobuf 


clean:
	rm server client Hand.pb.cc Hand.pb.h

